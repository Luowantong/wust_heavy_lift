#include "mtc/trajectory_executor.hpp"
#include "mtc/common/task_node_base.hpp"
#include "mtc/common/tcp_hollow.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/task_constructor/storage.h>
#include <functional>
#include <vector>

namespace mtc {
TaskNodeBase::TaskNodeBase(const rclcpp::NodeOptions &options,
                           const std::string &node_name,
                           const std::string &target_prefix)
    : target_prefix_(target_prefix) {
  /* ---------- node ---------- */
  node_ = std::make_shared<rclcpp::Node>(node_name, options);

  /* ---------- parameters ---------- */
  arm_group_      = node_->get_parameter_or("arm_group", std::string("main"));
  hand_frame_     = node_->get_parameter_or("hand_frame", std::string("LINK_TCP"));
  planning_frame_ = node_->get_parameter_or("planning_frame", std::string("base_link"));

  loadTargetParams();  // 读 target_slot.* / auto_start / auto_start_delay

  /* ---------- tf ---------- */
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  /* ---------- callback groups ---------- */
  cb_group_task_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_target_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  /* ---------- interfaces ---------- */
  state_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(
      "~/state", rclcpp::QoS(1).transient_local());

  plan_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/plan_and_execute",
      std::bind(&TaskNodeBase::onPlanAndExecute, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_task_);

  const double th_roll  = node_->get_parameter_or("tcp_hollow.roll",  0.0);
  const double th_pitch = node_->get_parameter_or("tcp_hollow.pitch", -M_PI / 2);
  const double th_yaw   = node_->get_parameter_or("tcp_hollow.yaw",   0.0);
  const double th_x     = node_->get_parameter_or("tcp_hollow.x",     0.0);
  const double th_y     = node_->get_parameter_or("tcp_hollow.y",     0.0);
  const double th_z     = node_->get_parameter_or("tcp_hollow.z",     0.0);

  T_tcp_hollow_ = makeTcpHollow(th_roll, th_pitch, th_yaw, th_x, th_y, th_z);

  RCLCPP_INFO(node_->get_logger(),
              "T_tcp_hollow rpy=[%.4f %.4f %.4f] xyz=[%.4f %.4f %.4f]",
              th_roll, th_pitch, th_yaw, th_x, th_y, th_z);
  
  // 运行时临时覆盖 YAML 里的目标（默认关闭，需 allow_slot_override:=true）
  if (allow_slot_override_) {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_target_;
    slot_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/exchange_slot/slot_pose", 10,
        std::bind(&TaskNodeBase::onTargetOverride, this, std::placeholders::_1),
        sub_opts);
  }

  /* ---------- auto start ---------- */
  if (auto_start_) {
    auto_start_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(auto_start_delay_),
        [this]() {
          auto_start_timer_->cancel();  // 只跑一次
          RCLCPP_INFO(node_->get_logger(), "auto_start: triggering task");
          startPlanningPipelineAsync();
        },
        cb_group_task_);
  }

  setState(State::IDLE);
  RCLCPP_INFO(node_->get_logger(), "MTCTaskNode constructed");
}

/* ===================== callbacks ===================== */
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TaskNodeBase::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

void TaskNodeBase::onTargetOverride(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  geometry_msgs::msg::PoseStamped p = *msg;
  if (p.header.frame_id.empty())
    p.header.frame_id = planning_frame_;

  if (p.header.frame_id != planning_frame_) {
    try {
      p = tf_buffer_->transform(p, planning_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN(node_->get_logger(), "target override dropped: %s", e.what());
      return;
    }
  }

  target_slot_ = p;
  has_target_ = true;
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                       "target overridden: [%.3f %.3f %.3f]",
                       p.pose.position.x, p.pose.position.y, p.pose.position.z);
}

TaskNodeBase::~TaskNodeBase() {
  // rclcpp 已 shutdown 后阻塞调用会很快返回，这里等待工作线程结束，
  // 避免节点析构时仍有线程访问成员导致 use-after-free / 退出卡死。
  if (pipeline_thread_.joinable())
    pipeline_thread_.join();
}
void TaskNodeBase::startPlanningPipelineAsync() {
  // 单飞：已有管线在执行则忽略本次触发
  if (task_running_.exchange(true))
    return;
  // 上一个工作线程若已结束但尚未 join，先回收（此处只可能在已结束时进入）
  if (pipeline_thread_.joinable())
    pipeline_thread_.join();
  // 完整规划/执行管线放到独立线程，避免长时间阻塞 executor（timer / 服务 / 订阅）
  pipeline_thread_ = std::thread([this]() {
    try {
      startPlanningPipeline();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Planning pipeline worker exception: %s", e.what());
      task_running_ = false;
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Planning pipeline worker unknown exception");
      task_running_ = false;
    }
  });
}
void TaskNodeBase::startPlanningPipeline() {
  // task_running_ 已由 startPlanningPipelineAsync() 置位（单飞保证），
  // 本函数只负责执行管线主体，并在所有出口复位 task_running_。
  if (!has_target_) {
    RCLCPP_ERROR(node_->get_logger(), "No target pose (target_slot.*)");
    setState(State::FAILED);
    task_running_ = false;
    return;
  }

  // 检查 MoveGroup 服务是否在线 ---
  auto get_scene_client =
      node_->create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene");
  if (!get_scene_client->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_DEBUG(node_->get_logger(), "MoveGroup not ready, resetting flag...");
    setState(State::FAILED);
    task_running_ = false; // 复位
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Start PLANNING");

  setState(State::PLANNING);

  setupPlanningScene(target_slot_);         

  const int max_attempts = static_cast<int>(
      node_->get_parameter_or("plan_attempts", 3));
  bool planned = false;

  for (int attempt = 1; attempt <= max_attempts && !planned; ++attempt) {
    // 失败过的 Task 内部状态不可信，直接重建
    task_ = createTask();
    task_.enableIntrospection();

    try {
      task_.init();
    } catch (const moveit::task_constructor::InitStageException &e) {
      RCLCPP_ERROR(node_->get_logger(), "MTC Task init failed: %s", e.what());
      std::cout << task_ << std::endl;
      setState(State::FAILED);
      task_running_ = false;
      return;
    }

    planned = static_cast<bool>(task_.plan(1));
    if (!planned)
      RCLCPP_WARN(node_->get_logger(), "planning attempt %d/%d failed",
                  attempt, max_attempts);
  }

  if (!planned) {
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed after %d attempts",
                 max_attempts);
    setState(State::FAILED);
    task_running_ = false;
    return;
  }

  auto solution = task_.solutions().front();

  // Debug：可视化
  task_.introspection().publishSolution(*solution);
  // std::cout << task_ << std::endl; // debug

  setState(State::EXECUTING);

  TrajectoryExecutor executor(node_, arm_group_, "hollow_cylinder", hand_frame_);
  const auto result = executor.execute(*solution);

  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    setState(State::DONE);
  else
    setState(State::FAILED);

  task_running_ = false;

}

/* ===================== trigger & state ===================== */

void TaskNodeBase::onPlanAndExecute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  if (!has_target_) {
    response->success = false;
    response->message = "no target pose";
    return;
  }
  if (task_running_) {
    response->success = false;
    response->message = "busy";
    return;
  }
  startPlanningPipelineAsync();
  response->success = true;
  response->message = "accepted"; // 结果看 ~/state
}

void TaskNodeBase::loadTargetParams() {
  const double x     = node_->get_parameter_or(target_prefix_+".x", 0.0);
  const double y     = node_->get_parameter_or(target_prefix_+".y", 0.0);
  const double z     = node_->get_parameter_or(target_prefix_+".z", 0.0);
  const double roll  = node_->get_parameter_or(target_prefix_+".roll", 0.0);
  const double pitch = node_->get_parameter_or(target_prefix_+".pitch", 0.0);
  const double yaw   = node_->get_parameter_or(target_prefix_+".yaw", 0.0);

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  target_slot_.header.frame_id = planning_frame_;
  target_slot_.pose.position.x = x;
  target_slot_.pose.position.y = y;
  target_slot_.pose.position.z = z;
  target_slot_.pose.orientation = tf2::toMsg(q);
  has_target_ = true;

  allow_slot_override_ = node_->get_parameter_or("allow_slot_override", false);
  auto_start_          = node_->get_parameter_or("auto_start", false);
  auto_start_delay_    = node_->get_parameter_or("auto_start_delay", 3.0);

  RCLCPP_INFO(node_->get_logger(),
              "target_slot=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f] "
              "override=%s auto_start=%s",
              x, y, z, roll, pitch, yaw,
              allow_slot_override_ ? "true" : "false",
              auto_start_ ? "true" : "false");
}

void TaskNodeBase::setState(State s) {
  state_.store(s);
  static const char *names[] = {"IDLE", "PLANNING", "EXECUTING", "DONE", "FAILED"};
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(s);
  state_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "State -> %s", names[static_cast<int>(s)]);
}

void TaskNodeBase::publishDebugTF(const tf2::Transform &T,
                                  const std::string &name) {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = node_->now();
  t.header.frame_id = planning_frame_;
  t.child_frame_id = name;
  t.transform = tf2::toMsg(T);

  tf_broadcaster_->sendTransform(t);
}

} // namespace mtc
