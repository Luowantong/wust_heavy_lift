#include "mtc/trajectory_executor.hpp"
#include "mtc/common/task_node_base.hpp"
#include "mtc/common/tcp_hollow.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/task_constructor/storage.h>
#include <algorithm>
#include <cmath>
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
  // 只给 place 用：pick 的取件位是固定的，不能被兑换站位姿覆盖
  if (allow_slot_override_ && target_prefix_ == "target_slot") {
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

bool TaskNodeBase::stationUnoccluded(const geometry_msgs::msg::Pose &target) {
  try {
    const auto tf = tf_buffer_->lookupTransform(
        planning_frame_, hand_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.05));
    const auto &t = tf.transform.translation;
    const double d = std::hypot(std::hypot(t.x - target.position.x,
                                           t.y - target.position.y),
                                t.z - target.position.z);
    if (d >= target_lock_min_distance_)
      return true;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "target sample skipped: hand %.2fm from station (occluded)", d);
  } catch (const tf2::TransformException &e) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "target sample skipped: no TF %s <- %s (%s)",
                         planning_frame_.c_str(), hand_frame_.c_str(), e.what());
  }
  return false;
}

void TaskNodeBase::onTargetOverride(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // 锁定之后后续帧全部忽略：站位物理上是固定的，感知读数的变化几乎全是
  // 遮挡产物（实测单元挡在站体前时偏 +9/-9mm，放好之后降到 ~6mm）。
  // 继续跟随只会把噪声带进目标。
  if (target_lock_ && target_locked_) {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "target already locked, override ignored");
    return;
  }

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

  // 只覆盖位置，姿态继续用 YAML 里配的。
  // detector 发的是标记块自身坐标系的 solvePnP 朝向，和站体 E 系（x 沿柱子、
  // z 朝上）没有对齐；而 MTC 的插入方向 -R.col(0)、顶起方向 R.col(2) 都直接
  // 取 target_slot_.orientation，用视觉姿态会把方向带偏。
  // 如果以后确认视觉四元数就是 E 的朝向，把下面一行换成 target_slot_ = p; 即可。

  if (!target_lock_) {  // 旧行为：来什么用什么
    target_slot_.pose.position = p.pose.position;
    target_slot_.header = p.header;
    has_target_ = true;
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                         "target overridden (position only): [%.3f %.3f %.3f]",
                         target_slot_.pose.position.x, target_slot_.pose.position.y,
                         target_slot_.pose.position.z);
    return;
  }

  // ---- 锁定模式：遮挡判据 → 一致性门限 → 与 YAML 的偏移门限 ----
  if (!stationUnoccluded(p.pose))
    return;

  target_samples_.emplace_back(p.pose.position.x, p.pose.position.y,
                               p.pose.position.z);
  while (target_samples_.size() > static_cast<size_t>(std::max(1, target_lock_frames_)))
    target_samples_.pop_front();
  if (target_samples_.size() < static_cast<size_t>(std::max(1, target_lock_frames_)))
    return;

  tf2::Vector3 mean(0, 0, 0);
  for (const auto &s : target_samples_)
    mean += s;
  mean /= static_cast<double>(target_samples_.size());

  double spread = 0.0;
  for (const auto &s : target_samples_)
    spread = std::max(spread, (s - mean).length());
  if (spread > target_lock_max_spread_)
    return;  // 还在抖，继续攒

  // 与 YAML 真值偏离过大 → 宁可不锁，也不要用一个明显错的值。
  // 锁不上就一直用 YAML，任务照常能跑。
  const tf2::Vector3 yaml_pos(yaml_slot_.pose.position.x,
                              yaml_slot_.pose.position.y,
                              yaml_slot_.pose.position.z);
  const double offset = (mean - yaml_pos).length();
  if (offset > target_lock_max_offset_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "target sample rejected: %.1fmm from YAML (limit %.1fmm)",
                         offset * 1000, target_lock_max_offset_ * 1000);
    return;
  }

  target_slot_.pose.position.x = mean.x();
  target_slot_.pose.position.y = mean.y();
  target_slot_.pose.position.z = mean.z();
  target_slot_.header = p.header;
  has_target_ = true;
  target_locked_ = true;
  RCLCPP_INFO(node_->get_logger(),
              "target LOCKED at [%.3f %.3f %.3f] (spread %.1fmm, offset %.1fmm from YAML)",
              mean.x(), mean.y(), mean.z(), spread * 1000, offset * 1000);
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
      setState(State::FAILED);   // 否则状态永远停在 PLANNING
      task_running_ = false;
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Planning pipeline worker unknown exception");
      setState(State::FAILED);
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
    // 失败过的 Task 内部状态不可信，直接重建。
    // createTask() 里会抛"目标相关"的异常（例如 pre-place IK 所有种子都失败），
    // 这类失败和"规划找不到解"性质相同，必须计入重试次数；
    // 之前它抛在 try 外面，一次 IK 失败就把整个任务打死，plan_attempts 形同虚设。
    try {
      task_ = createTask();
      task_.enableIntrospection();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(node_->get_logger(), "createTask() attempt %d/%d failed: %s",
                   attempt, max_attempts, e.what());
      continue;
    } catch (...) {
      RCLCPP_ERROR(node_->get_logger(),
                   "createTask() attempt %d/%d failed: unknown exception",
                   attempt, max_attempts);
      continue;
    }

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
  // 目标坐标统一由模型 yaml 的 /** 段提供：
  //   pick 读 workcell.object，place 读 workcell.slot。
  // MuJoCo 的 GraspPlugin 也从同一组参数里取，保证规划场景和仿真实物一致。
  const std::string role =
      (target_prefix_ == "target_object") ? "object" : "slot";
  const std::string prefix = "workcell." + role;

  const double x     = node_->get_parameter_or(prefix + ".x", 0.0);
  const double y     = node_->get_parameter_or(prefix + ".y", 0.0);
  const double z     = node_->get_parameter_or(prefix + ".z", 0.0);
  const double roll  = node_->get_parameter_or(prefix + ".roll", 0.0);
  const double pitch = node_->get_parameter_or(prefix + ".pitch", 0.0);
  const double yaw   = node_->get_parameter_or(prefix + ".yaw", 0.0);

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);

  target_slot_.header.frame_id = planning_frame_;
  target_slot_.pose.position.x = x;
  target_slot_.pose.position.y = y;
  target_slot_.pose.position.z = z;
  target_slot_.pose.orientation = tf2::toMsg(q);
  has_target_ = true;
  // 存档：后面 onTargetOverride 会覆盖 target_slot_，这里留一份 YAML 真值。
  // 锁定的一致性门限、以及 IK 全败时的降级目标都用它。
  yaml_slot_ = target_slot_;

  allow_slot_override_ = node_->get_parameter_or("allow_slot_override", false);
  target_lock_         = node_->get_parameter_or("target_lock", false);
  target_lock_frames_  = node_->get_parameter_or("target_lock_frames", 10);
  target_lock_max_spread_   = node_->get_parameter_or("target_lock_max_spread", 0.002);
  target_lock_min_distance_ = node_->get_parameter_or("target_lock_min_distance", 0.35);
  target_lock_max_offset_   = node_->get_parameter_or("target_lock_max_offset", 0.015);
  auto_start_          = node_->get_parameter_or("auto_start", false);
  auto_start_delay_    = node_->get_parameter_or("auto_start_delay", 3.0);

  RCLCPP_INFO(node_->get_logger(),
              "workcell.%s=[%.3f %.3f %.3f] rpy=[%.3f %.3f %.3f] "
              "override=%s auto_start=%s",
              role.c_str(), x, y, z, roll, pitch, yaw,
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
