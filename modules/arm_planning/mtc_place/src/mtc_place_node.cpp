#include "mtc_place/mtc_place_node.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mtc_place {
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options) {
  /* ---------- node ---------- */
  node_ = std::make_shared<rclcpp::Node>("mtc_place_node", options);

  /* ---------- parameters ---------- */
  node_->declare_parameter("arm_group", "main");
  node_->declare_parameter("hand_frame", "LINK_TCP");

  arm_group_ = node_->get_parameter("arm_group").as_string();
  hand_frame_ = node_->get_parameter("hand_frame").as_string();

  /* ---------- callback_group ---------- */
  cb_group_service_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_timer_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  /* ---------- interfaces ---------- */
  slot_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/exchange_slot/slot_pose", 10,
      std::bind(&MTCTaskNode::slotPoseCallback, this, std::placeholders::_1));
  get_state_client_ = node_->create_client<GetDetectorState>(
      "/detector/get_state", rmw_qos_profile_services_default,
      cb_group_service_);

  set_state_client_ = node_->create_client<SetDetectorState>(
      "/detector/set_state", rmw_qos_profile_services_default,
      cb_group_service_);
  RCLCPP_INFO(node_->get_logger(), "MTCTaskNode constructed");
  task_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500),
                                         std::bind(&MTCTaskNode::doTask, this),
                                         cb_group_timer_);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
}
/* ===================== planning scene ===================== */

void MTCTaskNode::setupPlanningScene(
    const geometry_msgs::msg::PoseStamped &slot_pose_stamped) {
  moveit::planning_interface::PlanningSceneInterface psi;
  // 对于hollow_cylinder，先detach再remove
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.object.id = "hollow_cylinder";
  detach.object.operation = detach.object.REMOVE;
  psi.applyAttachedCollisionObject(detach);
  // 清理场景
  psi.removeCollisionObjects({"slot_base", "slot_cylinder", "hollow_cylinder"});
  // ---------- 确认 slot_pose_stamped 已经在 base_link 下 ----------
  geometry_msgs::msg::PoseStamped slot_pose = slot_pose_stamped;
  slot_pose.header.frame_id = "base_link"; // 强制 frame_id 为 base_link

  // ---------- 1. 底座碰撞体 ----------
  moveit_msgs::msg::CollisionObject base;
  base.id = "slot_base";
  base.header.frame_id = "base_link";
  base.primitives.resize(1);
  base.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  base.primitives[0].dimensions = {slot_base_square_size, slot_base_square_size,
                                   slot_base_height};
  base.pose = slot_pose.pose;
  base.operation = base.ADD;
  psi.applyCollisionObject(base);

  // ---------- 2. 圆柱碰撞体 ----------
  moveit_msgs::msg::CollisionObject cylinder;
  cylinder.id = "slot_cylinder";
  cylinder.header.frame_id = "base_link";
  cylinder.primitives.resize(1);
  cylinder.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  cylinder.primitives[0].dimensions = {slot_cylinder_height,
                                       slot_cylinder_radius};

  // 使用 tf2::Transform 计算圆柱相对于底座的偏移，避免坐标系偏移导致的误差
  tf2::Transform tf_base, tf_offset, tf_cylinder;
  tf2::fromMsg(slot_pose.pose, tf_base); // 底座位姿
  tf_offset.setIdentity();
  tf_offset.setOrigin(tf2::Vector3(
      0, 0, slot_base_height / 2.0 + slot_cylinder_height / 2.0)); // z 偏移
  tf_cylinder = tf_base * tf_offset; // 底座+偏移

  cylinder.pose = tfToPose(tf_cylinder);
  cylinder.operation = cylinder.ADD;
  psi.applyCollisionObject(cylinder);

  // ---------- 3. 附着 hollow_cylinder ----------
  moveit_msgs::msg::CollisionObject hollow;
  hollow.id = "hollow_cylinder";
  hollow.header.frame_id = hand_frame_;

  hollow.primitives.resize(1);
  hollow.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  hollow.primitives[0].dimensions = {hollow_cylinder_h, hollow_cylinder_r};

  tf2::Transform tf_hollow;
  /* 计算位移：
     1. X 方向：移动 0.0366，使圆柱中心对齐 LINK7 的 Z 轴。
     2. Y 方向：0。
     3. Z 方向：从 hand_frame_ 往回退，直到靠近 LINK7。
        距离为 0.081603。若要相切，中心点应在 0.081603 - hollow_cylinder_r 处。
        但为了保险（防止碰撞检测过敏），我们先移到 -0.05 附近。
  */
  double offset_x = 0.0366;
  double offset_z =
      -(0.081603 - hollow_cylinder_r); // 往回移动，使侧面与 LINK7 平面相切

  tf_hollow.setOrigin(tf2::Vector3(offset_x, 0, offset_z));

  // 旋转：让圆柱长轴沿 Y 轴
  tf2::Quaternion q;
  q.setRPY(M_PI / 2.0, 0, 0);
  tf_hollow.setRotation(q);
  // 保存 TCP 到 hollow 的变换
  T_tcp_hollow_ = tf_hollow;

  hollow.pose = tfToPose(tf_hollow);
  hollow.operation = hollow.ADD;

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_->get_clock()->now();
  t.header.frame_id = hand_frame_; // 父坐标系
  t.child_frame_id = "hollow_cylinder";
  t.transform.translation.x = tf_hollow.getOrigin().x();
  t.transform.translation.y = tf_hollow.getOrigin().y();
  t.transform.translation.z = tf_hollow.getOrigin().z();
  t.transform.rotation = tf2::toMsg(tf_hollow.getRotation());
  tf_broadcaster_->sendTransform(t);

  // 附着
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object = hollow;
  attached.link_name = hand_frame_;
  attached.touch_links = {hand_frame_, "LINK7"}; // 与手爪和LINK7允许接触
  psi.applyAttachedCollisionObject(attached);

  // 等待附着生效
  std::map<std::string, moveit_msgs::msg::AttachedCollisionObject> objects;
  int attempts = 0;
  while (objects.find("hollow_cylinder") == objects.end() && attempts < 5) {
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    objects = psi.getAttachedObjects({"hollow_cylinder"});
    attempts++;
  }
  RCLCPP_DEBUG(node_->get_logger(),
               "Object 'hollow_cylinder' is now attached and visible.");
}

/* ===================== executor interface ===================== */

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
MTCTaskNode::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

/* ===================== callbacks ===================== */

void MTCTaskNode::slotPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  latest_slot_pose_ = *msg;

  RCLCPP_DEBUG(node_->get_logger(), "Received slot pose");
}

/* ===================== main task entry ===================== */
void MTCTaskNode::handleGetStateResponse(
    rclcpp::Client<GetDetectorState>::SharedFuture future) {
  RCLCPP_DEBUG(node_->get_logger(), "Received state response from detector!");
  uint8_t state = future.get()->state;

  if (state != static_cast<uint8_t>(State::LOCKED))
    return;

  startPlanningPipelineAsync();
  RCLCPP_DEBUG(node_->get_logger(), "Got detector state response");
}
MTCTaskNode::~MTCTaskNode() {
  // rclcpp 已 shutdown 后阻塞调用会很快返回，这里等待工作线程结束，
  // 避免节点析构时仍有线程访问成员导致 use-after-free / 退出卡死。
  if (pipeline_thread_.joinable())
    pipeline_thread_.join();
}
void MTCTaskNode::startPlanningPipelineAsync() {
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
void MTCTaskNode::startPlanningPipeline() {
  // task_running_ 已由 startPlanningPipelineAsync() 置位（单飞保证），
  // 本函数只负责执行管线主体，并在所有出口复位 task_running_。
  // 检查 MoveGroup 服务是否在线 ---
  auto get_scene_client =
      node_->create_client<moveit_msgs::srv::GetPlanningScene>(
          "/get_planning_scene");
  if (!get_scene_client->wait_for_service(std::chrono::milliseconds(500))) {
    RCLCPP_DEBUG(node_->get_logger(), "MoveGroup not ready, resetting flag...");
    task_running_ = false; // 复位
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Start PLANNING");

  setDetectorState(static_cast<uint8_t>(State::PLANNING));

  setupPlanningScene(latest_slot_pose_);
  task_ = createTask();
  task_.enableIntrospection();

  try {
    task_.init();
    RCLCPP_DEBUG(node_->get_logger(), "MTC Task init successfully");
  } catch (const moveit::task_constructor::InitStageException &e) {
    RCLCPP_ERROR(node_->get_logger(), "MTC Task init failed: %s", e.what());
    std::cout << task_ << std::endl; // debug
    task_running_ = false;
    return;
  }

  if (!task_.plan(5)) {
    task_running_ = false;
    setDetectorState(static_cast<uint8_t>(State::LOCKED));
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed!");
    std::cout << task_ << std::endl; // debug
    return;
  }

  auto solution = task_.solutions().front();

  // Debug：可视化
  task_.introspection().publishSolution(*solution);
  // std::cout << task_ << std::endl; // debug

  setDetectorState(static_cast<uint8_t>(State::EXECUTING));
  auto result = task_.execute(*solution);

  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    setDetectorState(static_cast<uint8_t>(State::LOST));
  else
    setDetectorState(static_cast<uint8_t>(State::LOCKED));

  task_running_ = false;
}
void MTCTaskNode::doTask() {
  if (task_running_)
    return;

  // 检查服务是否在线
  if (!get_state_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                         "Detector Service not ready...");
    return;
  }

  auto req = std::make_shared<GetDetectorState::Request>();
  get_state_client_->async_send_request(
      req, std::bind(&MTCTaskNode::handleGetStateResponse, this,
                     std::placeholders::_1));
}

/* ===================== MTC ===================== */
moveit::task_constructor::Task MTCTaskNode::createTask() {
  using namespace moveit::task_constructor;
  Task task;
  task.stages()->setName("place_hollow_cylinder");

  task.loadRobotModel(node_);
  task.setProperty("group", arm_group_);
  task.setProperty("ik_frame", hand_frame_);

  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node_,"ompl");
  pipeline->setTimeParameterization(
    std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());
  pipeline->setPlannerId("RRTstarkConfigDefault");
  
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  // 笛卡尔路径每 3mm 一个 waypoint
  cartesian->setStepSize(0.003);
  cartesian->setTimeParameterization(
    std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());
  cartesian->setMaxVelocityScalingFactor(1.0);
  cartesian->setMaxAccelerationScalingFactor(1.0);
  // 1. 获取当前状态
  Stage *current_state_ptr = nullptr;
  {
    auto cs = std::make_unique<stages::CurrentState>("current");
    current_state_ptr = cs.get();
    task.add(std::move(cs));
  }

  // 2. 连接阶段：从当前位置移动到预放置点上方
  {
    auto connect = std::make_unique<stages::Connect>(
        "move_to_place_area",
        stages::Connect::GroupPlannerVector{{arm_group_, pipeline}});
    connect->setTimeout(5.0);
    connect->properties().configureInitFrom(Stage::PARENT);
    task.add(std::move(connect));
  }
  

  // 3. 放置串行容器
  {
    auto place = std::make_unique<SerialContainer>("place_container");
    task.properties().exposeTo(place->properties(), {"group", "ik_frame"});
    // 3.1 生成预放置姿态
    {
      auto alt = std::make_unique<Alternatives>("ik_yaw_alternative");
      place->properties().exposeTo(alt->properties(), {"group", "ik_frame"});
      tf2::Transform base_T_tcp, base_T_hollow;
      if (!computeTargetTCP(base_T_tcp, base_T_hollow))
        throw std::runtime_error("target pose compute failed");

      publishDebugTF(base_T_hollow, "T_target_hollow");
      publishDebugTF(base_T_tcp, "T_target_tcp");

      // yaw sampling 参数
      int n_yaw = 6;               // 备选yaw 个数
      double yaw_range = M_PI / 2; // 备选yaw 范围
      tf2::Transform hollow_T_tcp = T_tcp_hollow_.inverse();
      for (int i = 0; i < n_yaw; ++i) {
        double yaw = -yaw_range + i * (2 * yaw_range) / (n_yaw - 1);
        // hollow yaw rotation
        tf2::Transform hollow_rot;
        hollow_rot.setIdentity();
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        hollow_rot.setRotation(q);
        // new hollow pose
        tf2::Transform base_T_hollow_yaw = base_T_hollow * hollow_rot;
        // recompute TCP
        tf2::Transform base_T_tcp_yaw = base_T_hollow_yaw * hollow_T_tcp;
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "base_link";
        tf2::toMsg(base_T_tcp_yaw, pose.pose);
        publishDebugTF(base_T_hollow_yaw,
                       "T_target_hollow_yaw_" + std::to_string(i));
        // GeneratePose
        auto gen = std::make_unique<stages::GeneratePose>("gen_pose_yaw_" +
                                                          std::to_string(i));
        gen->setPose(pose);
        gen->setMonitoredStage(current_state_ptr);
        // ComputeIK
        auto ik = std::make_unique<stages::ComputeIK>(
            "compute_ik_yaw_" + std::to_string(i), std::move(gen));
        ik->setIKFrame(hand_frame_);
        ik->properties().configureInitFrom(Stage::PARENT,
                                           {"group", "ik_frame"});
        ik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
        ik->setMaxIKSolutions(8);
        ik->setMinSolutionDistance(1.0);

        alt->insert(std::move(ik));
      }
      place->insert(std::move(alt));
    }

    // 3.2 插入时允许碰撞
    {
      auto allow_collision = std::make_unique<stages::ModifyPlanningScene>(
          "allow_collision_for_insert");
      std::vector<std::string> slot_objects = {"slot_cylinder", "slot_base"};
      allow_collision->allowCollisions("hollow_cylinder", slot_objects, true);
      place->insert(std::move(allow_collision));
    }

    // 3.3 笛卡尔坐标直线插入
    {
      auto insert =
          std::make_unique<stages::MoveRelative>("linear_insert", cartesian);
      insert->properties().configureInitFrom(Stage::PARENT, {"group"});
      insert->setIKFrame("hollow_cylinder");
      // 根据latest_slot_pose_计算插入方向：沿着插槽的负Z轴方向插入
      tf2::Quaternion q;
      tf2::fromMsg(latest_slot_pose_.pose.orientation, q);
      tf2::Matrix3x3 R(q);
      tf2::Vector3 z_axis = R.getColumn(2);
      tf2::Vector3 insert_dir = -z_axis.normalized();
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "base_link";
      dir.vector.x = insert_dir.x();
      dir.vector.y = insert_dir.y();
      dir.vector.z = insert_dir.z();

      insert->setDirection(dir);
      insert->setMinMaxDistance(insert_offset_ - 0.05,
                                insert_offset_); // 插入深度
      place->insert(std::move(insert));
    }
    // 3.4 添加物体
    {
      auto attach =
          std::make_unique<stages::ModifyPlanningScene>("attach_slot_objects");

      attach->attachObject("slot_cylinder", hand_frame_);
      attach->attachObject("slot_base", hand_frame_);
      place->insert(std::move(attach));
    }
    // 3.5 第二段平移
    {
      auto slide = std::make_unique<stages::MoveRelative>("slide_along_slot_y",
                                                          cartesian);

      slide->properties().configureInitFrom(Stage::PARENT, {"group"});
      slide->setIKFrame(hand_frame_);

      // ===== 计算 slot Y 方向 =====
      tf2::Quaternion q;
      tf2::fromMsg(latest_slot_pose_.pose.orientation, q);
      tf2::Matrix3x3 R(q);

      tf2::Vector3 y_axis = R.getColumn(1);
      tf2::Vector3 move_dir = y_axis.normalized();

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = "base_link";
      dir.vector.x = move_dir.x();
      dir.vector.y = move_dir.y();
      dir.vector.z = move_dir.z();

      slide->setDirection(dir);

      slide->setMinMaxDistance(0.09, 0.10); // 100mm

      cartesian->setMinFraction(0.9);

      place->insert(std::move(slide));
    }
    // 3.6 允许碰撞
    {
      auto collision = std::make_unique<stages::ModifyPlanningScene>(
          "allow_collision_after_place");
      std::vector<std::string> gripper_links = {"LINK7", hand_frame_};
      collision->allowCollisions("hollow_cylinder", gripper_links, true);
      place->insert(std::move(collision));
    }
    // 3.7 分离物体
    {
      auto detach =
          std::make_unique<stages::ModifyPlanningScene>("detach_objects");

      detach->detachObject("hollow_cylinder", hand_frame_);
      detach->detachObject("slot_cylinder", hand_frame_);
      detach->detachObject("slot_base", hand_frame_);

      place->insert(std::move(detach));
    }
    task.add(std::move(place));
  }
  return task;
}

/* ===================== detector lock ===================== */
moveit::task_constructor::Task MTCTaskNode::createTestTask() {
  using namespace moveit::task_constructor;
  Task task;
  task.stages()->setName("test_state_machine_task");
  task.loadRobotModel(node_);
  task.setProperty("group", arm_group_);
  task.setProperty("ik_frame", hand_frame_); // 明确 IK 框架

  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node_);

  // Current State
  task.add(std::make_unique<stages::CurrentState>("current"));

  // Move to 90deg
  {
    auto move = std::make_unique<stages::MoveTo>("joint1_90deg", pipeline);
    move->setGroup(arm_group_);
    std::map<std::string, double> target;
    target["joint1"] = M_PI / 2.0;
    move->setGoal(target);
    task.add(std::move(move));
  }
  // Detach and Remove
  {
    auto detach =
        std::make_unique<stages::ModifyPlanningScene>("detach_hollow");
    detach->detachObject("hollow_cylinder", hand_frame_);
    std::vector<std::string> gripper_links = {"LINK7", "LINK_TCP"};
    detach->allowCollisions("hollow_cylinder", gripper_links, true);
    task.add(std::move(detach));
  }

  // Back to Home
  {
    auto home = std::make_unique<stages::MoveTo>("go_home", pipeline);
    home->setGroup(arm_group_);
    std::map<std::string, double> home_pose;
    for (int i = 1; i <= 7; ++i)
      home_pose["joint" + std::to_string(i)] = 0.0;
    home->setGoal(home_pose);
    task.add(std::move(home));
  }

  return task;
}

geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf2::Quaternion q = tf.getRotation();
  pose.orientation = tf2::toMsg(q);
  return pose;
}
void MTCTaskNode::setDetectorState(uint8_t state) {
  auto req = std::make_shared<SetDetectorState::Request>();
  req->state = state;
  set_state_client_->async_send_request(req);
}

// 计算目标 TCP 位姿：基于最新的插槽位姿和预定义的 TCP → hollow
// 变换，计算出TCP的预插入位姿
bool MTCTaskNode::computeTargetTCP(tf2::Transform &base_T_tcp,
                                   tf2::Transform &base_T_hollow) {
  // base -> slot
  tf2::Transform base_T_slot;
  tf2::fromMsg(latest_slot_pose_.pose, base_T_slot);
  // slot -> hollow
  tf2::Transform slot_T_hollow;
  slot_T_hollow.setOrigin({0, 0, pre_place_height_});
  tf2::Quaternion q;
  q.setRPY(M_PI, 0, 0);
  slot_T_hollow.setRotation(q);
  // base -> hollow
  base_T_hollow = base_T_slot * slot_T_hollow;
  // hollow -> tcp
  tf2::Transform hollow_T_tcp = T_tcp_hollow_.inverse();
  // base -> tcp
  base_T_tcp = base_T_hollow * hollow_T_tcp;
  return true;
}
void MTCTaskNode::publishDebugTF(const tf2::Transform &T,
                                 const std::string &name) {
  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = node_->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = name;
  t.transform = tf2::toMsg(T);

  tf_broadcaster_->sendTransform(t);
}

} // namespace mtc_place

#include "mtc_place/mtc_place_node.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto mtc_node = std::make_shared<mtc_place::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  executor.spin(); // 一直运行

  rclcpp::shutdown();
  return 0;
}
