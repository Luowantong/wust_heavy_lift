#include "mtc/mtc_place_node.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <functional>
#include <vector>

namespace mtc {
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions &options) {
  /* ---------- node ---------- */
  node_ = std::make_shared<rclcpp::Node>("mtc_place_node", options);

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
      std::bind(&MTCTaskNode::onPlanAndExecute, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cb_group_task_);

  // 运行时临时覆盖 YAML 里的目标（默认关闭，需 allow_slot_override:=true）
  if (allow_slot_override_) {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = cb_group_target_;
    slot_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/exchange_slot/slot_pose", 10,
        std::bind(&MTCTaskNode::onTargetOverride, this, std::placeholders::_1),
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
  slot_pose.header.frame_id = planning_frame_; // 强制 frame_id 为 base_link

  // ---------- 1. 底座碰撞体 ----------
  moveit_msgs::msg::CollisionObject base;
  base.id = "slot_base";
  base.header.frame_id = planning_frame_;
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
  cylinder.header.frame_id = planning_frame_;
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
  double offset_z = 0.075; // 往回移动，使侧面与 LINK7 平面相切

  tf_hollow.setOrigin(tf2::Vector3(offset_x, 0, offset_z));

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
  attached.touch_links = {hand_frame_, "link7"}; // 与手爪和LINK7允许接触
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

void MTCTaskNode::onTargetOverride(
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
  task_ = createTask();
  task_.enableIntrospection();

  try {
    task_.init();
    RCLCPP_DEBUG(node_->get_logger(), "MTC Task init successfully");
  } catch (const moveit::task_constructor::InitStageException &e) {
    RCLCPP_ERROR(node_->get_logger(), "MTC Task init failed: %s", e.what());
    std::cout << task_ << std::endl; // debug
    setState(State::FAILED);
    task_running_ = false;
    return;
  }

  if (!task_.plan(1)) {
    setState(State::FAILED);
    task_running_ = false;
    RCLCPP_ERROR(node_->get_logger(), "Task planning failed!");
    std::cout << task_ << std::endl; // debug
    return;
  }

  auto solution = task_.solutions().front();

  // Debug：可视化
  task_.introspection().publishSolution(*solution);
  // std::cout << task_ << std::endl; // debug

  setState(State::EXECUTING);

  std::vector<const moveit::task_constructor::SubTrajectory*> leaves;
  std::function<void(const moveit::task_constructor::SolutionBase*)> collect =
      [&](const moveit::task_constructor::SolutionBase* s) {
        if (const auto* seq = dynamic_cast<const moveit::task_constructor::SolutionSequence*>(s)) {
          for (const auto* child : seq->solutions()) collect(child);
        } else if (const auto* sub = dynamic_cast<const moveit::task_constructor::SubTrajectory*>(s)) {
          if (sub->trajectory()) leaves.push_back(sub);
        }
      };
  collect(solution.get());

  auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, arm_group_);
  robot_trajectory::RobotTrajectory merged(mg->getRobotModel(), arm_group_);
  for (const auto* sub : leaves)
    merged.append(*sub->trajectory(), 0.0);

  RCLCPP_INFO(node_->get_logger(), "merged %zu leaves, %zu waypoints",
              leaves.size(), merged.getWayPointCount());

  robot_trajectory::RobotTrajectory clean(mg->getRobotModel(), arm_group_);
  if (merged.getWayPointCount() > 0) {
    moveit::core::RobotState prev = merged.getFirstWayPoint();
    clean.addSuffixWayPoint(prev, 0.0);
    for (size_t i = 1; i < merged.getWayPointCount(); ++i) {
      const auto& wp = merged.getWayPoint(i);
      if (wp.distance(prev) < 1e-6) continue;   // 同位姿 → 跳过
      clean.addSuffixWayPoint(wp, merged.getWayPointDurationFromPrevious(i));
      prev = wp;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "de-duplicated to %zu waypoints", clean.getWayPointCount());

  moveit::core::MoveItErrorCode result{ moveit_msgs::msg::MoveItErrorCodes::FAILURE };
  if (clean.getWayPointCount() >= 2) {
    trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.1, 0.1, 0.001);
    if (!totg.computeTimeStamps(clean, 1.0, 1.0))
      RCLCPP_ERROR(node_->get_logger(), "TOTG failed on solution");

    moveit_msgs::msg::RobotTrajectory msg;
    clean.getRobotTrajectoryMsg(msg);
    result = mg->execute(msg);
  }

  if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
    setState(State::DONE);
  else
    setState(State::FAILED);

  task_running_ = false;

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
  pipeline->setPlannerId("RRTConnectkConfigDefault");
  pipeline->setProperty("max_velocity_scaling_factor", 0.5);
  pipeline->setProperty("max_acceleration_scaling_factor", 0.5);

  
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  // 笛卡尔路径每 1mm 一个 waypoint
  cartesian->setStepSize(0.001);
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
    connect->setTimeout(2.0);
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
      int n_yaw = 3;               // 备选yaw 个数
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
        pose.header.frame_id = planning_frame_;
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
        ik->setMaxIKSolutions(3);
        ik->setMinSolutionDistance(0.1);

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
      // 根据target_slot_计算插入方向：沿着插槽的负Z轴方向插入
      tf2::Quaternion q;
      tf2::fromMsg(target_slot_.pose.orientation, q);
      tf2::Matrix3x3 R(q);
      tf2::Vector3 z_axis = R.getColumn(2);
      tf2::Vector3 insert_dir = -z_axis.normalized();
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = planning_frame_;
      dir.vector.x = insert_dir.x();
      dir.vector.y = insert_dir.y();
      dir.vector.z = insert_dir.z();

      insert->setDirection(dir);
      insert->setMinMaxDistance(insert_offset_ - 0.05,
                                insert_offset_); // 插入深度
      //place->insert(std::move(insert));
    }
    // 3.4 添加物体
    {
      auto attach =
          std::make_unique<stages::ModifyPlanningScene>("attach_slot_objects");

      //attach->attachObject("slot_cylinder", hand_frame_);
      //attach->attachObject("slot_base", hand_frame_);
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
      tf2::fromMsg(target_slot_.pose.orientation, q);
      tf2::Matrix3x3 R(q);

      tf2::Vector3 y_axis = R.getColumn(1);
      tf2::Vector3 move_dir = y_axis.normalized();

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = planning_frame_;
      dir.vector.x = move_dir.x();
      dir.vector.y = move_dir.y();
      dir.vector.z = move_dir.z();

      slide->setDirection(dir);

      slide->setMinMaxDistance(0.03, 0.04); // 100mm

      cartesian->setMinFraction(0.5);

      //place->insert(std::move(slide));
    }
    // 3.6 允许碰撞
    {
      auto collision = std::make_unique<stages::ModifyPlanningScene>(
          "allow_collision_after_place");
      std::vector<std::string> gripper_links = {"link7", hand_frame_};
      collision->allowCollisions("hollow_cylinder", gripper_links, true);
      place->insert(std::move(collision));
    }
    // 3.7 分离物体
    {
      auto detach =
          std::make_unique<stages::ModifyPlanningScene>("detach_objects");

      detach->detachObject("hollow_cylinder", hand_frame_);
      //detach->detachObject("slot_cylinder", hand_frame_);
      //detach->detachObject("slot_base", hand_frame_);

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
    std::vector<std::string> gripper_links = {"link7", "LINK_TCP"};
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
/* ===================== trigger & state ===================== */

void MTCTaskNode::onPlanAndExecute(
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

void MTCTaskNode::loadTargetParams() {
  const double x     = node_->get_parameter_or("target_slot.x", 0.0);
  const double y     = node_->get_parameter_or("target_slot.y", 0.0);
  const double z     = node_->get_parameter_or("target_slot.z", 0.0);
  const double roll  = node_->get_parameter_or("target_slot.roll", 0.0);
  const double pitch = node_->get_parameter_or("target_slot.pitch", 0.0);
  const double yaw   = node_->get_parameter_or("target_slot.yaw", 0.0);

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

void MTCTaskNode::setState(State s) {
  state_.store(s);
  static const char *names[] = {"IDLE", "PLANNING", "EXECUTING", "DONE", "FAILED"};
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(s);
  state_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "State -> %s", names[static_cast<int>(s)]);
}

// 计算目标 TCP 位姿：基于最新的插槽位姿和预定义的 TCP → hollow
// 变换，计算出TCP的预插入位姿
bool MTCTaskNode::computeTargetTCP(tf2::Transform &base_T_tcp,
                                   tf2::Transform &base_T_hollow) {
  // base -> slot
  tf2::Transform base_T_slot;
  tf2::fromMsg(target_slot_.pose, base_T_slot);
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
  t.header.frame_id = planning_frame_;
  t.child_frame_id = name;
  t.transform = tf2::toMsg(T);

  tf_broadcaster_->sendTransform(t);
}

} // namespace mtc_place

#include "mtc/mtc_place_node.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<mtc::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  executor.spin(); // 一直运行

  rclcpp::shutdown();
  return 0;
}
