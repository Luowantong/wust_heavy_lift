#include "mtc/pick/mtc_pick_node.hpp"
#include "mtc/common/geometry.hpp" 
#include "mtc/trajectory_executor.hpp"
#include "mtc/common/tcp_hollow.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit_msgs/msg/detail/robot_trajectory__struct.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/task_constructor/storage.h>
#include <functional>
#include <vector>

namespace mtc {
MTCPickNode::MTCPickNode(const rclcpp::NodeOptions &options)
    : TaskNodeBase(options, "mtc_pick_node", "target_object") {
  approach_height_ = node_->get_parameter_or("approach_height", 0.10);
  lift_height_     = node_->get_parameter_or("lift_height", 0.10);
  yaw_candidates_  = node_->get_parameter_or("yaw_candidates", 3);
  k_retreat_ = node_->get_parameter_or("k_retreat", 1.0);
}
/* ===================== planning scene ===================== */
void MTCPickNode::setupPlanningScene(
    const geometry_msgs::msg::PoseStamped &target) {
  moveit::planning_interface::PlanningSceneInterface psi;

  // 清掉上一次运行的残留
  moveit_msgs::msg::AttachedCollisionObject detach;
  detach.object.id = "hollow_cylinder";
  detach.object.operation = detach.object.REMOVE;
  psi.applyAttachedCollisionObject(detach);
  psi.removeCollisionObjects({"hollow_cylinder", "slot_base", "slot_cylinder"});

  // 目标物体：世界物体，不附着
  moveit_msgs::msg::CollisionObject hollow;
  hollow.id = "hollow_cylinder";
  hollow.header.frame_id = planning_frame_;
  hollow.primitives.resize(1);
  hollow.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  hollow.primitives[0].dimensions = {hollow_cylinder_h, hollow_cylinder_r};
  hollow.pose = target.pose;
  hollow.operation = hollow.ADD;
  psi.applyCollisionObject(hollow);

  // 夹爪属性（place/pick 共用）
  T_tcp_hollow_ = makeTcpHollow();

  // 调试 TF
  tf2::Transform base_T_object;
  tf2::fromMsg(target.pose, base_T_object);
  publishDebugTF(base_T_object, "T_target_object");
}
bool MTCPickNode::computeGraspTCP(tf2::Transform &base_T_tcp_pre) {
  // base -> object
  tf2::Transform base_T_object;
  tf2::fromMsg(target_slot_.pose, base_T_object);

  // 抓取瞬间的 TCP：hollow 正好落在物体上
  tf2::Transform hollow_T_tcp = T_tcp_hollow_.inverse();
  tf2::Transform base_T_tcp_grasp = base_T_object * hollow_T_tcp;

  // 预抓取点：沿物体 +z 抬高
  tf2::Vector3 z_axis =
      tf2::Matrix3x3(base_T_object.getRotation()).getColumn(2).normalized();

  base_T_tcp_pre = base_T_tcp_grasp;
  base_T_tcp_pre.setOrigin(base_T_tcp_grasp.getOrigin() +
                           z_axis * approach_height_);
  return true;
}

/* ===================== MTC ===================== */
moveit::task_constructor::Task MTCPickNode::createTask() {
  using namespace moveit::task_constructor;
  Task task;
  task.stages()->setName("pick_hollow_cylinder");

  task.loadRobotModel(node_);
  task.setProperty("group", arm_group_);
  task.setProperty("ik_frame", hand_frame_);

  auto pipeline = std::make_shared<solvers::PipelinePlanner>(node_,"ompl");
  pipeline->setTimeParameterization(
    std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());
  pipeline->setPlannerId("RRTConnectkConfigDefault");

  
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  // 笛卡尔路径每 1mm 一个 waypoint
  cartesian->setStepSize(0.005);
  cartesian->setMinFraction(0.9);
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
        "move_to_pick_area",
        stages::Connect::GroupPlannerVector{{arm_group_, pipeline}});
    connect->setTimeout(2.0);
    connect->properties().configureInitFrom(Stage::PARENT);
    task.add(std::move(connect));
  }
  

  // 3. 放置串行容器
  {
    auto pick = std::make_unique<SerialContainer>("pick_container");
    task.properties().exposeTo(pick->properties(), {"group", "ik_frame"});
      // 3.1 生成预抓取姿态
  {
    auto alt = std::make_unique<Alternatives>("ik_yaw_alternative");
    pick->properties().exposeTo(alt->properties(), {"group", "ik_frame"});

    tf2::Transform base_T_tcp_pre;
    if (!computeGraspTCP(base_T_tcp_pre))
      throw std::runtime_error("grasp pose compute failed");

    tf2::Transform base_T_object;
    tf2::fromMsg(target_slot_.pose, base_T_object);
    tf2::Vector3 z_axis =
    tf2::Matrix3x3(base_T_object.getRotation()).getColumn(2).normalized();
    tf2::Vector3 dir_v = z_axis;


    publishDebugTF(base_T_object, "T_target_object");
    publishDebugTF(base_T_tcp_pre, "T_target_tcp");

    // yaw sampling 参数
    const int n_yaw = yaw_candidates_;
    const double yaw_range = M_PI / 2;
    tf2::Transform hollow_T_tcp = T_tcp_hollow_.inverse();
    for (int i = 0; i < n_yaw; ++i) {
      const double yaw =
          (n_yaw > 1) ? (-yaw_range + i * (2 * yaw_range) / (n_yaw - 1)) : 0.0;

      // 绕物体自身 z 轴转
      tf2::Transform object_rot;
      object_rot.setIdentity();
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      object_rot.setRotation(q);

      tf2::Transform base_T_object_yaw = base_T_object * object_rot;
      tf2::Transform base_T_tcp_grasp = base_T_object_yaw * hollow_T_tcp;

      tf2::Transform base_T_tcp_pre_yaw = base_T_tcp_grasp;
      base_T_tcp_pre_yaw.setOrigin(base_T_tcp_grasp.getOrigin() +
                                   z_axis * approach_height_);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = planning_frame_;
      tf2::toMsg(base_T_tcp_pre_yaw, pose.pose);

      publishDebugTF(base_T_object_yaw,
                     "T_target_object_yaw_" + std::to_string(i));
      publishDebugTF(base_T_tcp_pre_yaw,
                     "T_target_tcp_yaw_" + std::to_string(i));

      // GeneratePose
      auto gen = std::make_unique<stages::GeneratePose>(
          "gen_pose_yaw_" + std::to_string(i));
      gen->setPose(pose);
      gen->setMonitoredStage(current_state_ptr);

      // ComputeIK
      auto ik = std::make_unique<stages::ComputeIK>(
          "compute_ik_yaw_" + std::to_string(i), std::move(gen));
      ik->setIKFrame(hand_frame_);
      ik->properties().configureInitFrom(Stage::PARENT, {"group", "ik_frame"});
      ik->properties().configureInitFrom(Stage::INTERFACE, {"target_pose"});
      ik->setMaxIKSolutions(3);
      ik->setMinSolutionDistance(0.1);
      ik->setIgnoreCollisions(true);    

      alt->insert(std::move(ik));
    }
    pick->insert(std::move(alt));
  }

  {
    auto allow = std::make_unique<stages::ModifyPlanningScene>(
        "allow_collision_for_grasp");
    std::vector<std::string> gripper_links = {
        "link1", "link2", "link3", "link4", "link5", "link6", "link7", hand_frame_};
    allow->allowCollisions("hollow_cylinder", gripper_links, true);
    pick->insert(std::move(allow));        
  }


  // 3.3 笛卡尔下降
  {
    auto approach = std::make_unique<stages::MoveRelative>("approach", cartesian);
    approach->properties().configureInitFrom(Stage::PARENT, {"group"});
    approach->setIKFrame(hand_frame_);

    tf2::Transform base_T_object;
    tf2::fromMsg(target_slot_.pose, base_T_object);
    tf2::Vector3 z_axis =
        tf2::Matrix3x3(base_T_object.getRotation()).getColumn(2).normalized();
    tf2::Vector3 dir_v = -z_axis;   // 沿物体轴向下

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = planning_frame_;
    dir.vector.x = dir_v.x();
    dir.vector.y = dir_v.y();
    dir.vector.z = dir_v.z();

    approach->setDirection(dir);
    approach->setMinMaxDistance(approach_height_ * 0.8, approach_height_);
    pick->insert(std::move(approach));
  }

  // 3.4 附着（触发 GraspPlugin 的 weld）
  {
    auto attach = std::make_unique<stages::ModifyPlanningScene>("attach_hollow");
    attach->attachObject("hollow_cylinder", hand_frame_);
    pick->insert(std::move(attach));
  }

  // 3.5 抬起
  {
    auto lift = std::make_unique<stages::MoveRelative>("lift", cartesian);
    lift->properties().configureInitFrom(Stage::PARENT, {"group"});
    lift->setIKFrame(hand_frame_);

      tf2::Transform base_T_object;
    tf2::fromMsg(target_slot_.pose, base_T_object);
    tf2::Vector3 z_axis =
        tf2::Matrix3x3(base_T_object.getRotation()).getColumn(2).normalized();

    // 抬起方向 = 物体 +z 向上 + 朝底座水平回撤。
    const double kRetreat = k_retreat_;  
    tf2::Vector3 toward_base(-base_T_object.getOrigin().x(),
                             -base_T_object.getOrigin().y(), 0.0);
    if (toward_base.length() > 1e-6)
      toward_base.normalize();
    else
      toward_base.setZero();
    tf2::Vector3 dir_v = (z_axis + toward_base * kRetreat).normalized();

    geometry_msgs::msg::Vector3Stamped dir;
    dir.header.frame_id = planning_frame_;
    dir.vector.x = dir_v.x();
    dir.vector.y = dir_v.y();
    dir.vector.z = dir_v.z();

    lift->setDirection(dir);
    lift->setMinMaxDistance(lift_height_ * 0.95, lift_height_);

    pick->insert(std::move(lift));
  }
  task.add(std::move(pick));
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
} // namespace mtc

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<mtc::MTCPickNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  executor.spin();  // 一直运行

  rclcpp::shutdown();
  return 0;
}
