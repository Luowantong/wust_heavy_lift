#include "mtc/place/mtc_place_node.hpp"
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

  tf2::Transform tf_hollow = makeTcpHollow();

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
  attached.touch_links = {hand_frame_, "link1", "link2", "link3",
                        "link4", "link5", "link6", "link7"};
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

  
  auto cartesian = std::make_shared<solvers::CartesianPath>();
  // 笛卡尔路径每 5mm 一个 waypoint（1mm 时 waypoint 太多，每个点都要独立解一次 IK）
  cartesian->setStepSize(0.020);
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
        // 单元是空心的、预放置点上已经套住兑换柱，IK 阶段不判碰撞；
        // 真正的碰撞由后面的 linear_insert 通过 allow_collision_for_insert 放行。
        //ik->setIgnoreCollisions(true);

        alt->insert(std::move(ik));
      }
      place->insert(std::move(alt));
    }

    // 3.2 插入时允许碰撞（放在 IK 之后：此时容器是正向计算的，编辑不会被反转）
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
      // 预放置点(物体中心 z=0.33) → 就位(0.28)，行程 5 cm
      insert->setMinMaxDistance(0.09, 0.105);
      place->insert(std::move(insert));
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
    // 注意：当前工装是"单元套在柱子上"，横向平移会把单元从柱子上蹭下来，
    // 所以暂不插入这个 stage。启用时用它自己的求解器，别改共享的 cartesian，
    // 否则会把 min_fraction=0.5 带进 linear_insert。
    {
      auto cartesian_slide = std::make_shared<solvers::CartesianPath>();
      cartesian_slide->setStepSize(0.005);
      cartesian_slide->setMinFraction(0.5);

      auto slide = std::make_unique<stages::MoveRelative>("slide_along_slot_y",
                                                          cartesian_slide);

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

geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf2::Quaternion q = tf.getRotation();
  pose.orientation = tf2::toMsg(q);
  return pose;
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
  // 物体的 z 朝上，和 pick 的抓取约定一致；若绕 x 转 180°，
  // hollow_T_tcp 会把 TCP 放到物体下方 7.5cm，直接撞进兑换柱。
  q.setRPY(0, 0, 0);
  slot_T_hollow.setRotation(q);
  // base -> hollow
  base_T_hollow = base_T_slot * slot_T_hollow;
  // hollow -> tcp
  tf2::Transform hollow_T_tcp = T_tcp_hollow_.inverse();
  // base -> tcp
  base_T_tcp = base_T_hollow * hollow_T_tcp;
  return true;
}
} // namespace mtc

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_node = std::make_shared<mtc::MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mtc_node->getNodeBaseInterface());

  executor.spin();  // 一直运行

  rclcpp::shutdown();
  return 0;
}
