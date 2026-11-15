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
#include <moveit/robot_state/robot_state.h>
#include <functional>
#include <map>
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
  // ---------- 3. 按抓取约定把 hollow 附着到 hand_frame ----------
  // 物体相对 hand_frame 的位姿必须用 T_tcp_hollow_（与 pick/仿真 weld 一致），
  // 不能给世界系固定位姿：MoveIt 会用"当前机器人状态"把它换算到 LINK_TCP 下，
  // 结果随触发时的手臂姿态漂移（RViz 里单元相对夹爪转 90°）。
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object.id = "hollow_cylinder";
  attached.object.header.frame_id = hand_frame_;
  attached.object.primitives.resize(1);
  attached.object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  attached.object.primitives[0].dimensions = {hollow_cylinder_h, hollow_cylinder_r};
  attached.object.pose = tfToPose(T_tcp_hollow_);
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  attached.link_name = hand_frame_;
  attached.touch_links = {hand_frame_, "link1", "link2", "link3",
                          "link4", "link5", "link6", "link7"};
  psi.applyAttachedCollisionObject(attached);

  // ---------- 4. 兑换站进规划场景 ----------
  // 只放柱体（collar / peg / tip / guide），暂不放 E_coll_base：
  // base 是站体本体的大盒子，全零 home 位时单元就已经埋进去 35~40 mm，
  // 一旦加进来 CurrentState 直接判碰撞，整个任务起不来；而柱体在 home 位
  // 离单元/机械臂最近也有 86 mm，可以放心加。
  // 坐标全部用站体 E 的局部坐标，与 exchange_station.xml 的碰撞体一一对应；
  // slot_pose_stamped 就是 E 的位姿，所以 CollisionObject.pose 用它，
  // primitive_poses 保持局部坐标。
  moveit_msgs::msg::CollisionObject station;
  station.header.frame_id = planning_frame_;
  station.id = "exchange_station";
  station.pose = slot_pose_stamped.pose;
  tf2::Quaternion qy;
  qy.setRPY(0.0, M_PI / 2, 0.0);  // 圆柱轴 z → x
  auto add_cyl = [&](double radius, double length, double x) {
    shape_msgs::msg::SolidPrimitive prim;
    prim.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    prim.dimensions = {radius, length};
    tf2::Transform t;
    t.setOrigin(tf2::Vector3(x, 0.0, 0.0));
    t.setRotation(qy);
    station.primitives.push_back(prim);
    station.primitive_poses.push_back(tfToPose(t));
  };
  add_cyl(0.070, 0.102, -0.149);  // E_coll_collar
  add_cyl(0.018, 0.088, -0.054);  // E_coll_peg
  add_cyl(0.014, 0.002, 0.001);   // E_coll_tip
  add_cyl(0.018, 0.010, -0.005);  // 引导锥：10 段阶梯用一段圆柱近似
  psi.applyCollisionObject(station);
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
  // 插入必须走满全程：0.9 会把"只走 90%"当成成功（实测臂停在离座位 18 mm
  // 处、joint5 刚好卡在 -3.13 的 IK 分支上），任务却报 DONE。
  cartesian->setMinFraction(0.98);
  cartesian->setTimeParameterization(
    std::make_shared<trajectory_processing::TimeOptimalTrajectoryGeneration>());
  cartesian->setMaxVelocityScalingFactor(1.0);
  cartesian->setMaxAccelerationScalingFactor(1.0);
  // 1. 获取当前状态
  {
    auto cs = std::make_unique<stages::CurrentState>("current");
    task.add(std::move(cs));
  }
  // 2. 预放置点：先按目标算出 TCP 位姿，再用固定种子（全零）求 IK，
  //    最后用关节空间 MoveTo 精确走到这个构型。
  //    不能直接 Connect 到姿态：OMPL 的位姿目标会在多解 IK 里随机落点，实测
  //    会选到 joint5≈-3.13 的翻转分支；该分支做笛卡尔插入时 joint5 在柱内
  //    120 mm 处顶到 -3.14 限位，CartesianPath 只能完成 90%（Achieved: 0.900000），
  //    臂停在离座位 18 mm 处而任务仍能报 DONE。用固定种子后走的是
  //    joint5≈-0.5 → +0.15 的分支，离线 IK 验证全程可行。
  tf2::Transform base_T_tcp, base_T_hollow;
  if (!computeTargetTCP(base_T_tcp, base_T_hollow))
    throw std::runtime_error("target pose compute failed");
  publishDebugTF(base_T_hollow, "T_target_hollow");

  {
    const auto &robot_model = task.getRobotModel();
    const auto *jmg = robot_model->getJointModelGroup(arm_group_);
    geometry_msgs::msg::PoseStamped pre_pose;
    pre_pose.header.frame_id = planning_frame_;
    tf2::toMsg(base_T_tcp, pre_pose.pose);

    // ---- 种子阶梯 ----
    // 目标锁定之后是确定性的，同一个失败会重复，所以必须换种子，
    // 只靠 plan_attempts 重试是没用的。
    // 注意：不能随机撒种子。q5≈-3.1 那条分支插到一半顶 -3.14 限位，
    // 全零种子是特意挑出来的"能走完 180mm"的分支，扰动必须小到不会换分支。
    auto zero_seed = [&]() {
      moveit::core::RobotState s(robot_model);
      s.setToDefaultValues();
      return s;
    };

    std::vector<moveit::core::RobotState> seeds;
    seeds.push_back(zero_seed());  // 1) 全零：验证过的好分支

    if (!last_pre_place_q_.empty()) {  // 2) 上一次成功的解：跨次运行连续
      moveit::core::RobotState s = zero_seed();
      const auto &gvars = jmg->getVariableNames();
      for (size_t i = 0; i < gvars.size() && i < last_pre_place_q_.size(); ++i)
        s.setVariablePosition(gvars[i], last_pre_place_q_[i]);
      seeds.push_back(s);
    }

    // 3) 全零 + joint5 小扰动：只为了躲开局部优化器的卡点
    for (double dq : {+0.03, -0.03}) {
      moveit::core::RobotState s = zero_seed();
      std::vector<double> qj;
      s.copyJointGroupPositions(jmg, qj);
      qj[4] += dq;
      s.setJointGroupPositions(jmg, qj);
      seeds.push_back(s);
    }

    auto try_seeds = [&](const geometry_msgs::msg::Pose &target,
                         std::vector<double> &q_out) {
      for (size_t i = 0; i < seeds.size(); ++i) {
        if (seeds[i].setFromIK(jmg, target, hand_frame_, 0.5)) {
          seeds[i].copyJointGroupPositions(jmg, q_out);
          last_pre_place_q_ = q_out;
          RCLCPP_INFO(node_->get_logger(),
                      "pre-place IK solved with seed %zu/%zu", i + 1, seeds.size());
          return true;
        }
      }
      return false;
    };

    std::vector<double> q;
    if (!try_seeds(pre_pose.pose, q)) {
      // 4) 兜底：感知目标不可信时退回 YAML 真值再试一次。
      //    这一步会完全绕过感知，属于"能用但不可信"，必须大声报出来。
      RCLCPP_WARN(node_->get_logger(),
                  "pre-place IK failed on all seeds, falling back to YAML slot");
      target_slot_.pose = yaml_slot_.pose;
      if (!computeTargetTCP(base_T_tcp, base_T_hollow) ||
          !try_seeds(tfToPose(base_T_tcp), q))
        throw std::runtime_error("pre-place IK failed (perceived and YAML)");
      publishDebugTF(base_T_hollow, "T_target_hollow");
      RCLCPP_WARN(node_->get_logger(),
                  "DEGRADED: this run uses the YAML slot pose, perception bypassed");
    }

    std::map<std::string, double> goal;
    const auto &vars = jmg->getVariableNames();
    for (size_t i = 0; i < vars.size(); ++i)
      goal[vars[i]] = q[i];

    auto move = std::make_unique<stages::MoveTo>("move_to_pre_place", pipeline);
    move->properties().configureInitFrom(Stage::PARENT, {"group"});
    move->setGoal(goal);
    task.add(std::move(move));
  }

  // 3. 放置串行容器
  {
    auto place = std::make_unique<SerialContainer>("place_container");
    task.properties().exposeTo(place->properties(), {"group", "ik_frame"});

    // 3.2 插入时允许碰撞（放在 IK 之后：此时容器是正向计算的，编辑不会被反转）
    // 站体已经进规划场景，接近段（move_to_place_area）会绕着它走；
    // 这里放行"单元 ↔ 站体"和"腕部 ↔ 站体"，让 linear_insert 能真的套进去。
    //
    // 为什么腕部也要放行：单元和夹爪在碰撞模型里都是实心体。单元插到底时
    // 柱子（peg，r=18mm）整根在单元内部，夹爪的 collision mesh 又包着单元，
    // 于是 link7 的网格会罩住 peg 前端。实测插到底位姿 check_state_validity
    // 报 link7<->exchange_station 深 11.6mm，把 peg 那段 primitive 去掉这个
    // 接触就消失（collar 去掉仍在）——纯属实心碰撞体的近似，不是真撞。
    // 后果很具体：最后 0.5mm 的 waypoint 被判碰撞，CartesianPath 只完成
    // 0.90（162/180mm），臂停在离座位 18mm 处，任务却报 DONE。
    {
      auto allow_collision = std::make_unique<stages::ModifyPlanningScene>(
          "allow_collision_for_insert");
      allow_collision->allowCollisions("hollow_cylinder", "exchange_station", true);
      allow_collision->allowCollisions("link7", "exchange_station", true);
      allow_collision->allowCollisions(hand_frame_, "exchange_station", true);
      place->insert(std::move(allow_collision));
    }

    // 3.3 笛卡尔坐标直线插入
    {
      auto insert =
          std::make_unique<stages::MoveRelative>("linear_insert", cartesian);
      insert->properties().configureInitFrom(Stage::PARENT, {"group"});
      insert->setIKFrame("hollow_cylinder");
      tf2::Quaternion q;
      tf2::fromMsg(target_slot_.pose.orientation, q);
      tf2::Matrix3x3 R(q);
      //tf2::Vector3 z_axis = R.getColumn(2);
      tf2::Vector3 insert_dir = -R.getColumn(0).normalized();
      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = planning_frame_;
      dir.vector.x = insert_dir.x();
      dir.vector.y = insert_dir.y();
      dir.vector.z = insert_dir.z();

      insert->setDirection(dir);
      insert->setMinMaxDistance(pre_place_height_, pre_place_height_);
      place->insert(std::move(insert));
    }
    // 3.4 插入到位后沿站体 +z 向上顶（此时单元还在夹爪上，必须在 detach 之前）
    //     用独立求解器，避免把 min_fraction=0.5 带进 linear_insert
    if (push_up_height_ > 1e-6) {
      auto cartesian_push = std::make_shared<solvers::CartesianPath>();
      cartesian_push->setStepSize(0.002);
      cartesian_push->setMinFraction(0.9);

      auto push =
          std::make_unique<stages::MoveRelative>("push_up", cartesian_push);
      push->properties().configureInitFrom(Stage::PARENT, {"group"});
      push->setIKFrame(hand_frame_);

      tf2::Quaternion q;
      tf2::fromMsg(target_slot_.pose.orientation, q);
      tf2::Vector3 z_axis = tf2::Matrix3x3(q).getColumn(2).normalized();

      geometry_msgs::msg::Vector3Stamped dir;
      dir.header.frame_id = planning_frame_;
      dir.vector.x = z_axis.x();
      dir.vector.y = z_axis.y();
      dir.vector.z = z_axis.z();

      push->setDirection(dir);
      push->setMinMaxDistance(push_up_height_ * 0.95, push_up_height_);
      place->insert(std::move(push));
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
  // slot_collar_x  = 兑换柱法兰面（站体局部 x，实测 -0.098）
  // insert_offset_ = 单元半长：法兰面到座位中心的距离（0.075）
  // pre_place_height_ = 预放置点到座位的插入行程
  // 座位中心 = slot_collar_x + insert_offset_ = -0.023，此时单元开口端法兰
  // 正好顶在 collar 面上；预放置点再往外退 pre_place_height_。
  tf2::Transform slot_T_hollow;
  slot_T_hollow.setOrigin(
      {slot_collar_x + insert_offset_ + pre_place_height_, 0.0, 0.0});
  tf2::Quaternion q;
  q.setRPY(0.0, -M_PI / 2, 0.0);   // hollow z 轴 → 柱子 -x
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
