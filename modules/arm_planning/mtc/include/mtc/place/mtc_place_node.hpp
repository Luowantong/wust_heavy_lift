#pragma once

#include "mtc/common/task_node_base.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/task.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <thread>
#include <vector>

namespace mtc {
  geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf);
class MTCTaskNode : public TaskNodeBase {
public:
    explicit MTCTaskNode(const rclcpp::NodeOptions &options)
    : TaskNodeBase(options, "mtc_place_node", "target_slot") {}
protected:
  moveit::task_constructor::Task createTask() override;
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped &target) override;
  // 目标定稿：种子阶梯求 IK，全败则退回 YAML（允许改写 target）。
  // 由 TaskNodeBase 在 setupPlanningScene 之前调用，解出的构型存 pre_place_q_。
  bool resolveTarget(geometry_msgs::msg::PoseStamped &target) override;
private:
  bool computeTargetTCP(const geometry_msgs::msg::PoseStamped &slot,
                        tf2::Transform &, tf2::Transform &);
  // 上一次成功的预放置解（关节角度，按 arm_group_ 变量顺序）。
  // 作为 IK 种子阶梯的第 2 级：跨次运行保持连续性，优先复用同一条分支。
  std::vector<double> last_pre_place_q_;
  // 本次运行解出的预放置构型（resolveTarget 写、createTask 读）
  std::vector<double> pre_place_q_;
  // 惰性加载：resolveTarget 用 setFromIK，需要组上的 IK 求解器
  moveit::core::RobotModelConstPtr robot_model_;
};
}
