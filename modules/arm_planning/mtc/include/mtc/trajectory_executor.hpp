#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <moveit/move_group_interface/move_group_interface.h>   
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <cstddef>
#include <string>
#include <vector>

namespace mtc {

/// 把 MTC 的解（可能含多个 SubTrajectory）合并、去重、TOTG 重定时后执行。
class TrajectoryExecutor {
public:
    TrajectoryExecutor(rclcpp::Node::SharedPtr node, std::string group,
                     std::string weld_object = "hollow_cylinder",
                     std::string weld_link   = "LINK_TCP");

  moveit::core::MoveItErrorCode
  execute(const moveit::task_constructor::SolutionBase &solution);

private:
  static std::vector<const moveit::task_constructor::SubTrajectory *>
  collectLeaves(const moveit::task_constructor::SolutionBase &solution);
  void publishWeld(bool attach);
  moveit::core::MoveItErrorCode
  executeSegment(moveit::planning_interface::MoveGroupInterface &mg,
                 const std::vector<const moveit::task_constructor::SubTrajectory *> &leaves,
                 std::size_t begin, std::size_t end);

  rclcpp::Node::SharedPtr node_;
  std::string group_;
  rclcpp::Publisher<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr weld_pub_;
  std::string weld_object_;
  std::string weld_link_;
};

}  // namespace mtc
