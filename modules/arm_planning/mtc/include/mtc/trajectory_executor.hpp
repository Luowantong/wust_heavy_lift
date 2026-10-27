#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit/utils/moveit_error_code.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit/utils/moveit_error_code.h>

#include <string>
#include <vector>

namespace mtc {

/// 把 MTC 的解（可能含多个 SubTrajectory）合并、去重、TOTG 重定时后执行。
class TrajectoryExecutor {
public:
  TrajectoryExecutor(rclcpp::Node::SharedPtr node, std::string group);

  moveit::core::MoveItErrorCode
  execute(const moveit::task_constructor::SolutionBase &solution);

private:
  static std::vector<const moveit::task_constructor::SubTrajectory *>
  collectLeaves(const moveit::task_constructor::SolutionBase &solution);

  rclcpp::Node::SharedPtr node_;
  std::string group_;
};

}  // namespace mtc
