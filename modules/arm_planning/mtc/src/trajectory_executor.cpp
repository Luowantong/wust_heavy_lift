#include "mtc/trajectory_executor.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include <functional>
#include <utility>

namespace mtc {

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node::SharedPtr node, std::string group)
    : node_(std::move(node)), group_(std::move(group)) {}

std::vector<const moveit::task_constructor::SubTrajectory *>
TrajectoryExecutor::collectLeaves(const moveit::task_constructor::SolutionBase &solution) {
  std::vector<const moveit::task_constructor::SubTrajectory *> leaves;
  std::function<void(const moveit::task_constructor::SolutionBase *)> collect =
      [&](const moveit::task_constructor::SolutionBase *s) {
        if (const auto *seq =
                dynamic_cast<const moveit::task_constructor::SolutionSequence *>(s)) {
          for (const auto *child : seq->solutions()) collect(child);
        } else if (const auto *sub =
                       dynamic_cast<const moveit::task_constructor::SubTrajectory *>(s)) {
          if (sub->trajectory()) leaves.push_back(sub);
        }
      };
  collect(&solution);
  return leaves;
}

moveit::core::MoveItErrorCode
TrajectoryExecutor::execute(const moveit::task_constructor::SolutionBase &solution) {
  const auto leaves = collectLeaves(solution);

  auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_);
  robot_trajectory::RobotTrajectory merged(mg->getRobotModel(), group_);
  for (const auto *sub : leaves)
    merged.append(*sub->trajectory(), 0.0);

  RCLCPP_INFO(node_->get_logger(), "merged %zu leaves, %zu waypoints",
              leaves.size(), merged.getWayPointCount());

  robot_trajectory::RobotTrajectory clean(mg->getRobotModel(), group_);
  if (merged.getWayPointCount() > 0) {
    moveit::core::RobotState prev = merged.getFirstWayPoint();
    clean.addSuffixWayPoint(prev, 0.0);
    for (size_t i = 1; i < merged.getWayPointCount(); ++i) {
      const auto &wp = merged.getWayPoint(i);
      if (wp.distance(prev) < 1e-6) continue;   // 同位姿 → 跳过
      clean.addSuffixWayPoint(wp, merged.getWayPointDurationFromPrevious(i));
      prev = wp;
    }
  }
  RCLCPP_INFO(node_->get_logger(), "de-duplicated to %zu waypoints",
              clean.getWayPointCount());

  if (clean.getWayPointCount() < 2)
    return moveit::core::MoveItErrorCode{ moveit_msgs::msg::MoveItErrorCodes::FAILURE };

  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.1, 0.1, 0.001);
  if (!totg.computeTimeStamps(clean, 1.0, 1.0))
    RCLCPP_ERROR(node_->get_logger(), "TOTG failed on solution");

  moveit_msgs::msg::RobotTrajectory msg;
  clean.getRobotTrajectoryMsg(msg);
  return mg->execute(msg);
}

}  // namespace mtc
