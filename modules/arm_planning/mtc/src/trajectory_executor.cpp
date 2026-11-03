#include "mtc/trajectory_executor.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/task_constructor/stage.h>

#include <functional>
#include <utility>
#include <chrono>
#include <thread>

namespace mtc {

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node::SharedPtr node, std::string group,
                                       std::string weld_object, std::string weld_link)
    : node_(std::move(node)),
      group_(std::move(group)),
      weld_object_(std::move(weld_object)),
      weld_link_(std::move(weld_link)) {
  weld_pub_ = node_->create_publisher<moveit_msgs::msg::AttachedCollisionObject>(
      "/attached_collision_object", rclcpp::QoS(10));
}

void TrajectoryExecutor::publishWeld(bool attach) {
  moveit_msgs::msg::AttachedCollisionObject msg;
  msg.object.id = weld_object_;
  msg.object.operation = attach ? moveit_msgs::msg::CollisionObject::ADD
                                : moveit_msgs::msg::CollisionObject::REMOVE;
  if (attach) msg.link_name = weld_link_;
  weld_pub_->publish(msg);
  RCLCPP_INFO(node_->get_logger(), "weld trigger: %s", attach ? "grab" : "release");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 等插件 pre_step 生效
}

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
          leaves.push_back(sub); 
      }
    };
  collect(&solution);
  return leaves;
}

moveit::core::MoveItErrorCode
TrajectoryExecutor::executeSegment(
    moveit::planning_interface::MoveGroupInterface &mg,
    const std::vector<const moveit::task_constructor::SubTrajectory *> &leaves,
    std::size_t begin, std::size_t end) {
  if (end <= begin)
    return moveit::core::MoveItErrorCode{ moveit_msgs::msg::MoveItErrorCodes::SUCCESS };

  robot_trajectory::RobotTrajectory merged(mg.getRobotModel(), group_);
  for (std::size_t i = begin; i < end; ++i)
    if (const auto &t = leaves[i]->trajectory())
      merged.append(*t, 0.0);


  robot_trajectory::RobotTrajectory clean(mg.getRobotModel(), group_);
  if (merged.getWayPointCount() > 0) {
    moveit::core::RobotState prev = merged.getFirstWayPoint();
    clean.addSuffixWayPoint(prev, 0.0);
    for (std::size_t i = 1; i < merged.getWayPointCount(); ++i) {
      const auto &wp = merged.getWayPoint(i);
      if (wp.distance(prev) < 1e-6) continue;
      clean.addSuffixWayPoint(wp, merged.getWayPointDurationFromPrevious(i));
      prev = wp;
    }
  }

  if (clean.getWayPointCount() < 2)
    return moveit::core::MoveItErrorCode{ moveit_msgs::msg::MoveItErrorCodes::SUCCESS };

  if (auto current = mg.getCurrentState()) {
    robot_trajectory::RobotTrajectory anchored(mg.getRobotModel(), group_);
    anchored.addSuffixWayPoint(*current, 0.0);
    for (std::size_t i = 1; i < clean.getWayPointCount(); ++i)
      anchored.addSuffixWayPoint(clean.getWayPoint(i), 0.0);
    clean.swap(anchored);   // 时间戳随后由 TOTG 重算
  }
  
  trajectory_processing::TimeOptimalTrajectoryGeneration totg(0.1, 0.1, 0.001);
  if (!totg.computeTimeStamps(clean, 1.0, 1.0))
    RCLCPP_ERROR(node_->get_logger(), "TOTG failed on segment [%zu, %zu)", begin, end);

  moveit_msgs::msg::RobotTrajectory msg;
  clean.getRobotTrajectoryMsg(msg);
  return mg.execute(msg);
}

moveit::core::MoveItErrorCode
TrajectoryExecutor::execute(const moveit::task_constructor::SolutionBase &solution) {
  const auto leaves = collectLeaves(solution);
  auto mg = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, group_);

  RCLCPP_INFO(node_->get_logger(), "solution has %zu leaves", leaves.size());

  // place 任务一开始物体就附着在夹爪上（MoveIt 层面），但 MuJoCo 插件不知道。
  // 若解的第一个 waypoint 已经带着附着体，先补一条抓取事件，
  // 否则插件的 pre_step 会一直激活 rest，把物体钉在原地。
  if (!leaves.empty() && leaves.front()->trajectory() &&
      leaves.front()->trajectory()->getFirstWayPoint().hasAttachedBody(weld_object_)) {
    RCLCPP_INFO(node_->get_logger(), "object already attached at start -> grab");
    publishWeld(true);
  }

  std::size_t begin = 0;
  bool executed_any = false;

  for (std::size_t i = 0; i <= leaves.size(); ++i) {
    const bool at_end = (i == leaves.size());
    const std::string name =
        (at_end || !leaves[i]->creator()) ? std::string{} : leaves[i]->creator()->name();

    const bool is_attach = (name == "attach_hollow");
    const bool is_detach = (name == "detach_objects");
    if (!at_end && !is_attach && !is_detach)
      continue;

    if (i > begin) {
      auto rc = executeSegment(*mg, leaves, begin, i);
      if (rc.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        return rc;
      executed_any = true;
    }

    if (is_attach) publishWeld(true);
    if (is_detach) publishWeld(false);
    begin = i + 1;       // 触发用的 leaf 本身没有运动，跳过
  }

  if (!executed_any)
    return moveit::core::MoveItErrorCode{ moveit_msgs::msg::MoveItErrorCodes::FAILURE };
  return moveit::core::MoveItErrorCode{ moveit_msgs::msg::MoveItErrorCodes::SUCCESS };
}

}  // namespace mtc
