#pragma once

#include "mtc/common/task_node_base.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
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

namespace mtc {
  geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf);
class MTCTaskNode : public TaskNodeBase {
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions &options) : TaskNodeBase(options) {}
protected:
  moveit::task_constructor::Task createTask() override;
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped &target) override;
private:
  bool computeTargetTCP(tf2::Transform &, tf2::Transform &);
};
}
