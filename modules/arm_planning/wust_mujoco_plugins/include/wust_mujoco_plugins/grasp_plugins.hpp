#pragma once

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>

namespace wust_mujoco_plugins {

class GraspPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase {
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void pre_step(mjData* data) override;
  void on_reset(mjData* data) override;
  void cleanup() override;

private:
  int grasp_eq_id_{-1};
  int rest_eq_id_{-1};
  std::atomic<bool> attached_{false};
  rclcpp::Subscription<moveit_msgs::msg::AttachedCollisionObject>::SharedPtr sub_;
};

} // namespace wust_mujoco_plugins