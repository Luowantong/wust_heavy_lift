#pragma once

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

  enum class State { IDLE, PLANNING, EXECUTING, DONE, FAILED };

class TaskNodeBase {
public:
    explicit TaskNodeBase(const rclcpp::NodeOptions &options,
                        const std::string &node_name,
                        const std::string &target_prefix);
  virtual ~TaskNodeBase();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

protected:
  virtual moveit::task_constructor::Task createTask() = 0;
  virtual void setupPlanningScene(const geometry_msgs::msg::PoseStamped &target) = 0;
  void publishDebugTF(const tf2::Transform &T, const std::string &child_frame);

  rclcpp::Node::SharedPtr node_;
  std::string arm_group_, hand_frame_, planning_frame_;
  geometry_msgs::msg::PoseStamped target_slot_;
  bool has_target_{false};
  bool allow_slot_override_{false};
  bool auto_start_{false};
  double auto_start_delay_{3.0};
  std::atomic<State> state_{State::IDLE};
  std::atomic<bool> task_running_{false};
  std::thread pipeline_thread_;
  std::string target_prefix_;
  moveit::task_constructor::Task task_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::Transform T_tcp_hollow_;

private:
  void onTargetOverride(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPlanAndExecute(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void loadTargetParams();
  void setState(State s);
  void startPlanningPipelineAsync();
  void startPlanningPipeline();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slot_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr auto_start_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_task_, cb_group_target_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace mtc