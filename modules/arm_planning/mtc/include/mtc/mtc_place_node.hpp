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
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <thread>

#include "detector_interfaces/srv/get_detector_state.hpp"
#include "detector_interfaces/srv/set_detector_state.hpp"

using GetDetectorState = detector_interfaces::srv::GetDetectorState;
using SetDetectorState = detector_interfaces::srv::SetDetectorState;

namespace mtc {
constexpr double slot_base_square_size = 0.10;             // 底座边长
constexpr double slot_base_height = 0.01;                  // 底座厚度
constexpr double slot_cylinder_height = 0.10;              // 兑换柱高度
constexpr double slot_cylinder_radius = 0.018;             // 兑换柱半径
constexpr double hollow_cylinder_h = 0.15;                 // 能量单元高度
constexpr double hollow_cylinder_r = 0.0325;               // 能量单元半径
constexpr double insert_offset_ = hollow_cylinder_h / 2.0; // 插入深度
constexpr double pre_place_height_ = slot_cylinder_height +
                                     hollow_cylinder_h / 2.0 +
                                     0.01; // 预放置点高度（在插槽上方）
enum class State { LOST, TRACKING, LOCKED, PLANNING, EXECUTING }; // 状态机
class MTCTaskNode {
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);
  ~MTCTaskNode();

  /** TaskExecutor interface **/
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  /** Trigger task manually **/
  void doTask();

  /** Planning scene setup **/
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped &slot_pose);

private:
  /** Callback **/
  void slotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  /** Task constructor **/
  moveit::task_constructor::Task createTask();
  moveit::task_constructor::Task createTestTask();
  /** Detector State **/
  void setDetectorState(uint8_t state);

private:
  rclcpp::Node::SharedPtr node_;

  std::string arm_group_;
  std::string hand_frame_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      slot_pose_sub_;

  // 状态机管理
  rclcpp::Client<GetDetectorState>::SharedPtr get_state_client_;
  rclcpp::Client<SetDetectorState>::SharedPtr set_state_client_;
  rclcpp::TimerBase::SharedPtr task_timer_;
  void
  handleGetStateResponse(rclcpp::Client<GetDetectorState>::SharedFuture future);
  /** 在独立工作线程中执行完整规划/执行管线，避免阻塞 executor **/
  void startPlanningPipelineAsync();
  /** 工作线程入口，执行规划管线主体（运行时行为与原同步版一致） **/
  void startPlanningPipeline();
  // 互斥回调组
  rclcpp::CallbackGroup::SharedPtr cb_group_service_;
  rclcpp::CallbackGroup::SharedPtr cb_group_timer_;
  // TCP → hollow
  tf2::Transform T_tcp_hollow_;
  bool computeTargetTCP(tf2::Transform &T_base_tcp_target,
                        tf2::Transform &T_base_hollow_target);
  void publishDebugTF(const tf2::Transform &T, const std::string &child_frame);

  geometry_msgs::msg::PoseStamped
      latest_slot_pose_; // 视觉系统提供的插槽位姿，已经转换到
                         // base_link 坐标系下
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::atomic<bool> task_running_{false};
  std::thread pipeline_thread_; // 规划/执行工作线程，析构时 join

  moveit::task_constructor::Task task_;
};
geometry_msgs::msg::Pose tfToPose(const tf2::Transform &tf);
} // namespace mtc_place

// 解算正确：
// T_target_hollow 在 target_slot_fixed 正上方，且z轴方向相反

// T_base_slot       (视觉)
// T_slot_hollow     (预定义)
// T_tcp_hollow      (URDF计算)
// 要得到：T_base_tcp
// 变换链为：
// T_base_hollow = T_base_slot * T_slot_hollow
// T_base_tcp = T_base_hollow * (T_tcp_hollow)^-1