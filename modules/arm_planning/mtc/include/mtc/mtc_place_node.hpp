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
enum class State { IDLE, PLANNING, EXECUTING, DONE, FAILED }; // 状态机
class MTCTaskNode {
public:
  MTCTaskNode(const rclcpp::NodeOptions &options);
  ~MTCTaskNode();

  /** TaskExecutor interface **/
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  /** Planning scene setup **/
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped &slot_pose);

private:
  rclcpp::Node::SharedPtr node_;

  std::string arm_group_;
  std::string hand_frame_;
  std::string planning_frame_;

  /** 目标位姿覆盖（/exchange_slot/slot_pose） **/
  void onTargetOverride(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  /** 规划 + 执行服务 **/
  void onPlanAndExecute(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  /** 从 ROS 参数读目标位姿 **/
  void loadTargetParams();
  /** 状态发布 **/
  void setState(State s);
  /** Task constructor **/
  moveit::task_constructor::Task createTask();
  moveit::task_constructor::Task createTestTask();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      slot_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr auto_start_timer_;

  /** 在独立工作线程中执行完整规划/执行管线，避免阻塞 executor **/
  void startPlanningPipelineAsync();
  /** 工作线程入口，执行规划管线主体 **/
  void startPlanningPipeline();

  // 互斥回调组
  rclcpp::CallbackGroup::SharedPtr cb_group_task_;    // service + auto_start
  rclcpp::CallbackGroup::SharedPtr cb_group_target_;  // 目标覆盖订阅

  // TCP → hollow
  tf2::Transform T_tcp_hollow_;
  bool computeTargetTCP(tf2::Transform &T_base_tcp_target,
                        tf2::Transform &T_base_hollow_target);
  void publishDebugTF(const tf2::Transform &T, const std::string &child_frame);

  geometry_msgs::msg::PoseStamped target_slot_;  // base_link 下的插槽位姿
  bool has_target_{false};
  bool allow_slot_override_{false};  // 是否允许话题覆盖 YAML 目标
  bool auto_start_{false};
  double auto_start_delay_{3.0};
  std::atomic<State> state_{State::IDLE};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::atomic<bool> task_running_{false};
  std::thread pipeline_thread_;

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