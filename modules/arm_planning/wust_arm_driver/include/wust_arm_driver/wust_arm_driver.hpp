#ifndef WUST_ARM_DRIVER__WUST_ARM_DRIVER_HPP_
#define WUST_ARM_DRIVER__WUST_ARM_DRIVER_HPP_

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <eigen3/Eigen/Dense>
#include <moveit/kinematics_base/kinematics_base.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

// C++ system
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "wust_arm_driver/crc.hpp"
#include "wust_arm_driver/packet.hpp"

namespace wust_arm_driver {
class WustArmDriver : public rclcpp::Node {
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
      rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  explicit WustArmDriver(const rclcpp::NodeOptions &options);
  ~WustArmDriver() override;

private:
  // 获取参数
  void getParams();

  // Action 回调
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const FollowJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);

  // 执行与接收逻辑
  void
  execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle);
  void receiveData();

  // 串口成员
  std::unique_ptr<IoContext> owned_ctx_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::string device_name_;

  // 线程与同步
  std::thread receive_thread_;

  // ROS 接口
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  std::vector<std::string> joint_names_;

  // 无串口
  bool use_fake_hardware_{false};
  std::vector<double> joint_positions_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  void publishFakeJointStates();

  // ===== 仿真控制 =====
  std::vector<double> sim_target_positions_;
  std::vector<double> sim_target_velocities_;

  std::thread sim_control_thread_;
  std::mutex sim_mutex_;
  bool sim_running_{false};

  double sim_control_freq_;   // 例如 1000 Hz
  double sim_kp_;             // PID参数
  double sim_kv_;
  double sim_max_speed_;
  void simControlLoop();

  // 判断是否成功到达目标点
  double goal_tolerance_; // 允许误差 (rad)
  double goal_timeout_;   // 允许超时 (s)
  bool check_goal_reached(const std::vector<double> &target_positions);

  int controller_freq_;     // 发送频率 (Hz)
  bool debug_;              // 调试模式
  bool debug_single_point_; // 只发送一个点（最后一个）

  double state_publish_rate_; // joint_state发布频率 (Hz)

  std::vector<double> joint_velocities_;
  // 时间参数化
  moveit::core::RobotModelPtr robot_model_;
  const moveit::core::JointModelGroup* joint_model_group_;
  double totg_path_tolerance_ ;
  double totg_min_angle_change_;
  // 初始化 robot_model_
  void initMoveIt();
};
} // namespace wust_arm_driver

#endif // WUST_ARM_DRIVER__WUST_ARM_DRIVER_HPP_