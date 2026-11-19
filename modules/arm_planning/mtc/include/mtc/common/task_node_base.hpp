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
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include <tf2/LinearMath/Vector3.h>

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
  // 目标定稿钩子：place 用它做 IK 种子阶梯 + YAML 降级（允许改写 target）。
  // 必须在 setupPlanningScene 之前跑完，否则站体碰撞模型会和运动目标错开。
  virtual bool resolveTarget(geometry_msgs::msg::PoseStamped & /*target*/) { return true; }
  void publishDebugTF(const tf2::Transform &T, const std::string &child_frame);

  rclcpp::Node::SharedPtr node_;
  std::string arm_group_, hand_frame_, planning_frame_;
  geometry_msgs::msg::PoseStamped target_slot_;
  // YAML 里配的原始目标（override 之前）。锁定的一致性门限和 IK 降级都拿它当基准。
  geometry_msgs::msg::PoseStamped yaml_slot_;
  // 本次运行的目标快照。订阅回调只改 target_slot_；管线里
  // setupPlanningScene / createTask 一律读这一份，避免"场景按 A 帧建、运动按 B 帧走"。
  geometry_msgs::msg::PoseStamped resolved_target_;
  const geometry_msgs::msg::PoseStamped &target() const { return resolved_target_; }
  bool has_target_{false};
  bool allow_slot_override_{false};
  // ---- 目标锁定：站位是固定的，只在"站体无遮挡"时采一次样，之后全程复用 ----
  bool target_lock_{false};
  // 写：订阅回调（cb_group_target_）；读：pipeline 工作线程。跨线程，必须原子。
  // 配套 lock_mtx_ / lock_cv_：观察阶段的事件驱动等待（见 moveToObservationPose）。
  std::atomic<bool> target_locked_{false};
  std::mutex lock_mtx_;
  std::condition_variable lock_cv_;
  int target_lock_frames_{10};            // 连续多少帧一致才采纳
  double target_lock_max_spread_{0.002};  // 这几帧之间的最大互差
  double target_lock_min_distance_{0.35}; // 末端离站体多远才算"无遮挡"
  double target_lock_max_offset_{0.015};  // 与 YAML 偏离超过这个值就拒锁
  std::deque<tf2::Vector3> target_samples_;
  // ---- 观察位：只有 place 单独跑/调试时才需要（比赛流程 pick→place 手臂本来就在远处）----
  bool observe_pose_before_place_{false};
  std::vector<double> observe_pose_;      // main 组 7 个关节角；为空则整段跳过
  double observe_lock_timeout_{3.0};      // 等锁定的上限（17 Hz 下 10 帧约 0.6 s）
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
  // 末端离站体太近时，单元本体正挡在相机和站体之间，关键点必然偏。
  // 返回 false 表示这一帧不许用来锁定。
  bool stationUnoccluded(const geometry_msgs::msg::Pose &target);
  void onPlanAndExecute(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void loadTargetParams();
  void setState(State s);
  void startPlanningPipelineAsync();
  void startPlanningPipeline();
  // 观察位：关节空间移动 + 清空旧采样（不中止任务，失败/超时都退回 YAML）
  void moveToObservationPose();

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slot_pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr auto_start_timer_;
  rclcpp::CallbackGroup::SharedPtr cb_group_task_, cb_group_target_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace mtc