#include "wust_arm_driver/wust_arm_driver.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace wust_arm_driver {
WustArmDriver::WustArmDriver(const rclcpp::NodeOptions &options)
    : Node("wust_arm_driver", options), owned_ctx_{new IoContext(2)},
      serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)} {
  RCLCPP_INFO(get_logger(), "Starting WustArmDriver!");
  getParams();

  joint_names_ = {"joint1", "joint2", "joint3", "joint4",
                  "joint5", "joint6", "joint7"};
  joint_positions_.assign(7, 0.0);
  joint_velocities_.assign(7, 0.0);
  joint_state_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, "main_controller/follow_joint_trajectory",
      std::bind(&WustArmDriver::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&WustArmDriver::handle_cancel, this, std::placeholders::_1),
      std::bind(&WustArmDriver::handle_accepted, this, std::placeholders::_1));
  // 仿真控制
  sim_target_positions_.assign(7, 0.0);
  sim_target_velocities_.assign(7, 0.0);

  if (use_fake_hardware_) {
    sim_running_ = true;
    sim_control_thread_ = std::thread(&WustArmDriver::simControlLoop, this);

    int interval_ms = static_cast<int>(1000.0 / state_publish_rate_);
    joint_state_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        std::bind(&WustArmDriver::publishFakeJointStates, this));
    publishFakeJointStates(); 
  }
  else {
    try {
      serial_driver_->init_port(device_name_, *device_config_);
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&WustArmDriver::receiveData, this);
    } catch (const std::exception &ex) {
      RCLCPP_FATAL(get_logger(), "Serial init failed: %s", ex.what());
      throw;
    }
  }
}

WustArmDriver::~WustArmDriver() {
  if (!use_fake_hardware_) {
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (serial_driver_ && serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void WustArmDriver::simControlLoop()
{
  double dt = 1.0 / sim_control_freq_;
  rclcpp::Rate rate(sim_control_freq_);

  while (rclcpp::ok() && sim_running_) {

    for (size_t i = 0; i < joint_positions_.size(); ++i) {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      double pos = joint_positions_[i];
      double vel = joint_velocities_[i];
      double target_pos = sim_target_positions_[i];
      double target_vel = sim_target_velocities_[i];

      // ===== PID 控制 =====
      double error = target_pos - pos;
      double vel_error = target_vel - vel;

      double cmd_vel = sim_kp_ * error + sim_kv_ * vel_error;

      // ===== 限速 =====
      cmd_vel = std::clamp(cmd_vel, -sim_max_speed_, sim_max_speed_);

      // ===== 积分 =====
      joint_positions_[i] += cmd_vel * dt;
      joint_velocities_[i] = cmd_vel;
    }

    rate.sleep();
  }
}
void WustArmDriver::publishFakeJointStates()
{
  auto now = this->now();

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now;
  msg.name = joint_names_;
  {
    std::lock_guard<std::mutex> lock(sim_mutex_);
    msg.position = joint_positions_;
    msg.velocity = joint_velocities_;
  }

  joint_state_pub_->publish(msg);
}

void WustArmDriver::getParams() {
  // 声明参数
  this->declare_parameter<bool>("use_fake_hardware", true);
  this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);
  this->declare_parameter<double>("state_publish_rate", 50.0);
  this->declare_parameter<double>("goal_tolerance", 0.01);
  this->declare_parameter<double>("goal_timeout", 5.0);
  this->declare_parameter<int>("controller_freq", 100);
  this->declare_parameter<bool>("debug", true);
  this->declare_parameter<bool>("debug_single_point", false);
  // 时间参数化相关
  this->declare_parameter<double>("totg_path_tolerance", 0.1);
  this->declare_parameter<double>("totg_min_angle_change", 0.001);
  // 仿真控制相关
  this->declare_parameter<double>("sim_max_speed", 1.5);
  this->declare_parameter<double>("sim_control_freq", 1000.0);
  this->declare_parameter<double>("sim_kp", 5.0);
  this->declare_parameter<double>("sim_kv", 0.5);

  // 获取参数值并赋值给成员变量
  use_fake_hardware_ = this->get_parameter("use_fake_hardware").as_bool();
  device_name_ = this->get_parameter("device_name").as_string();
  int baud_rate = this->get_parameter("baud_rate").as_int();
  state_publish_rate_ = this->get_parameter("state_publish_rate").as_double();
  goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
  goal_timeout_ = this->get_parameter("goal_timeout").as_double();
  controller_freq_ = this->get_parameter("controller_freq").as_int();
  debug_ = this->get_parameter("debug").as_bool();
  debug_single_point_ = this->get_parameter("debug_single_point").as_bool();
  totg_path_tolerance_ = this->get_parameter("totg_path_tolerance").as_double();
  totg_min_angle_change_ = this->get_parameter("totg_min_angle_change").as_double();
  sim_control_freq_ = this->get_parameter("sim_control_freq").as_double();
  sim_kp_ = this->get_parameter("sim_kp").as_double();
  sim_kv_ = this->get_parameter("sim_kv").as_double();
  sim_max_speed_ = this->get_parameter("sim_max_speed").as_double();
  // 打印确认
  // RCLCPP_INFO(this->get_logger(), "Mode: %s, Freq: %d, Tolerance: %.3f",
  //             use_fake_hardware_ ? "FAKE" : "REAL", controller_freq_,
  //             goal_tolerance_);

  // 串口配置
  auto fc = drivers::serial_driver::FlowControl::NONE;
  auto pt = drivers::serial_driver::Parity::NONE;
  auto sb = drivers::serial_driver::StopBits::ONE;
  device_config_ = std::make_unique<drivers::serial_driver::SerialPortConfig>(
      baud_rate, fc, pt, sb);
}

// Action Server 回调实现
rclcpp_action::GoalResponse
WustArmDriver::handle_goal(const rclcpp_action::GoalUUID &,
                           std::shared_ptr<const FollowJointTrajectory::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WustArmDriver::handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WustArmDriver::handle_accepted(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle) {
  // 在独立线程中执行，避免阻塞主循环
  std::thread{std::bind(&WustArmDriver::execute, this, std::placeholders::_1),
              goal_handle}
      .detach();
}

void WustArmDriver::execute(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
{
  if (!robot_model_) {
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt RobotModel...");
    initMoveIt();
  }

  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<FollowJointTrajectory::Result>();

  // === 1. 构造 RobotTrajectory ===
  robot_trajectory::RobotTrajectory rt(robot_model_, joint_model_group_);

  moveit::core::RobotState start_state(robot_model_);
  start_state.setVariablePositions(joint_names_, joint_positions_);
  start_state.update();

  rt.setRobotTrajectoryMsg(start_state, goal->trajectory);

  // === 2. 时间参数化（统一处理所有段）===
  double totg_resample_dt = 1.0 / controller_freq_;
  trajectory_processing::TimeOptimalTrajectoryGeneration totg(
    totg_path_tolerance_,
    totg_resample_dt,
    totg_min_angle_change_
  );
  if (!totg.computeTimeStamps(rt)) {
    RCLCPP_WARN(this->get_logger(), "TOTG failed, using original trajectory");
  } else {
    RCLCPP_INFO(this->get_logger(), "TOTG success");
  }

  // === 3. 转回消息 ===
  moveit_msgs::msg::RobotTrajectory rt_msg;
  rt.getRobotTrajectoryMsg(rt_msg);

  const auto& traj = rt_msg.joint_trajectory.points;

  if (traj.size() < 2) {
    goal_handle->abort(result);
    return;
  }

  // debug
  if (debug_) {
    RCLCPP_INFO(this->get_logger(), "===== EXECUTE TRAJECTORY =====");

    for (size_t i = 0; i < traj.size(); ++i) {
      double t = traj[i].time_from_start.sec +
                traj[i].time_from_start.nanosec * 1e-9;

      std::ostringstream oss;
      oss << "pt " << i << " time=" << std::fixed << std::setprecision(3) << t;

      oss << " | pos: [";
      for (size_t j = 0; j < traj[i].positions.size(); ++j) {
        oss << std::setprecision(3) << traj[i].positions[j];
        if (j != traj[i].positions.size() - 1) oss << ", ";
      }
      oss << "]";

      oss << " | vel: [";
      for (size_t j = 0; j < traj[i].velocities.size(); ++j) {
        oss << std::setprecision(3) << traj[i].velocities[j];
        if (j != traj[i].velocities.size() - 1) oss << ", ";
      }
      oss << "]";

      RCLCPP_DEBUG(this->get_logger(), "%s", oss.str().c_str());
    }
  }

  // === 4. 按时间执行 ===
  rclcpp::Time start_time = this->now();

  for (size_t i = 0; i < traj.size(); ++i) {
    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    const auto& pt = traj[i];

    double t = pt.time_from_start.sec +
               pt.time_from_start.nanosec * 1e-9;

    rclcpp::Time target_time = start_time + rclcpp::Duration::from_seconds(t);

    // === 等待到该时间点 ===
    while (this->now() < target_time) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if (use_fake_hardware_) {
      std::lock_guard<std::mutex> lock(sim_mutex_);
      sim_target_positions_ = pt.positions;

      if (pt.velocities.size() == pt.positions.size())
        sim_target_velocities_ = pt.velocities;
      else
        sim_target_velocities_.assign(pt.positions.size(), 0.0);
    }
    else {
      // === 真机：发送目标点 ===
      SendPacket packet;
      for (size_t j = 0; j < pt.positions.size(); ++j) {
        packet.target_joint_positions[j] = pt.positions[j];
      if (pt.velocities.size() == pt.positions.size())
        packet.target_joint_velocities[j] = pt.velocities[j];
      else
        packet.target_joint_velocities[j] = 0.0;
      }

      crc16::Append_CRC16_Check_Sum(
          reinterpret_cast<uint8_t*>(&packet), sizeof(packet));

      serial_driver_->port()->send(toVector(packet));
    }
  }

  // ===== 5. 等待最终收敛（核心） =====
  const auto& final_target = traj.back().positions;
  bool reached = false;
  rclcpp::Time wait_start = this->now();
  while (rclcpp::ok()) {

    if (goal_handle->is_canceling()) {
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      goal_handle->canceled(result);
      return;
    }

    if (check_goal_reached(final_target)) {
      reached = true;
      break;
    }
    double elapsed = (this->now() - wait_start).seconds();
    if (elapsed > goal_timeout_) {
      RCLCPP_WARN(this->get_logger(),
                  "Goal timeout! elapsed=%.3f", elapsed);
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  // ===== 6. 返回结果 =====
  if (reached) {
    RCLCPP_INFO(this->get_logger(), "Trajectory execution SUCCESS");
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    goal_handle->succeed(result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Trajectory execution FAILED");
    result->error_code =
        FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
    goal_handle->abort(result);
  }
}

// 辅助函数：判断所有关节是否进入容差范围
bool WustArmDriver::check_goal_reached(
    const std::vector<double>& target_positions)
{
  std::lock_guard<std::mutex> lock(sim_mutex_);

  for (size_t i = 0; i < joint_positions_.size(); ++i) {
    if (std::abs(joint_positions_[i] - target_positions[i]) > goal_tolerance_) {
      return false;
    }
  }
  return true;
}
void WustArmDriver::initMoveIt()
{
  robot_model_loader::RobotModelLoader loader(this->shared_from_this(), "robot_description");
  robot_model_ = loader.getModel();

  if (!robot_model_) {
    throw std::runtime_error("robot_model load failed");
  }

  joint_model_group_ = robot_model_->getJointModelGroup("main");
}


void WustArmDriver::receiveData() {
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivePacket) - 1);
        serial_driver_->port()->receive(data);
        data.insert(data.begin(), header[0]);

        ReceivePacket packet = fromVector(data);

        if (crc16::Verify_CRC16_Check_Sum(
                reinterpret_cast<const uint8_t *>(&packet), sizeof(packet))) {
          auto msg = sensor_msgs::msg::JointState();
          // 使用 this->now() 保证 TF 同步
          msg.header.stamp = this->now();
          msg.name = joint_names_;
          for (int i = 0; i < 7; ++i) {
            double pos = packet.current_joint_positions[i];
            joint_positions_[i] = pos;
            RCLCPP_DEBUG(this->get_logger(),
                         "Receive Joint %d: %.3f rad (%.2f deg)", i + 1, pos,
                         pos * 180.0 / M_PI);

            msg.position.push_back(packet.current_joint_positions[i]);
          }
          joint_state_pub_->publish(msg);
        }
      }
    } catch (const std::exception &ex) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 20, "Receive error: %s",
                            ex.what());
      // 避免死循环
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

} // namespace wust_arm_driver

RCLCPP_COMPONENTS_REGISTER_NODE(wust_arm_driver::WustArmDriver)