#include "detector/detector_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
using GetDetectorState = detector_interfaces::srv::GetDetectorState;
using SetDetectorState = detector_interfaces::srv::SetDetectorState;
namespace exchange_slot {
ExchangeSlotDetectorNode::ExchangeSlotDetectorNode(
    const rclcpp::NodeOptions &options)
    : Node("exchange_slot_detector", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing ExchangeSlotDetectorNode!");

  detector_ = std::make_unique<ExchangeSlotDetector>();

  // 1. 参数与 Marker 初始化
  this->declare_parameter("debug", true);
  this->declare_parameter("no_hardware", false);
  this->declare_parameter("stable_threshold", 15);
  this->declare_parameter("pos_tolerance", 0.005);
  this->declare_parameter("ori_tolerance", 2.0); // 默认 2 度
  this->declare_parameter("force_lock", false);
  this->declare_parameter("lost_buffer_limit", 5); // 允许丢失 5 帧
  this->declare_parameter("square_size", 0.10);
  this->declare_parameter("binary_thresh", 150);

  debug_ = this->get_parameter("debug").as_bool();
  no_hardware_ = this->get_parameter("no_hardware").as_bool();
  stable_threshold_ = this->get_parameter("stable_threshold").as_int();
  pos_tolerance_ = this->get_parameter("pos_tolerance").as_double();
  ori_tolerance_ = this->get_parameter("ori_tolerance").as_double();
  force_lock_ = this->get_parameter("force_lock").as_bool();
  lost_buffer_limit_ = this->get_parameter("lost_buffer_limit").as_int();
  double square_size = this->get_parameter("square_size").as_double();

  // 初始化TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // 初始化底座 Marker (正方形)
  base_marker_.ns = "slot_base";
  base_marker_.type = visualization_msgs::msg::Marker::CUBE;
  base_marker_.scale.x = square_size;
  base_marker_.scale.y = square_size;
  base_marker_.scale.z = 0.01; // 薄片
  base_marker_.color.r = 0.0;
  base_marker_.color.g = 1.0;
  base_marker_.color.b = 0.0;
  base_marker_.color.a = 0.5;
  base_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 初始化圆柱体 Marker
  cylinder_marker_.ns = "slot_cylinder";
  cylinder_marker_.type = visualization_msgs::msg::Marker::CYLINDER;
  cylinder_marker_.scale.x = 0.036;
  cylinder_marker_.scale.y = 0.036;
  cylinder_marker_.scale.z = 0.10; // 高度
  cylinder_marker_.color.r = 1.0;
  cylinder_marker_.color.g = 0.0;
  cylinder_marker_.color.b = 0.0;
  cylinder_marker_.color.a = 0.8;
  cylinder_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);

  // 2. 发布者与订阅者
  slot_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/exchange_slot/slot_pose", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/exchange_slot/marker", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  if (debug_)
    createDebugPublishers();

  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
        cv::Mat k(3, 3, CV_64F, const_cast<double *>(camera_info->k.data()));
        cv::Mat d(1, 5, CV_64F, const_cast<double *>(camera_info->d.data()));
        detector_->setCameraParams(k.clone(), d.clone());
        cam_info_sub_.reset();
      });

  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ExchangeSlotDetectorNode::imageCallback, this,
                std::placeholders::_1));
  // 3. 服务
  get_state_srv_ = this->create_service<GetDetectorState>(
      "/detector/get_state",
      std::bind(&ExchangeSlotDetectorNode::getStateCb, this,
                std::placeholders::_1, std::placeholders::_2));

  set_state_srv_ = this->create_service<SetDetectorState>(
      "/detector/set_state",
      std::bind(&ExchangeSlotDetectorNode::setStateCb, this,
                std::placeholders::_1, std::placeholders::_2));
}

void ExchangeSlotDetectorNode::imageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
  auto cv_ptr = cv_bridge::toCvShare(img_msg, "rgb8");
  cv::Mat img = cv_ptr->image.clone();

  // 实时更新识别参数
  detector_->setParams(this->get_parameter("binary_thresh").as_int(),
                       this->get_parameter("square_size").as_double(), debug_);

  bool detected = detector_->detect(img);
  auto latency = (this->now() - img_msg->header.stamp).seconds() * 1000.0;

  if (detected) {

    lost_frames_counter_ = 0; // 只要识别到，重置丢失缓冲
    auto pose_data = detector_->getLastPose();

    // 1. 构造相机坐标系下的位姿
    geometry_msgs::msg::PoseStamped pose_in_camera;
    pose_in_camera.header = img_msg->header;
    pose_in_camera.header.stamp = rclcpp::Time(0);
    pose_in_camera.pose.position.x = pose_data.tvec[0];
    pose_in_camera.pose.position.y = pose_data.tvec[1];
    pose_in_camera.pose.position.z = pose_data.tvec[2];

    cv::Mat rot_mat;
    cv::Rodrigues(pose_data.rvec, rot_mat);
    tf2::Matrix3x3 tf2_rot(rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1),
                           rot_mat.at<double>(0, 2), rot_mat.at<double>(1, 0),
                           rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
                           rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1),
                           rot_mat.at<double>(2, 2));
    tf2::Quaternion q;
    tf2_rot.getRotation(q);
    pose_in_camera.pose.orientation = tf2::toMsg(q);

    try {

      // 2. 转换到 base_link (消除相机运动影响)
      geometry_msgs::msg::PoseStamped pose_in_world = tf_buffer_->transform(
          pose_in_camera, "base_link", tf2::durationFromSec(0.05));

      pose_in_world.header.frame_id = "base_link";

      // 缩放修正 (实验调参)
      if(no_hardware_)
      {
        pose_in_world.pose.position.x *= 1.0;
        pose_in_world.pose.position.y *= 1.2;
        pose_in_world.pose.position.z *= 1.0;
      }

      // 3. 状态机逻辑
      if (current_state_ == State::LOST) {

        current_state_ = State::TRACKING;
        locked_pose_ = pose_in_world.pose;
        stable_frames_ = 0;

      } else if (current_state_ == State::TRACKING) {

        if (isStable(pose_in_world.pose, locked_pose_)) {
          stable_frames_++;

          if (stable_frames_ >= stable_threshold_ || force_lock_) {
            current_state_ = State::LOCKED;
            locked_pose_ = pose_in_world.pose; // 锁定最终值
            RCLCPP_INFO(this->get_logger(),
                        "Target LOCKED at stable position.");
          }
        } else {
          // 如果位姿跳变剧烈，说明没稳住，以最新帧为基准重新计数
          stable_frames_ = 0;
          locked_pose_ = pose_in_world.pose;
        }
      }

      // LOCKED / PLANNING / EXECUTING 状态下不更新 locked_pose_
      // 视觉完全冻结，只靠之前锁定的位姿保持稳定，直到收到新的状态命令
      // 4. 统一发布逻辑
      geometry_msgs::msg::PoseStamped final_out_pose = pose_in_world;
      final_out_pose.header.frame_id = "base_link"; // 明确输出坐标系

      if (current_state_ == State::LOCKED ||
          current_state_ == State::PLANNING ||
          current_state_ == State::EXECUTING) {
        final_out_pose.pose = locked_pose_;
      }

      slot_pose_pub_->publish(final_out_pose);

      geometry_msgs::msg::TransformStamped t;
      t.header = final_out_pose.header;
      t.header.frame_id = "base_link";
      t.child_frame_id = "target_slot_fixed";
      t.transform.translation.x = final_out_pose.pose.position.x;
      t.transform.translation.y = final_out_pose.pose.position.y;
      t.transform.translation.z = final_out_pose.pose.position.z;
      t.transform.rotation = final_out_pose.pose.orientation;
      tf_broadcaster_->sendTransform(t);

      if (debug_) {
        publishMarkers(final_out_pose.pose, final_out_pose.header);
        detector_->drawResults(img);
      }

    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "TF Transform failed: %s", ex.what());
    }

  } else {

    // 5. 丢失处理
    // EXECUTING / PLANNING 不允许自动 LOST，必须等外部命令切回 LOST
    if (current_state_ != State::PLANNING &&
        current_state_ != State::EXECUTING && current_state_ != State::LOST) {
      lost_frames_counter_++;

      if (lost_frames_counter_ > lost_buffer_limit_) {
        current_state_ = State::LOST;
        stable_frames_ = 0;
        RCLCPP_WARN(this->get_logger(),
                    "Target lost for %d frames, resetting state.",
                    lost_buffer_limit_);
      }
    }
  }

  // 6. 状态显示
  if (debug_) {

    std::string state_str[] = {"LOST", "TRACKING", "LOCKED", "PLANNING",
                               "EXECUTING"};

    std::stringstream ss;
    ss << "State: " << state_str[(int)current_state_]
       << " | Latency: " << std::fixed << std::setprecision(2) << latency
       << "ms";

    cv::Scalar color = (current_state_ == State::LOCKED)
                           ? cv::Scalar(0, 255, 0)
                           : cv::Scalar(0, 255, 255);

    cv::putText(img, ss.str(), cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1.2,
                color, 2);

    result_img_pub_.publish(
        cv_bridge::CvImage(img_msg->header, "rgb8", img).toImageMsg());
  }
}

void ExchangeSlotDetectorNode::publishMarkers(
    const geometry_msgs::msg::Pose &pose, const std_msgs::msg::Header &header) {
  visualization_msgs::msg::MarkerArray ma;

  // 输入的 pose 是 world/base_link 坐标系下的
  base_marker_.header = header;
  base_marker_.pose = pose;
  base_marker_.id = 0;

  cylinder_marker_.header = header;

  // 计算圆柱体相对于底座的偏移
  tf2::Transform tf_base, tf_offset, tf_cylinder;
  tf2::fromMsg(pose, tf_base);
  // 向上偏移 base_thickness / 2.0 + cylinder_height
  // / 2.0，使圆柱恰好处于底座上方
  tf_offset.setOrigin(tf2::Vector3(0, 0, 0.01 / 2.0 + 0.1 / 2.0));
  tf_offset.setRotation(tf2::Quaternion::getIdentity());

  tf_cylinder = tf_base * tf_offset;
  tf2::toMsg(tf_cylinder, cylinder_marker_.pose);
  cylinder_marker_.id = 1;

  ma.markers.push_back(base_marker_);
  ma.markers.push_back(cylinder_marker_);
  marker_pub_->publish(ma);
}

void ExchangeSlotDetectorNode::createDebugPublishers() {
  binary_img_pub_ =
      image_transport::create_publisher(this, "/exchange_slot/binary_img");
  result_img_pub_ =
      image_transport::create_publisher(this, "/exchange_slot/result_img");
}

void ExchangeSlotDetectorNode::destroyDebugPublishers() {
  binary_img_pub_.shutdown();
  result_img_pub_.shutdown();
}

bool ExchangeSlotDetectorNode::isStable(const geometry_msgs::msg::Pose &p1,
                                        const geometry_msgs::msg::Pose &p2) {
  // 位置约束
  double d = std::sqrt(std::pow(p1.position.x - p2.position.x, 2) +
                       std::pow(p1.position.y - p2.position.y, 2) +
                       std::pow(p1.position.z - p2.position.z, 2));
  if (d > pos_tolerance_)
    return false;

  // 角度约束 (使用 tf2 提取角度差)
  tf2::Quaternion q1, q2;
  tf2::fromMsg(p1.orientation, q1);
  tf2::fromMsg(p2.orientation, q2);
  double angle_diff = q1.angleShortestPath(q2) * 180.0 / M_PI; // 弧度转角度

  return angle_diff < ori_tolerance_;
}

void ExchangeSlotDetectorNode::getStateCb(
    const std::shared_ptr<GetDetectorState::Request> request,
    std::shared_ptr<GetDetectorState::Response> response) {
  (void)request;
  response->state = static_cast<uint8_t>(current_state_);
}
void ExchangeSlotDetectorNode::setStateCb(
    const std::shared_ptr<SetDetectorState::Request> request,
    std::shared_ptr<SetDetectorState::Response> response) {
  current_state_ = static_cast<State>(request->state);

  // 定义状态名称映射
  std::string state_names[] = {"LOST", "TRACKING", "LOCKED", "PLANNING",
                               "EXECUTING"};
  std::string current_name =
      (request->state < 5) ? state_names[request->state] : "UNKNOWN";

  RCLCPP_INFO(this->get_logger(), "State changed to: %s", current_name.c_str());

  response->success = true;
}

} // namespace exchange_slot
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(exchange_slot::ExchangeSlotDetectorNode)