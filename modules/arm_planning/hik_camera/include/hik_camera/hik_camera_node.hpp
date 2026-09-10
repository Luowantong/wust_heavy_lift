#ifndef HIK_CAMERA_NODE_HPP_
#define HIK_CAMERA_NODE_HPP_

#include "MvCameraControl.h"
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera {
class HikCameraNode : public rclcpp::Node {
public:
  explicit HikCameraNode(const rclcpp::NodeOptions &options);
  ~HikCameraNode() override;

private:
  void declareParameters();
  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

  // 核心发布逻辑
  void hikCameraLoop();
  void localFileLoop();

  // 海康相机相关
  int nRet = MV_OK;
  void *camera_handle_ = nullptr;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  // ROS 2 相关
  image_transport::CameraPublisher camera_pub_;
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  // 配置与线程
  std::string input_type_; // "camera", "video", "image"
  std::string file_path_;
  int fail_count_ = 0;
  std::thread capture_thread_;
  bool is_running_ = true;
};
} // namespace hik_camera

#endif