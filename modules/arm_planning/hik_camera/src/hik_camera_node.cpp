#include "hik_camera/hik_camera_node.hpp"

namespace hik_camera {
HikCameraNode::HikCameraNode(const rclcpp::NodeOptions &options)
    : Node("hik_camera", options) {
  RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

  // --- 1. 参数声明与获取 ---
  // 必须先 declare 才能 get
  input_type_ = this->declare_parameter("input_type", "camera");
  file_path_ = this->declare_parameter("file_path", "");
  std::string camera_name =
      this->declare_parameter("camera_name", "narrow_stereo");
  auto camera_info_url = this->declare_parameter(
      "camera_info_url", "package://hik_camera/config/camera_info.yaml");
  bool use_sensor_data_qos =
      this->declare_parameter("use_sensor_data_qos", true);

  // --- 2. 加载 Camera Info ---
  camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this,
                                                               camera_name);
  if (camera_info_manager_->validateURL(camera_info_url)) {
    camera_info_manager_->loadCameraInfo(camera_info_url);
    camera_info_msg_ = camera_info_manager_->getCameraInfo();
    RCLCPP_INFO(this->get_logger(), "Loaded camera info from: %s",
                camera_info_url.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                camera_info_url.c_str());
  }

  // --- 3. 设置发布者 QoS ---
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data
                                 : rmw_qos_profile_default;
  camera_pub_ =
      image_transport::create_camera_publisher(this, "image_raw", qos);

  // --- 4. 分模式初始化 ---
  if (input_type_ == "camera") {
    RCLCPP_INFO(this->get_logger(), "Mode: Real Camera. Initializing SDK...");

    MV_CC_DEVICE_INFO_LIST device_list;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);

    // 如果没插相机，会循环等待，不会崩溃
    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "No Hik Camera found! Please check hardware. Retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(2));
      MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    if (rclcpp::ok()) {
      MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);
      MV_CC_OpenDevice(camera_handle_);

      MV_CC_GetImageInfo(camera_handle_, &img_info_);
      image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

      // 声明海康特有的硬件参数（曝光、增益）
      declareParameters();
      MV_CC_StartGrabbing(camera_handle_);

      params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
          &HikCameraNode::parametersCallback, this, std::placeholders::_1));

      capture_thread_ = std::thread(&HikCameraNode::hikCameraLoop, this);
    }
  } else {
    RCLCPP_INFO(this->get_logger(), "Mode: %s. Using OpenCV reader.",
                input_type_.c_str());
    if (file_path_.empty()) {
      RCLCPP_FATAL(this->get_logger(),
                   "File path is empty! Please set 'file_path' parameter.");
      return;
    }
    // 视频或图片模式不需要初始化任何海康 SDK
    capture_thread_ = std::thread(&HikCameraNode::localFileLoop, this);
  }
}

void HikCameraNode::hikCameraLoop() {
  MV_FRAME_OUT out_frame;
  image_msg_.header.frame_id = "camera_optical_frame";
  image_msg_.encoding = "rgb8";
  convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

  while (rclcpp::ok() && is_running_) {
    nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
    if (MV_OK == nRet) {
      image_msg_.header.stamp = this->now();
      // 转换
      image_msg_.height = out_frame.stFrameInfo.nHeight;
      image_msg_.width = out_frame.stFrameInfo.nWidth;
      image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
      image_msg_.data.resize(image_msg_.step * image_msg_.height);

      convert_param_.pDstBuffer = image_msg_.data.data();
      convert_param_.nDstBufferSize = image_msg_.data.size();
      convert_param_.pSrcData = out_frame.pBufAddr;
      convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
      convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;
      convert_param_.nWidth = out_frame.stFrameInfo.nWidth;
      convert_param_.nHeight = out_frame.stFrameInfo.nHeight;

      MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

      camera_info_msg_.header = image_msg_.header;
      camera_pub_.publish(image_msg_, camera_info_msg_);

      MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
      fail_count_ = 0;
    } else {
      RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void HikCameraNode::localFileLoop() {
  cv::VideoCapture cap;
  cv::Mat frame;
  bool is_image = (input_type_ == "image");
  if (is_image) {
    frame = cv::imread(file_path_);
    if (frame.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Read image failed: %s",
                   file_path_.c_str());
      return;
    }
  }

  if (!is_image) {
    cap.open(file_path_);
    if (!cap.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s",
                   file_path_.c_str());
      return;
    }
    // 获取视频原生帧率，如果没有则默认 30
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 0)
      fps = 30.0;
    RCLCPP_INFO(this->get_logger(), "Video FPS: %.2f", fps);
  }

  rclcpp::WallRate loop_rate(30);

  while (rclcpp::ok() && is_running_) {
    if (!is_image) {
      cap >> frame;
      if (frame.empty()) {
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      }
    }

    cv::Mat rgb_frame;
    cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", rgb_frame)
                   .toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_optical_frame";

    camera_info_msg_.header = msg->header;
    camera_pub_.publish(*msg, camera_info_msg_);
    loop_rate.sleep();
  }
}

void HikCameraNode::declareParameters() {
  // 仅在 SDK 成功初始化后调用
  MVCC_FLOATVALUE f_value;
  MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
  this->declare_parameter("exposure_time", (double)f_value.fCurValue);

  MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
  this->declare_parameter("gain", (double)f_value.fCurValue);
}

rcl_interfaces::msg::SetParametersResult HikCameraNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  // 只有在相机模式下才去设置 SDK 参数
  if (input_type_ != "camera" || !camera_handle_)
    return result;

  for (const auto &param : parameters) {
    if (param.get_name() == "exposure_time") {
      MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
    } else if (param.get_name() == "gain") {
      MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
    }
  }
  return result;
}

HikCameraNode::~HikCameraNode() {
  is_running_ = false;
  if (capture_thread_.joinable())
    capture_thread_.join();
  if (camera_handle_) {
    MV_CC_StopGrabbing(camera_handle_);
    MV_CC_CloseDevice(camera_handle_);
    MV_CC_DestroyHandle(&camera_handle_);
  }
}
} // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)