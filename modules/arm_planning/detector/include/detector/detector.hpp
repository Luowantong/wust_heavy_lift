#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace exchange_slot {

struct Pose {
  cv::Vec3d rvec; // 旋转向量
  cv::Vec3d tvec; // 平移向量
  cv::Vec4d quat; // 四元数 w,x,y,z
};

class ExchangeSlotDetector {
public:
  // 构造函数不再负责加载 YAML，由 Node 层统一管理
  ExchangeSlotDetector();

  // 设置参数接口，供 Node 层调用 declare_parameter 后注入
  void setParams(int binary_thresh, double square_size, bool debug);
  void setCameraParams(const cv::Mat &camera_matrix,
                       const cv::Mat &dist_coeffs);

  // 主检测函数
  bool detect(const cv::Mat &input);

  // 绘制检测结果
  void drawResults(cv::Mat &image);

  // 获取当前解算结果
  Pose getPose() const { return last_pose_; }

  // 算法内部步骤
  cv::Mat preprocessImage(const cv::Mat &input);
  std::vector<std::vector<cv::Point>>
  findContoursAndFilter(const cv::Mat &binary);
  std::vector<cv::Point2f>
  findCornerPoints(const std::vector<std::vector<cv::Point>> &contours);
  std::vector<cv::Point2f> findSmallSquares(const cv::Mat &binary);

  // 核心逻辑：排序与解算
  std::vector<cv::Point2f>
  getSortedCorners(const std::vector<cv::Point2f> &corners,
                   const std::vector<cv::Point2f> &small_squares,
                   const std::vector<std::vector<cv::Point>> &contours);

  void filterCorners(std::vector<cv::Point2f> &current);
  bool solvePnP_IPPE(const std::vector<cv::Point2f> &corners, Pose &pose);
  bool validatePose(const std::vector<cv::Point2f> &corners, const Pose &pose);

  // 获取参数接口，供外部验证使用
  double getSquareSize() const { return square_size_; }
  cv::Mat getCameraMatrix() const { return camera_matrix_; }
  cv::Mat getDistCoeffs() const { return dist_coeffs_; }
  Pose getLastPose() const { return last_pose_; }

private:
  // 状态与参数
  int binary_thresh_ = 100;
  double square_size_ = 0.25; // 兑换槽物理尺寸(正方形边长)
  bool debug_ = false;

  std::vector<cv::Point2f> corner_points_; // 存储当前帧最终角点
  std::vector<cv::Point2f> last_corners_;  // 用于滤波
  Pose last_pose_;
  bool has_last_pose_ = false;

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
};

} // namespace exchange_slot