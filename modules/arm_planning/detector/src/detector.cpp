#include "detector/detector.hpp"

namespace exchange_slot {

ExchangeSlotDetector::ExchangeSlotDetector() {
  corner_points_.clear();
  last_corners_.clear();
  has_last_pose_ = false;
  // 初始化默认相机参数以防万一
  camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
  dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
}

void ExchangeSlotDetector::setParams(int binary_thresh, double square_size,
                                     bool debug) {
  binary_thresh_ = binary_thresh;
  square_size_ = square_size;
  debug_ = debug;
}

void ExchangeSlotDetector::setCameraParams(const cv::Mat &camera_matrix,
                                           const cv::Mat &dist_coeffs) {
  camera_matrix_ = camera_matrix.clone();
  dist_coeffs_ = dist_coeffs.clone();
}

cv::Mat ExchangeSlotDetector::preprocessImage(const cv::Mat &input) {
  cv::Mat bgr[3];
  cv::split(input, bgr);
  cv::Mat combined;
  cv::add(bgr[0], bgr[2], combined); // 红蓝叠加
  cv::GaussianBlur(combined, combined, cv::Size(5, 5), 0);

  cv::Mat binary;
  cv::threshold(combined, binary, binary_thresh_, 255, cv::THRESH_BINARY);
  return binary;
}

std::vector<std::vector<cv::Point>>
ExchangeSlotDetector::findContoursAndFilter(const cv::Mat &binary) {
  std::vector<std::vector<cv::Point>> all_contours;
  cv::findContours(binary, all_contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  struct Candidate {
    std::vector<cv::Point> poly;
    double area;
  };
  std::vector<Candidate> temp_candidates;

  for (const auto &contour : all_contours) {
    double area = cv::contourArea(contour);
    if (area < 250 || area > 100000)
      continue;

    // 1. 凸度过滤 (Solidity = 面积 / 凸包面积)
    // L 形灯条因为弯折，Solidity 通常在 0.4 ~ 0.7 之间
    // 斜向长条形干扰物 Solidity 通常 > 0.85
    std::vector<cv::Point> hull;
    cv::convexHull(contour, hull);
    double hull_area = cv::contourArea(hull);
    double solidity = (hull_area > 0) ? (area / hull_area) : 0;

    if (solidity > 0.85)
      continue; // 滤除掉实心的长条形

    // 2. 比例过滤
    cv::Rect rect = cv::boundingRect(contour);
    float ratio = static_cast<float>(rect.width) / rect.height;
    if (ratio > 4.5 || 1.0f / ratio > 4.5f)
      continue;

    // 3. 多边形拟合顶点数过滤
    std::vector<cv::Point> poly;
    cv::approxPolyDP(contour, poly, 3, true);

    // 针对 L 形，通常顶点数在 6 到 12 之间 (4点拟合太粗糙)
    if (poly.size() >= 5 && poly.size() < 14) {
      temp_candidates.push_back({poly, area});
    }
  }

  // 4. 面积一致性选择：如果候选者 > 4，选面积最接近的 4 个
  if (temp_candidates.size() > 4) {
    std::sort(
        temp_candidates.begin(), temp_candidates.end(),
        [](const Candidate &a, const Candidate &b) { return a.area > b.area; });
    // 取面积最大的前 4 个，前提是它们面积不能相差太悬殊
    if (temp_candidates[0].area / temp_candidates[3].area < 5.0) {
      temp_candidates.resize(4);
    } else {
      return {}; // 面积分布太乱，识别不可信
    }
  }

  std::vector<std::vector<cv::Point>> final_polys;
  for (const auto &c : temp_candidates)
    final_polys.push_back(c.poly);
  return final_polys;
}

std::vector<cv::Point2f> ExchangeSlotDetector::findCornerPoints(
    const std::vector<std::vector<cv::Point>> &contours) {
  std::vector<cv::Point2f> circle_centers;
  for (const auto &contour : contours) {
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);
    circle_centers.push_back(center);
  }

  cv::Point2f midpoint(0, 0);
  for (const auto &p : circle_centers)
    midpoint += p;
  midpoint *= 0.25f;

  std::vector<cv::Point2f> corners;
  for (const auto &contour : contours) {
    std::vector<cv::Point2f> triangle(3);
    cv::minEnclosingTriangle(contour, triangle);

    double max_angle = 0.0;
    int max_angle_idx = 0;
    for (int i = 0; i < 3; i++) {
      cv::Point2f a = triangle[i], b = triangle[(i + 1) % 3],
                  c = triangle[(i + 2) % 3];
      cv::Point2f ab = b - a, ac = c - a;
      double angle =
          acos(ab.dot(ac) / (cv::norm(ab) * cv::norm(ac))) * 180.0 / CV_PI;
      if (angle > max_angle) {
        max_angle = angle;
        max_angle_idx = i;
      }
    }

    if (max_angle > 130.0)
      corners.push_back(triangle[max_angle_idx]);
    else {
      auto it = std::max_element(
          triangle.begin(), triangle.end(), [&](cv::Point2f a, cv::Point2f b) {
            return cv::norm(a - midpoint) < cv::norm(b - midpoint);
          });
      corners.push_back(*it);
    }
  }
  return corners;
}

std::vector<cv::Point2f>
ExchangeSlotDetector::findSmallSquares(const cv::Mat &binary) {
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);
  std::vector<cv::Point2f> centers;
  for (const auto &c : contours) {
    double area = cv::contourArea(c);
    if (area < 50 || area > 200)
      continue;
    cv::RotatedRect r = cv::minAreaRect(c);
    float ratio =
        std::max(r.size.width / r.size.height, r.size.height / r.size.width);
    if (r.size.area() < 1.2 * area && ratio < 2.0)
      centers.push_back(r.center);
  }
  return centers;
}

// 角点排序
std::vector<cv::Point2f> ExchangeSlotDetector::getSortedCorners(
    const std::vector<cv::Point2f> &corners,
    const std::vector<cv::Point2f> &small_squares,
    const std::vector<std::vector<cv::Point>> &contours) {
  if (corners.size() != 4)
    return {};

  // 1. 计算几何中心
  cv::Point2f center(0, 0);
  for (const auto &p : corners)
    center += p;
  center *= 0.25f;

  // 2. 极角排序（建立稳定的环）
  struct IndexedPoint {
    int original_idx;
    float angle;
    cv::Point2f pt;
  };

  std::vector<IndexedPoint> pts;
  pts.reserve(4);

  for (int i = 0; i < 4; i++) {
    float ang = std::atan2(corners[i].y - center.y, corners[i].x - center.x);
    pts.push_back({i, ang, corners[i]});
  }

  std::sort(pts.begin(), pts.end(),
            [](const IndexedPoint &a, const IndexedPoint &b) {
              return a.angle < b.angle;
            });

  // 3. 寻找“特殊角点”（start_pos）
  int start_pos = -1;

  // --- 策略 A：小方块优先 ---
  if (!small_squares.empty()) {
    cv::Point2f avg_sq(0, 0);
    for (const auto &p : small_squares)
      avg_sq += p;
    avg_sq *= (1.0f / small_squares.size());

    double min_d = 1e9;
    for (int i = 0; i < 4; i++) {
      double d = cv::norm(pts[i].pt - avg_sq);
      if (d < min_d) {
        min_d = d;
        start_pos = i;
      }
    }
    // std::cout << "[Sorting Log] Strategy A (Small Squares) Success." <<
    // std::endl;
  }
  // --- 策略 B：上帧记忆  ---
  /*
  else if (has_last_pose_ && !last_corners_.empty()) {
      double min_d = 1e9;
      for (int i = 0; i < 4; i++) {
          double d = cv::norm(pts[i].pt - last_corners_[0]);
          if (d < min_d) {
              min_d = d;
              start_pos = i;
          }
      }
      if (min_d > 100.0) start_pos = -1;
      if (start_pos != -1)
          std::cout << "[Sorting Log] Strategy B (Memory) Success." <<
  std::endl;
  }
  */

  // --- 策略 C：面积最小兜底 ---
  if (start_pos == -1) {
    double min_area = 1e9;
    for (int i = 0; i < 4; i++) {
      double area = cv::contourArea(contours[pts[i].original_idx]);
      if (area < min_area) {
        min_area = area;
        start_pos = i;
      }
    }
    // std::cout << "[Sorting Log] Strategy C (Min Area) Success. Target Area: "
    //   << min_area << std::endl;
  }

  //    将“特殊角点”映射到 object_points 的 0/1/2/3（左上，右上，右下，左下）
  cv::Point2f sp = pts[start_pos].pt;
  float dx = sp.x - center.x;
  float dy = sp.y - center.y;

  // object_points:
  // 0: 左上, 1: 右上, 2: 右下, 3: 左下
  int obj_idx;
  if (dx < 0 && dy < 0) {
    obj_idx = 0; // 左上
  } else if (dx > 0 && dy < 0) {
    obj_idx = 1; // 右上
  } else if (dx > 0 && dy > 0) {
    obj_idx = 2; // 右下
  } else {
    obj_idx = 3; // 左下
  }

  // 环状重排，使 corners[i] 语义 == object_points[i]
  std::vector<cv::Point2f> sorted(4);

  for (int i = 0; i < 4; i++) {
    int src = (start_pos + i) % 4;
    int dst = (obj_idx + i) % 4;
    sorted[dst] = pts[src].pt;
  }

  return sorted;
}

void ExchangeSlotDetector::filterCorners(std::vector<cv::Point2f> &current) {
  if (last_corners_.empty()) {
    last_corners_ = current;
    return;
  }
  for (size_t i = 0; i < 4; i++)
    current[i] = last_corners_[i] * 0.9f + current[i] * 0.1f;
  last_corners_ = current;
}

// PNP 解算：SOLVEPNP_IPPE_SQUARE，输入顺序为左上、右上、右下、左下
bool ExchangeSlotDetector::solvePnP_IPPE(
    const std::vector<cv::Point2f> &corners, Pose &pose) {
  double s = square_size_ / 2.0;
  std::vector<cv::Point3f> object_points = {
      {(float)-s, (float)s, 0}, // 0：左上
      {(float)s, (float)s, 0},  // 1：右上
      {(float)s, (float)-s, 0}, // 2：右下
      {(float)-s, (float)-s, 0} // 3：左下
  };

  cv::Mat rvec, tvec;
  // 注意：SOLVEPNP_IPPE_SQUARE 对点序非常敏感，必须保证是连续的环
  bool ok = cv::solvePnP(object_points, corners, camera_matrix_, dist_coeffs_,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

  if (ok) {
    pose.rvec = rvec;
    pose.tvec = tvec;
  }
  return ok;
}

bool ExchangeSlotDetector::validatePose(const std::vector<cv::Point2f> &corners,
                                        const Pose &pose) {
  // 1. 凸性校验：确保四个点没有交叉成“沙漏型”
  if (!cv::isContourConvex(corners)) {
    return false;
  }

  // 2. 几何角度校验
  for (int i = 0; i < 4; i++) {
    cv::Point2f a = corners[i], b = corners[(i + 1) % 4],
                c = corners[(i + 2) % 4];
    cv::Point2f ab = b - a, ac = c - a;
    double dot_prod = ab.dot(ac);
    double cos_val = dot_prod / (cv::norm(ab) * cv::norm(ac));

    cos_val = std::max(-1.0, std::min(1.0, cos_val));
    double angle = acos(cos_val) * 180.0 / CV_PI;

    if (angle < 30.0 || angle > 150.0)
      return false;
  }

  // 3. 重投影数值验证 (暂时注释掉阈值过滤，仅保留状态记录)

  // double s = square_size_ / 2.0;
  // std::vector<cv::Point3f> object_points = {
  //     {(float)s, (float)s, 0}, {(float)s, -(float)s, 0},
  //     {-(float)s, -(float)s, 0}, {-(float)s, (float)s, 0}
  // };
  // std::vector<cv::Point2f> projected;
  // cv::projectPoints(object_points, pose.rvec, pose.tvec, camera_matrix_,
  // dist_coeffs_, projected);

  // double total_err = 0;
  // for (int i = 0; i < 4; i++) total_err += cv::norm(projected[i] -
  // corners[i]); if ((total_err / 4.0) > 20.0) return false;

  // 4. 成功后记录状态（用于无方块时的历史补全）
  last_corners_ = corners;
  has_last_pose_ = true;
  return true;
}

bool ExchangeSlotDetector::detect(const cv::Mat &input) {
  cv::Mat binary = preprocessImage(input);
  auto contours = findContoursAndFilter(binary);
  if (contours.size() != 4)
    return false;

  auto corners = findCornerPoints(contours);
  auto small_sqs = findSmallSquares(binary);
  auto sorted = getSortedCorners(corners, small_sqs, contours);
  if (sorted.size() != 4)
    return false;

  filterCorners(sorted);
  Pose current_pose;
  if (!solvePnP_IPPE(sorted, current_pose))
    return false;
  if (!validatePose(sorted, current_pose))
    return false;

  last_pose_ = current_pose;
  corner_points_ = sorted;
  return true;
}

void ExchangeSlotDetector::drawResults(cv::Mat &image) {
  if (corner_points_.empty())
    return;
  for (int i = 0; i < 4; i++) {
    cv::circle(image, corner_points_[i], 8, cv::Scalar(0, 255, 0), -1);
    cv::line(image, corner_points_[i], corner_points_[(i + 1) % 4],
             cv::Scalar(0, 255, 255), 4);
  }
}

} // namespace exchange_slot