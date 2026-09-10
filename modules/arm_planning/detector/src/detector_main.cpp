#include "detector/detector.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @brief 使用类成员函数复现 detect 逻辑，并插入调试信息
 */
bool debugDetect(cv::Mat &input,
                 exchange_slot::ExchangeSlotDetector &detector) {
  std::string w1 = "1.Raw_Contours", w2 = "2.Bounding_Rects",
              w3 = "3.Poly_Approx", w4 = "4.Final_Candidates",
              w5 = "5.Sorted_Result";

  // --- 1. 预处理 ---
  cv::Mat binary = detector.preprocessImage(input);

  // --- 2. 轮廓提取与过滤 ---
  std::vector<std::vector<cv::Point>> all_contours;
  cv::findContours(binary, all_contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

  // 窗口 1: 原始轮廓
  cv::Mat raw_cont_img = input.clone();
  cv::drawContours(raw_cont_img, all_contours, -1, cv::Scalar(0, 255, 255), 2);
  cv::namedWindow(w1, cv::WINDOW_NORMAL);
  cv::resizeWindow(w1, 800, 600);
  cv::imshow(w1, raw_cont_img);

  cv::Mat rect_img = input.clone();
  cv::Mat poly_img = input.clone();
  std::vector<std::vector<cv::Point>> candidates =
      detector.findContoursAndFilter(binary);

  // 窗口 2 & 3: 辅助观察
  for (const auto &contour : all_contours) {
    cv::Rect rect = cv::boundingRect(contour);
    cv::rectangle(rect_img, rect, cv::Scalar(255, 255, 0), 2);
    std::vector<cv::Point> poly;
    cv::approxPolyDP(contour, poly, 3, true);
    cv::putText(poly_img, std::to_string(poly.size()), poly[0],
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
  }
  cv::namedWindow(w2, cv::WINDOW_NORMAL);
  cv::resizeWindow(w2, 800, 600);
  cv::imshow(w2, rect_img);
  cv::namedWindow(w3, cv::WINDOW_NORMAL);
  cv::resizeWindow(w3, 800, 600);
  cv::imshow(w3, poly_img);

  // 窗口 4: 最终选出的候选者
  if (candidates.size() == 4) {
    cv::Mat final_cand_img = input.clone();
    cv::drawContours(final_cand_img, candidates, -1, cv::Scalar(0, 255, 0), 4);
    cv::namedWindow(w4, cv::WINDOW_NORMAL);
    cv::resizeWindow(w4, 800, 600);
    cv::imshow(w4, final_cand_img);
  } else {
    std::cout << "[Step 2 Failed] Found " << candidates.size()
              << " candidates. Expected 4." << std::endl;
    cv::destroyWindow(w4);
    cv::destroyWindow(w5);
    return false;
  }

  // --- 3 & 4. 角点与排序 ---
  auto corners = detector.findCornerPoints(candidates);
  std::cout << "[Log] Step 3: findCornerPoints found " << corners.size()
            << " corner candidates." << std::endl;

  auto small_sqs = detector.findSmallSquares(binary);
  std::cout << "[Log] Step 3.5: findSmallSquares found " << small_sqs.size()
            << " small squares." << std::endl;
  auto sorted = detector.getSortedCorners(corners, small_sqs, candidates);

  if (sorted.size() != 4) {
    std::cout << "[Step 4 Failed] Corner sorting failed. Found points: "
              << sorted.size() << std::endl;
    cv::destroyWindow(w5);
    return false;
  }

  // 窗口 5: 排序结果
  cv::Mat sorted_img = input.clone();
  for (int i = 0; i < 4; i++) {
    cv::line(sorted_img, sorted[i], sorted[(i + 1) % 4], cv::Scalar(0, 255, 0),
             3);
    cv::putText(sorted_img, std::to_string(i), sorted[i],
                cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0, 0, 255), 3);
  }
  cv::namedWindow(w5, cv::WINDOW_NORMAL);
  cv::resizeWindow(w5, 800, 600);
  cv::imshow(w5, sorted_img);

  // --- 5 & 6. PnP ---
  exchange_slot::Pose pose;
  if (detector.solvePnP_IPPE(sorted, pose)) {
    if (detector.validatePose(sorted, pose)) {
      std::cout << "[Success] Distance: " << pose.tvec[2] << " mm" << std::endl;
      return true;
    } else {
      std::cout << "[Step 6 Failed] Pose validation failed (Non-convex or bad "
                   "angles)."
                << std::endl;
    }
  } else {
    std::cout << "[Step 5 Failed] PnP Solver failed." << std::endl;
  }
  return false;
}
int main(int argc, char **argv) {
  std::string config_path = "src/detector/config/exchange_slot_detector.yaml";
  if (argc > 1)
    config_path = argv[1];

  YAML::Node config = YAML::LoadFile(config_path);
  auto params = config["exchange_slot_detector"]["ros__parameters"];

  exchange_slot::ExchangeSlotDetector detector;
  detector.setParams(params["binary_thresh"].as<int>(),
                     params["square_size"].as<double>(), true);

  std::vector<double> k_vec = params["camera_matrix"].as<std::vector<double>>();
  std::vector<double> d_vec = params["dist_coeffs"].as<std::vector<double>>();
  detector.setCameraParams(cv::Mat(k_vec).reshape(1, 3),
                           cv::Mat(d_vec).reshape(1, d_vec.size()));

  std::string path = params["local_path"].as<std::string>();

  // --- 修改部分：使用 VideoCapture ---
  cv::VideoCapture cap(path);
  if (!cap.isOpened()) {
    std::cerr << "Error: Could not open image or video at: " << path
              << std::endl;
    return -1;
  }

  cv::Mat frame;
  std::cout << "--- Processing: " << path << " ---" << std::endl;
  std::cout << "Press 'q' to exit, 'p' to pause." << std::endl;

  while (true) {
    if (!cap.read(frame)) {
      // 如果是视频，读到末尾自动重头开始；如果是图片，读取失败即退出
      if (cap.get(cv::CAP_PROP_FRAME_COUNT) > 1) {
        cap.set(cv::CAP_PROP_POS_FRAMES, 0);
        continue;
      } else {
        break;
      }
    }

    // 调用带调试信息的检测
    bool result = debugDetect(frame, detector);

    // 如果是图片（总帧数为1），则无限等待
    if (cap.get(cv::CAP_PROP_FRAME_COUNT) <= 1) {
      std::cout << "Image detected. Final Result: "
                << (result ? "SUCCESS" : "FAILED") << std::endl;
      cv::waitKey(0);
      break;
    }

    // 如果是视频，控制播放速度
    char key = (char)cv::waitKey(30);
    if (key == 'q')
      break;
    if (key == 'p')
      cv::waitKey(0); // 暂停
  }

  return 0;
}