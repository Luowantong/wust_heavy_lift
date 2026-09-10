#include "detector/detector_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // 使用 NodeOptions 支持参数注入
  rclcpp::NodeOptions options;
  auto node =
      std::make_shared<exchange_slot::ExchangeSlotDetectorNode>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}