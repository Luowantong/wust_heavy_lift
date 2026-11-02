#pragma once

#include "mtc/common/task_node_base.hpp"

namespace mtc {

class MTCPickNode : public TaskNodeBase {
public:
  explicit MTCPickNode(const rclcpp::NodeOptions &options);
protected:
  moveit::task_constructor::Task createTask() override;
  void setupPlanningScene(const geometry_msgs::msg::PoseStamped &target) override;

private:
  double approach_height_{0.10};
  double lift_height_{0.10};
  int yaw_candidates_{3};
  double k_retreat_{1.0};
  bool computeGraspTCP(tf2::Transform &base_T_tcp);
};

}  // namespace mtc
