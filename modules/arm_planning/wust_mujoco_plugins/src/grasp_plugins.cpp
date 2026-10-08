#include "wust_mujoco_plugins/grasp_plugins.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace wust_mujoco_plugins {

bool GraspPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData*) {
  // 找到两个 weld 约束的 ID
  grasp_eq_id_ = mj_name2id(model, mjOBJ_EQUALITY, "grasp");
  rest_eq_id_ = mj_name2id(model, mjOBJ_EQUALITY, "rest");

  if (grasp_eq_id_ < 0) {
    RCLCPP_ERROR(node->get_logger(), "MJCF 里找不到 'grasp' 约束");
    return false;
  }
  if (rest_eq_id_ < 0) {
    RCLCPP_ERROR(node->get_logger(), "MJCF 里找不到 'rest' 约束");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "GraspPlugin: grasp_eq_id=%d, rest_eq_id=%d",
              grasp_eq_id_, rest_eq_id_);

  // 订阅 move_group 的附着事件
  sub_ = node->create_subscription<moveit_msgs::msg::AttachedCollisionObject>(
      "/attached_collision_object", rclcpp::QoS(10),
      [this](moveit_msgs::msg::AttachedCollisionObject::ConstSharedPtr msg) {
        if (msg->object.id != "hollow_cylinder") return;
        // ADD + 非空 link_name = 附着；REMOVE 或空 link_name = 分离
        bool is_attached =
            (msg->object.operation == moveit_msgs::msg::CollisionObject::ADD) &&
            !msg->link_name.empty();
        attached_.store(is_attached);
        RCLCPP_INFO(rclcpp::get_logger("GraspPlugin"),
                    "hollow_cylinder attached=%s", is_attached ? "true" : "false");
      });

  return true;
}

void GraspPlugin::pre_step(mjData* data) {
  if (attached_.load()) {
    // 抓取时：关 rest，开 grasp
    data->eq_active[rest_eq_id_] = 0;
    data->eq_active[grasp_eq_id_] = 1;
  } else {
    // 未抓取时：开 rest，关 grasp
    data->eq_active[rest_eq_id_] = 1;
    data->eq_active[grasp_eq_id_] = 0;
  }
}

void GraspPlugin::on_reset(mjData*) {
  attached_.store(false);
}

void GraspPlugin::cleanup() {
  sub_.reset();
}

} // namespace wust_mujoco_plugins

PLUGINLIB_EXPORT_CLASS(wust_mujoco_plugins::GraspPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)