#include "wust_mujoco_plugins/grasp_plugins.hpp"
#include <cmath>
#include <optional>
#include <pluginlib/class_list_macros.hpp>
#include <string>

namespace wust_mujoco_plugins {

namespace {
std::optional<double> paramOrNull(const rclcpp::Node::SharedPtr& node,
                                  const std::string& name) {
  if (!node->has_parameter(name)) return std::nullopt;
  return node->get_parameter(name).as_double();
}

// URDF 约定：R = Rz(yaw) * Ry(pitch) * Rx(roll)，返回 MuJoCo 的 (w, x, y, z)
void rpyToQuat(double roll, double pitch, double yaw, double q[4]) {
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  q[0] = cr * cp * cy + sr * sp * sy;  // w
  q[1] = sr * cp * cy - cr * sp * sy;  // x
  q[2] = cr * sp * cy + sr * cp * sy;  // y
  q[3] = cr * cp * sy - sr * sp * cy;  // z
}
}  // namespace

bool GraspPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model_c,
                       mjData* data) {
  const mjModel* model = model_c;

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

  // ------------------------------------------------------------------
  // workcell：用 yaml 里的目标位姿覆盖 MJCF 中的 slot / hollow / rest weld
  // 位置和姿态都支持（roll/pitch/yaw，URDF rpy 约定，单位弧度）
  // ------------------------------------------------------------------
  const bool has_pose =
      node->has_parameter("workcell.object.x") &&
      node->has_parameter("workcell.object.y") &&
      node->has_parameter("workcell.object.z") &&
      node->has_parameter("workcell.slot.x") &&
      node->has_parameter("workcell.slot.y") &&
      node->has_parameter("workcell.slot.z");

  auto get = [&](const char* name, double fallback) {
    const auto p = paramOrNull(node, name);
    return p ? *p : fallback;
  };

  if (!has_pose) {
    RCLCPP_WARN(node->get_logger(),
                "workcell.* 参数不可见，沿用 MJCF 里的 slot/hollow/rest 位姿");
  } else {
    // 位置必填，姿态缺省为 0（兼容只有 xyz 的旧 yaml）
    const double obj[3] = {get("workcell.object.x", 0.0),
                           get("workcell.object.y", 0.0),
                           get("workcell.object.z", 0.0)};
    const double slot[3] = {get("workcell.slot.x", 0.0),
                            get("workcell.slot.y", 0.0),
                            get("workcell.slot.z", 0.0)};
    const double obj_rpy[3] = {get("workcell.object.roll", 0.0),
                               get("workcell.object.pitch", 0.0),
                               get("workcell.object.yaw", 0.0)};
    const double slot_rpy[3] = {get("workcell.slot.roll", 0.0),
                                get("workcell.slot.pitch", 0.0),
                                get("workcell.slot.yaw", 0.0)};
    double q_obj[4], q_slot[4];
    rpyToQuat(obj_rpy[0], obj_rpy[1], obj_rpy[2], q_obj);
    rpyToQuat(slot_rpy[0], slot_rpy[1], slot_rpy[2], q_slot);

    // 注意：base class 给的是 const mjModel*，但仿真内部持有的是非 const 模型。
    // 如果 mujoco_ros2_control 未来改成加载后只读、或 reset 时重新加载模型，
    // 这里需要换成官方提供的非 const 入口。
    mjModel* m = const_cast<mjModel*>(model_c);

    const int slot_id = mj_name2id(model, mjOBJ_BODY, "slot");
    const int hollow_id = mj_name2id(model, mjOBJ_BODY, "hollow_cylinder");
    if (slot_id < 0 || hollow_id < 0) {
      RCLCPP_WARN(node->get_logger(),
                  "workcell: MJCF 里找不到 slot / hollow_cylinder body");
    } else {
      // slot 本体（位置 + 姿态）
      for (int i = 0; i < 3; ++i) m->body_pos[3 * slot_id + i] = slot[i];
      for (int i = 0; i < 4; ++i) m->body_quat[4 * slot_id + i] = q_slot[i];

      // hollow 本体（位置 + 姿态）
      for (int i = 0; i < 3; ++i) m->body_pos[3 * hollow_id + i] = obj[i];
      for (int i = 0; i < 4; ++i) m->body_quat[4 * hollow_id + i] = q_obj[i];

      // hollow 的 freejoint 初始位姿（reset_world 会恢复到 qpos0）
      const int jid = m->body_jntadr[hollow_id];
      if (jid >= 0 && m->jnt_type[jid] == mjJNT_FREE) {
        const int qa = m->jnt_qposadr[jid];
        for (int i = 0; i < 3; ++i) m->qpos0[qa + i] = obj[i];
        for (int i = 0; i < 4; ++i) m->qpos0[qa + 3 + i] = q_obj[i];
        if (data) {
          for (int i = 0; i < 3; ++i) data->qpos[qa + i] = obj[i];
          for (int i = 0; i < 4; ++i) data->qpos[qa + 3 + i] = q_obj[i];
        }
      } else {
        RCLCPP_WARN(node->get_logger(),
                    "workcell: hollow_cylinder 没有 freejoint，跳过 qpos0");
      }

      // rest weld relpose = world_T_hollow
      // eq_data 每个 equality 占 mjNEQDATA 个 double：
      // [0..2] anchor, [3..5] relpose 位置, [6..9] 四元数(wxyz), [10] torque scale
      const int e = rest_eq_id_ * mjNEQDATA;
      for (int i = 0; i < 3; ++i) m->eq_data[e + 3 + i] = obj[i];
      for (int i = 0; i < 4; ++i) m->eq_data[e + 6 + i] = q_obj[i];

      RCLCPP_INFO(
          node->get_logger(),
          "workcell from yaml: object=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f) "
          "slot=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
          obj[0], obj[1], obj[2], obj_rpy[0], obj_rpy[1], obj_rpy[2],
          slot[0], slot[1], slot[2], slot_rpy[0], slot_rpy[1], slot_rpy[2]);
    }
  }

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