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
  // workcell：用 yaml 里的 object 位姿覆盖 MJCF 中的 hollow / rest weld
  // 位置和姿态都支持（roll/pitch/yaw，URDF rpy 约定，单位弧度）
  // slot 已从 yaml/MJCF 移除，这里不再处理
  // ------------------------------------------------------------------
  const bool has_pose = node->has_parameter("workcell.object.x") &&
                        node->has_parameter("workcell.object.y") &&
                        node->has_parameter("workcell.object.z");

  auto get = [&](const char* name, double fallback) {
    const auto p = paramOrNull(node, name);
    return p ? *p : fallback;
  };

  if (!has_pose) {
    RCLCPP_WARN(node->get_logger(),
                "workcell.object.* 参数不可见，沿用 MJCF 里的 hollow/rest 位姿");
  } else {
    const double obj[3] = {get("workcell.object.x", 0.0),
                           get("workcell.object.y", 0.0),
                           get("workcell.object.z", 0.0)};
    const double obj_rpy[3] = {get("workcell.object.roll", 0.0),
                               get("workcell.object.pitch", 0.0),
                               get("workcell.object.yaw", 0.0)};
    double q_obj[4];
    rpyToQuat(obj_rpy[0], obj_rpy[1], obj_rpy[2], q_obj);

    mjModel* m = const_cast<mjModel*>(model_c);
    const int hollow_id = mj_name2id(model, mjOBJ_BODY, "hollow_cylinder");
    if (hollow_id < 0) {
      RCLCPP_WARN(node->get_logger(),
                  "workcell: MJCF 里找不到 hollow_cylinder body");
    } else {
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
      // eq_data: [0..2] anchor, [3..5] relpose 位置, [6..9] 四元数(wxyz), [10] torque scale
      const int e = rest_eq_id_ * mjNEQDATA;
      for (int i = 0; i < 3; ++i) m->eq_data[e + 3 + i] = obj[i];
      for (int i = 0; i < 4; ++i) m->eq_data[e + 6 + i] = q_obj[i];

      RCLCPP_INFO(node->get_logger(),
                  "workcell from yaml: object=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
                  obj[0], obj[1], obj[2], obj_rpy[0], obj_rpy[1], obj_rpy[2]);
    }
  }

  // ------------------------------------------------------------------
  // 松手时要把 rest 焊点重锚到单元当前位姿，这里先记下重锚需要的东西：
  // hollow 自由关节的 qpos 地址 + rest weld 的初始 relpose（取件位）
  // ------------------------------------------------------------------
  model_ = model;
  const int hollow_id = mj_name2id(model, mjOBJ_BODY, "hollow_cylinder");
  if (hollow_id >= 0) {
    hollow_qpos_adr_ = model->jnt_qposadr[model->body_jntadr[hollow_id]];
  } else {
    RCLCPP_WARN(node->get_logger(),
                "GraspPlugin: 找不到 hollow_cylinder，松手重锚不可用");
  }
  const int re = rest_eq_id_ * mjNEQDATA;
  for (int i = 0; i < 7; ++i) rest_relpose0_[i] = model->eq_data[re + 3 + i];

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
  const bool attached = attached_.load();

  // 抓取 → 松开的下降沿：把 rest 焊点锚到单元当前位姿。
  // 否则 rest 还指着取件位，单元一松手就被拽回工作区。
  if (attached_prev_ && !attached && model_ && hollow_qpos_adr_ >= 0) {
    mjtNum* e = model_->eq_data + rest_eq_id_ * mjNEQDATA;
    const mjtNum* q = data->qpos + hollow_qpos_adr_;
    for (int i = 0; i < 3; ++i) e[3 + i] = q[i];      // relpose 位置
    for (int i = 0; i < 4; ++i) e[6 + i] = q[3 + i];  // relpose 四元数
  }
  attached_prev_ = attached;

  if (attached) {
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
  attached_prev_ = false;
  // reset_world 把单元放回 qpos0（取件位），rest 焊点也要回到初始 relpose，
  // 否则两者打架，单元下一帧会被拉回站体
  if (!model_) return;
  mjtNum* e = model_->eq_data + rest_eq_id_ * mjNEQDATA;
  for (int i = 0; i < 7; ++i) e[3 + i] = rest_relpose0_[i];
}

void GraspPlugin::cleanup() {
  sub_.reset();
}

} // namespace wust_mujoco_plugins

PLUGINLIB_EXPORT_CLASS(wust_mujoco_plugins::GraspPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)