#include "wust_mujoco_plugins/grasp_plugins.hpp"
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
  // workcell：用 yaml 里的目标坐标覆盖 MJCF 中的 slot / hollow / rest weld
  // ------------------------------------------------------------------
  const char* names[6] = {"workcell.object.x", "workcell.object.y",
                          "workcell.object.z", "workcell.slot.x",
                          "workcell.slot.y",   "workcell.slot.z"};
  double v[6] = {0};
  bool has_all = true;
  for (int i = 0; i < 6; ++i) {
    const auto p = paramOrNull(node, names[i]);
    if (!p) {
      has_all = false;
      break;
    }
    v[i] = *p;
  }

  if (!has_all) {
    RCLCPP_WARN(node->get_logger(),
                "workcell.* 参数不可见，沿用 MJCF 里的 slot/hollow/rest 位置");
  } else {
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
      // slot 本体
      m->body_pos[3 * slot_id + 0] = v[3];
      m->body_pos[3 * slot_id + 1] = v[4];
      m->body_pos[3 * slot_id + 2] = v[5];

      // hollow 本体
      m->body_pos[3 * hollow_id + 0] = v[0];
      m->body_pos[3 * hollow_id + 1] = v[1];
      m->body_pos[3 * hollow_id + 2] = v[2];

      // hollow 的 freejoint 初始位姿（reset_world 会恢复到 qpos0）
      const int jid = m->body_jntadr[hollow_id];
      if (jid >= 0 && m->jnt_type[jid] == mjJNT_FREE) {
        const int qa = m->jnt_qposadr[jid];
        m->qpos0[qa + 0] = v[0];
        m->qpos0[qa + 1] = v[1];
        m->qpos0[qa + 2] = v[2];
        m->qpos0[qa + 3] = 1.0;
        m->qpos0[qa + 4] = m->qpos0[qa + 5] = m->qpos0[qa + 6] = 0.0;
        if (data) {
          data->qpos[qa + 0] = v[0];
          data->qpos[qa + 1] = v[1];
          data->qpos[qa + 2] = v[2];
          data->qpos[qa + 3] = 1.0;
          data->qpos[qa + 4] = data->qpos[qa + 5] = data->qpos[qa + 6] = 0.0;
        }
      } else {
        RCLCPP_WARN(node->get_logger(),
                    "workcell: hollow_cylinder 没有 freejoint，跳过 qpos0");
      }

      // rest weld relpose = world_T_hollow
      // eq_data 每个 equality 占 mjNEQDATA 个 double：
      // [0..2] anchor, [3..5] relpose 位置, [6..9] 四元数(wxyz), [10] torque scale
      const int e = rest_eq_id_ * mjNEQDATA;
      m->eq_data[e + 3] = v[0];
      m->eq_data[e + 4] = v[1];
      m->eq_data[e + 5] = v[2];
      m->eq_data[e + 6] = 1.0;
      m->eq_data[e + 7] = m->eq_data[e + 8] = m->eq_data[e + 9] = 0.0;

      RCLCPP_INFO(node->get_logger(),
                  "workcell from yaml: object=(%.3f, %.3f, %.3f) "
                  "slot=(%.3f, %.3f, %.3f)",
                  v[0], v[1], v[2], v[3], v[4], v[5]);
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