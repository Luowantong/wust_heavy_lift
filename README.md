# WUST Arm Robot

RoboMaster 工程机器人软件仓库，采用多任务模块化架构。

## 目录结构

- `common/`：共享基础设施（相机、串口、工具）
- `modules/arm_planning/`：机械臂运动规划（基于 MoveIt / MTC）
- `modules/vision/`：视觉模块（兑矿、吊射）
- `modules/launch_install/`：发射机构安装任务
- `launch/`：不同模式的启动入口

## 快速开始

```bash
# 编译
colcon build --symlink-install

# 加载环境
source install/setup.bash

# 启动兑矿模式
ros2 launch launch/exchange_mode.launch.py
```

## 机械臂规划（MTC）

### 支持的模型

| model | 取料点（物体中心） | 放置点（插槽原点） |
|---|---|---|
| `new_wust` | (0.40, 0, 0.28) | (0.40, 0, 0.20) |
| `wust_heavy_lift` | (0.34, 0, 0.28) | (0.34, 0, 0.20) |

两个模型的末端都是真空吸盘（`vacuum_tcp` → `LINK_TCP`），抓取时吸盘原点压在**单元顶面中心**，单元中心在吸盘正下方 7.5 cm。

### 运行

```bash
cd modules/arm_planning
colcon build --symlink-install
source install/setup.bash

# 拾取
ros2 launch wust_arm_bringup mujoco_no_hardware.launch.py model:=new_wust task:=pick
ros2 service call /mtc_pick_node/plan_and_execute std_srvs/srv/Trigger

# 放置
ros2 launch wust_arm_bringup mujoco_no_hardware.launch.py model:=new_wust task:=place
ros2 service call /mtc_place_node/plan_and_execute std_srvs/srv/Trigger

# 复位仿真（服务类型不是 std_srvs/Trigger）
ros2 service call /mujoco_ros2_control_node/reset_world \
  mujoco_ros2_control_msgs/srv/ResetWorld "{keyframe: ''}"
```

`pick_once.launch.py` 会在启动 8 秒后自动触发一次 pick。

**注意**：`auto_start` 默认为 `false`，`mujoco_no_hardware.launch.py` 不会自动执行任务，必须手动调服务。

### 参数分层

```
config/common.yaml → config/{pick,place}.yaml → config/models/<model>.yaml
```

ROS 2 里**后加载的覆盖前面的**，所以模型相关的值必须放在最后一个文件里。

- `common.yaml`：触发方式（`auto_start` 等）
- `pick.yaml` / `place.yaml`：与模型无关的参数（如 `yaw_candidates`）
- `models/<model>.yaml`：目标坐标、`approach_height`、`lift_height`、`k_retreat`

### 改目标坐标时要同步的地方

1. `mtc/config/models/<model>.yaml` 的 `target_object` / `target_slot`
2. MJCF 里 `slot` body 的 `pos`
3. MJCF 里 `hollow_cylinder` body 的 `pos`
4. MJCF 里 `rest` weld 的 `relpose`
5. 若同时改了高度，还有 `mtc/include/mtc/common/geometry.hpp` 的 `pre_place_height_`

前四项漏改任何一处，都会出现"MoveIt 规划的位置和 MuJoCo 里的实物对不上"。

### 已知约束

- **工作空间**：`wust_heavy_lift` 在 x ≥ 0.38 处、z≈0.40~0.43 这一段没有任何无碰撞 IK 解，所以它的取料点定在 0.34。`new_wust` 在 x=0.45 附近竖直只能抬约 6 cm，抬升方向要用 `k_retreat` 斜向回收。
- **MTC 的 `ModifyPlanningScene`**：允许/禁止碰撞的 stage 必须放在受它影响的运动 stage **之前**，并且要是正向传播（任务树里箭头为 `→`）。反向传播（`←`）会把 `allow` 反转成 `forbid`。
- **抓取事件**：MuJoCo 的 `rest`/`grasp` 两条 weld 由 GraspPlugin 按 `/attached_collision_object` 切换，而 move_group / MTC **不会发布**这个 topic，需要 `TrajectoryExecutor` 在 `attach_hollow` / `detach_objects` 阶段自己发。
- **place 单独运行**会立刻把单元吸附到夹爪（它假设物体已经在手上）；要演示完整流程，需要先跑 pick。

### 排查提示

- 日志里的 `Achieved: 0.xx` 是**笛卡尔路径完成比例**，不是距离；`min_distance not reached` 只是它的下游表现。
- `compute_ik` 全部 `no IK found`：先用 `/compute_ik` 扫一下目标点是否在工作空间内（`avoid_collisions: true` 才是真实可用的判据）。
- 仿真启动失败先看这一行，确认加载的是哪个 MJCF：
  `[MujocoSystemInterface]: Loading 'mujoco_model' from: ...`
