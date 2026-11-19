# WUST Heavy Lift

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

两个模型的末端抓取点都是 `LINK_TCP`（URDF 里的 `vacuum_tcp` fixed joint）。单元相对 TCP 的位姿由 `tcp_hollow` 参数决定，当前两个模型都配置为：

```yaml
tcp_hollow: {roll: 0.0, pitch: -1.5708, yaw: 0.0, x: 0.0, y: 0.0, z: 0.0}
```

即 `T_tcp_hollow = Ry(-90°)`、原点重合：**单元中心就是 LINK_TCP 原点**，单元轴线沿 TCP 的 -X（侧抓）。换抓取方式（顶抓/侧抓）只改这个参数，不用动 URDF。

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

仓库根目录另有两只一键脚本（会顺带拉起外置相机静态 TF 和感知节点）：

```bash
bash start_rm26.sh place     # 或 pick
bash stop_rm26.sh            # 停整栈 + 清 DDS 残留；--dry-run 只看会杀谁
```

`start_rm26.sh` 默认 `ROS_LOCALHOST_ONLY=1`，避免实验室网段另一台机器的 `/clock` 串进来。

**注意**：`reset_world` 之后位置控制器会重新下发上一轮的 setpoint，手臂不会自己回 home。要让仿真回到干净起点，复位后必须再发一条回零轨迹（例如往 `/main_controller/joint_trajectory` 发单点全零轨迹）并确认关节到位。

`pick_once.launch.py` 会在启动 8 秒后自动触发一次 pick。

**注意**：`auto_start` 默认为 `false`，`mujoco_no_hardware.launch.py` 不会自动执行任务，必须手动调服务。

### 参数分层

```
config/common.yaml → config/{pick,place}.yaml → config/models/<model>.yaml
```

ROS 2 里**后加载的覆盖前面的**，所以模型相关的值必须放在最后一个文件里。

- `common.yaml`：触发方式（`auto_start` 等）
- `pick.yaml` / `place.yaml`：与模型无关的参数（如 `yaw_candidates`）
- `models/<model>.yaml`：`workcell` 目标坐标、`tcp_hollow`、`approach_height`、`lift_height`、`k_retreat`

模型 yaml 顶部的 `/**` 段会同时被 `mtc_*_node` 和 `mujoco_ros2_control` 读取：

- MTC 用它作为规划目标（pick 读 `workcell.object`，place 读 `workcell.slot`）；
- `GraspPlugin` 在 `init()` 里用它覆盖 MJCF 的 `slot` / `hollow_cylinder` 位置和 `rest` weld，保证仿真里的实物和规划目标一致。

### 改目标坐标时只需要改一处

```yaml
/**:
  ros__parameters:
    workcell:
      object: {x: 0.34, y: 0.0, z: 0.28, roll: 0.0, pitch: 0.0, yaw: 0.0}
      slot:   {x: 0.34, y: 0.0, z: 0.20, roll: 0.0, pitch: 0.0, yaw: 0.0}
```

`roll/pitch/yaw` 是 URDF rpy 约定、单位弧度。位置和姿态都会被 MTC 和 GraspPlugin 同时读取：

- MTC 用它生成 pick/place 的 6-DoF 目标；
- GraspPlugin 用它覆盖 MJCF 的 `slot` body、`hollow_cylinder` body/freejoint 和 `rest` weld 的位置 + 四元数。

所以 slot 朝向任意角度时，只需要改这里的 rpy。注意：物体在插槽里时，`object` 的**姿态**应与 `slot` 一致，位置应满足 `object = slot + R_slot · (0, 0, 0.08)`（竖直时就是 z 差 8 cm）；否则物体会和销钉错开。

启动日志里确认参数确实传到了插件：

```
[GraspPlugin]: workcell from yaml: object=(...) slot=(...)
```

如果打印的是 `workcell.* 参数不可见`，说明 plugin 的子节点没有继承到这组参数，需要把 `workcell` 段挪到 `wust_arm_bringup/config/mujoco_plugins.yaml`。

唯一仍需手动保持一致的量是 **z 方向**：改高度时同步 `mtc/include/mtc/common/geometry.hpp` 的 `pre_place_height_`（place 的预放置高度）。

### place 的目标处理顺序

每次触发都按这条顺序走（`task_node_base.cpp`）：

1. **快照**：把 `target_slot_` 拷到 `resolved_target_`，之后整轮只读这一份；
2. **定稿**：`resolveTarget()` 跑 IK 种子阶梯，全败则回退 `workcell.slot`（日志 `DEGRADED`）；
3. **建场景**：`setupPlanningScene(resolved_target_)`；
4. **建树**：`createTask()` 只建 stage，不再改目标。

这三步在 `plan_attempts` 循环里每个 attempt 重跑一次，所以降级/重试都不会出现"场景按 A 建、运动按 B 走"。

单独调试 place 时手臂停在 home 位（距站体约 0.31 m），会被遮挡判据全部拒绝、锁不上。此时打开观察位：

```yaml
observe_pose_before_place: true
observe_pose: [0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # main 组 7 个关节角
```

规划前会先走到该构型、等 `target_locked_`（最多 `observe_lock_timeout` 秒）再继续；锁不上就用 YAML 跑，不报错。比赛流程 pick→place 手臂本来就在远处，不需要开。

**推顶**：自动 `push_up` 已移除，插入到位后的"向上顶"交给操作手。若以后恢复，用独立的 `CartesianPath` 沿站体 +z 走，且在 `detach_objects` 之前（那时单元还在夹爪上）。

### 感知解码约定

检测器解码必须和训练侧 `MSRAHeatmap` 一致（mmpose `keypoints_from_heatmaps`）：

```
输入坐标 = 热图坐标 × (input_size / heatmap_size)
```

不能额外加减 0.5。实测：原实现 `(x+0.5)*stride` 会让 12 个关键点整体偏 1 输入像素，最终放置偏差约 4.7 mm；对齐后降到 1.4 mm。

`bbox_padding` 在仿真里实测 **1.10 优于训练默认的 1.25**（放置误差 1.43 mm vs 5.66 mm）。这个结论是在"单元焊在夹爪前、部分遮挡"的仿真视角下得到的，接真机后需要重新标定。

### 10 次连跑与几何判据

单次 `DONE` 不能当成功判据，验收用 10 次连跑：复位 → 回 home → 触发 → 等 DONE/FAILED → 量几何误差。

几何口径：

- 期望座位中心：`target · T(-0.023, 0, 0) · Ry(-90°)`（站体局部坐标）；
- 实测物体系：`base←LINK_TCP · T_tcp_hollow`；
- 这是**松手时刻的运动学位姿**，测不到松手之后的掉落。

当前基线（各 10 次）：

| 组 | err_vs_truth 均值 | 最大 | 3 mm 内 |
|---|---|---|---|
| YAML（`allow_slot_override: false`） | 0.59 mm | 0.76 mm | 10/10 |
| 感知锁定（`true`） | 1.43 mm | 1.52 mm | 10/10 |

### 已知约束

- **工作空间**：`wust_heavy_lift` 在 x ≥ 0.38 处、z≈0.40~0.43 这一段没有任何无碰撞 IK 解，所以它的取料点定在 0.34。`new_wust` 在 x=0.45 附近竖直只能抬约 6 cm，抬升方向要用 `k_retreat` 斜向回收。
- **MTC 的 `ModifyPlanningScene`**：允许/禁止碰撞的 stage 必须放在受它影响的运动 stage **之前**，并且要是正向传播（任务树里箭头为 `→`）。反向传播（`←`）会把 `allow` 反转成 `forbid`。
- **抓取事件**：MuJoCo 的 `rest`/`grasp` 两条 weld 由 GraspPlugin 按 `/attached_collision_object` 切换，而 move_group / MTC **不会发布**这个 topic，需要 `TrajectoryExecutor` 在 `attach_hollow` / `detach_objects` 阶段自己发。
- **place 单独运行**会立刻把单元吸附到夹爪（它假设物体已经在手上）；要演示完整流程，需要先跑 pick。
- **碰撞放行（当前状态）**：`place_container` 里对 `hollow ↔ exchange_station`、`link7 ↔ exchange_station`、`LINK_TCP ↔ exchange_station` 都是 **allow**。原因是碰撞模型是实心近似：link7 的网格把 peg 整个罩住，不放行会在最后几毫米判碰撞，`CartesianPath` 只走 90%（臂停在离座位 18 mm 处却报 DONE）。这是临时豁免，不是真实无碰撞。
- **什么时候可以收回放行**：拿到夹爪单独模型、或把 link7 碰撞体换成更贴合的形状（例如把 peg 段从 link7↔station 的检查里剔除）之后，`link7 ↔ station` 这一条应撤掉，只保留 `hollow ↔ station`（插入本身必须）；`LINK_TCP` 目前没有碰撞体，可先保留。
- **抓取方式**：当前是 weld（`rest`/`grasp` 两条 equality 约束）而不是接触夹取——末端在模型里是 `vacuum_tcp` 一个 fixed joint，link7 网格内也没有独立的手指关节。要换成真实夹取，需要机械组提供夹爪拆分模型 + 手指关节/行程/质量惯量，仿真侧还要补手指 body、actuator 和接触参数。

### 排查提示

- 日志里的 `Achieved: 0.xx` 是**笛卡尔路径完成比例**，不是距离；`min_distance not reached` 只是它的下游表现。
- `compute_ik` 全部 `no IK found`：先用 `/compute_ik` 扫一下目标点是否在工作空间内（`avoid_collisions: true` 才是真实可用的判据）。
- 仿真启动失败先看这一行，确认加载的是哪个 MJCF：
  `[MujocoSystemInterface]: Loading 'mujoco_model' from: ...`
