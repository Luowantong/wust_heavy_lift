# WUST Arm Robot

RoboMaster 工程机器人软件仓库，采用多任务模块化架构。

## 目录结构

- `common/`：共享基础设施（相机、串口、工具）
- `modules/arm_planning/`：机械臂运动规划（基于 MoveIt）
- `modules/vision/`：视觉模块（兑矿、吊射）
- `modules/launcher_install/`：发射机构安装任务
- `launch/`：不同模式的启动入口

## 快速开始

```bash
# 编译
colcon build --symlink-install

# 加载环境
source install/setup.bash

# 启动兑矿模式
ros2 launch launch/exchange_mode.launch.py
