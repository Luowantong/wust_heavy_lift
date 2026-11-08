#!/usr/bin/env bash
# RM26 兑矿仿真一键启动：仿真 + 固定相机 TF + 感知节点
# 用法: bash start_rm26.sh  (或 ./start_rm26.sh)
# 不用 -u：ROS 的 setup.bash 会引用未设置的 AMENT_* 变量，-u 会误报
set -eo pipefail

# 与 env.sh 同目录，按脚本自身位置加载环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

LAUNCH=/tmp/rm26_sim.launch.py
cat > "$LAUNCH" <<'PY'
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('wust_arm_bringup')

    # 1) 仿真 + move_group + rviz + mtc_pick_node
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup, 'launch', 'mujoco_no_hardware.launch.py')
        ),
        launch_arguments={
            'model': 'wust_heavy_lift',
            'task': 'place',
        }.items(),
    )

    # 2) 固定外置相机 TF：base -> exchange_cam_optical
    #    (q 是 ROS 光学帧，R_optical = R_mujoco * Rx(-pi)，与 MJCF <camera> 的
    #     pos/quat 对应；xyzw 顺序)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='exchange_cam_tf',
        arguments=[
            '--x', '0.00', '--y', '0.15', '--z', '1.10',
            '--qx', '-0.668585', '--qy', '0.668585', '--qz', '-0.230204',
            '--qw', '0.230204',
            '--frame-id', 'base', '--child-frame-id', 'exchange_cam_optical',
        ],
    )

    # 3) 感知节点（PYTHONPATH 由 env.sh 提供，随 launch 进程继承）
    perception = ExecuteProcess(
        cmd=[
            'python3', '-m', 'arm_exchange_host.perception_node', '--ros-args',
            '-p', 'use_sim_time:=true',
            '-p', 'camera_side:=left',
            '-p', 'show_window:=false',
            '-r', '/host/perception/exchange_station_pose:=/exchange_slot/slot_pose',
        ],
        name='arm_exchange_perception',
        output='screen',
    )

    # 等仿真把 /clock 和 TF 树拉起来再起感知
    perception_delayed = TimerAction(period=8.0, actions=[perception])

    return LaunchDescription([sim, static_tf, perception_delayed])
PY

exec ros2 launch "$LAUNCH"
