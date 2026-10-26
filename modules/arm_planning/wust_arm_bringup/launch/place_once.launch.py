"""一键入口：起整套 MuJoCo 仿真，延迟一段时间后自动触发一次"放置"任务。

等价于手敲：
  ros2 launch wust_arm_bringup mujoco_no_hardware.launch.py model:=new_wust
  ros2 service call /mtc_place_node/plan_and_execute std_srvs/srv/Trigger

终点坐标仍然来自 mtc/config/mtc_place_node.yaml 的 target_slot.*。
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup = get_package_share_directory('wust_arm_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='new_wust'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'mujoco_no_hardware.launch.py')),
            launch_arguments={'model': LaunchConfiguration('model')}.items(),
        ),

        # 等 move_group 和两个 controller（spawner 在 2s/3s）就绪再触发。
        # ros2 service call 本身会等 service 出现，这里只是给个起步延迟。
        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/mtc_place_node/plan_and_execute',
                     'std_srvs/srv/Trigger'],
                output='screen'),
        ]),
    ])
