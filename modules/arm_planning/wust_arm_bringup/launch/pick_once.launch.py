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
            launch_arguments={
                'model': LaunchConfiguration('model'),
                'task': 'pick',
            }.items(),
        ),

        TimerAction(period=8.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call',
                     '/mtc_pick_node/plan_and_execute',
                     'std_srvs/srv/Trigger'],
                output='screen'),
        ]),
    ])
