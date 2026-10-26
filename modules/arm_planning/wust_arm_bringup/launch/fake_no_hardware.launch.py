import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

bringup_path = get_package_share_directory('wust_arm_bringup')
sys.path.append(os.path.join(bringup_path, 'launch'))


def launch_setup(context, *args, **kwargs):
    from common import build, node_params

    model = LaunchConfiguration('model').perform(context)
    r = build(model)

    wust_arm_driver_node = Node(
        package="wust_arm_driver",
        executable="wust_arm_driver_node",
        name="wust_arm_driver",
        output="both",
        parameters=[node_params, r.moveit_config.to_dict()],   # ← 用 r 的
    )

    return [
        r.robot_state_publisher,
        wust_arm_driver_node,
        r.move_group_node,
        r.mtc_node,
        r.rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='wust_seven_axis_arm'),
        OpaqueFunction(function=launch_setup),
    ])
