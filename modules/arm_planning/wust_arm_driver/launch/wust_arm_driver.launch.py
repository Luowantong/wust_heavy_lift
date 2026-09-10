import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wust_arm_driver'), 'config', 'serial_driver.yaml')

    wust_arm_driver_node = Node(
        package='wust_arm_driver',
        executable='wust_arm_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[config],
    )

    return LaunchDescription([wust_arm_driver_node])
