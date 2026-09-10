import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Get the package share directory
    pkg_name = 'wust_seven_axis_arm_description'
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # Path to the URDF file
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'wust_seven_axis_arm.urdf')

    # Read the URDF file content
    try:
        with open(urdf_path, 'r') as infp:
            robot_description_content = infp.read()
    except EnvironmentError:
        print("URDF file not found. Please ensure it is in the urdf/ directory and the package has been built.")
        return LaunchDescription([]) # 启动失败，返回空描述

    # Define the robot description parameter
    robot_description = {'robot_description': robot_description_content}

    # Start the robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Start the joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # Start the RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])