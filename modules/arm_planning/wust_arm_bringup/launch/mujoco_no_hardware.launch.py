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
    from common import build

    model = LaunchConfiguration('model').perform(context)
    use_sim_time_param = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    r = build(model, use_sim_time=use_sim_time_param)


    # 控制器路径也跟着 model 走
    ros2_controllers_path = os.path.join(
        get_package_share_directory(r.spec.moveit_pkg),
        "config", "ros2_controllers.yaml")

    ros2_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        parameters=[
            r.moveit_config.robot_description,
            ros2_controllers_path,
            os.path.join(get_package_share_directory('wust_arm_bringup'),
                         'config', 'mujoco_plugins.yaml'),
            {"use_sim_time": use_sim_time_param}
        ],
        output="screen",
    )


    joint_state_broadcaster = TimerAction(period=2.0, actions=[Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"], output="screen")])

    main_controller_spawner = TimerAction(period=3.0, actions=[Node(
        package="controller_manager", executable="spawner",
        arguments=["main_controller"], output="screen")])

    base_link_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base", "base_link"],
    )

    return [
        r.robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        main_controller_spawner,
        r.move_group_node,
        r.rviz_node,
        r.mtc_node,
        base_link_alias,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='wust_seven_axis_arm'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
