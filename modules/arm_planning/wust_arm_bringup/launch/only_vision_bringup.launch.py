import os
import sys
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction, Shutdown
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
bringup_path = get_package_share_directory('wust_arm_bringup')
sys.path.append(os.path.join(bringup_path, 'launch'))

def generate_launch_description():
    from common import node_params, launch_params
    # 1. 组织相机组件
    hik_camera_node = ComposableNode(
        package='hik_camera',
        plugin='hik_camera::HikCameraNode',
        name='hik_camera',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}], # 开启进程内通信
    )

    # 2. 组织检测组件
    detector_node = ComposableNode(
        package='detector',
        plugin='exchange_slot::ExchangeSlotDetectorNode',
        name='exchange_slot_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}], # 开启进程内通信
    )

    # 3. 将组件放入容器
    container = ComposableNodeContainer(
        name='exchange_slot_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            hik_camera_node,
            detector_node
        ],
        output='both',
        ros_arguments=['--ros-args', '--log-level',
                       f'exchange_slot_detector:={launch_params["detector_log_level"]}'],
        on_exit=Shutdown(),
    )
    # 4. RViz 节点
    # (可选) RViz 配置文件路径
    # 如果你之前保存过 rviz 配置，可以指向它；否则启动默认配置
    pkg_share = get_package_share_directory('wust_arm_bringup')
    rviz_config_path = os.path.join(pkg_share, 'config', 'wust_arm_bringup.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # 如果有保存好的配置文件则加载，没有则打开空白 rviz
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='both'
    )

    # 5. (可选) 静态 TF 发布器
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    return LaunchDescription([
            static_tf,
            container,
            rviz_node,
            robot_state_publisher
            ])