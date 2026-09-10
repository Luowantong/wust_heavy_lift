import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包路径
    pkg_share = get_package_share_directory('detector')
    
    # 2. 配置文件路径
    config_path = os.path.join(pkg_share, 'config', 'exchange_slot_detector.yaml')
    
    # 3. (可选) RViz 配置文件路径
    # 如果你之前保存过 rviz 配置，可以指向它；否则启动默认配置
    rviz_config_path = os.path.join(pkg_share, 'config', 'detector.rviz')

    # 4. 识别节点
    detector_node = Node(
        package='detector',
        executable='detector_node',
        name='exchange_slot_detector',
        output='screen',
        parameters=[config_path], # 建议在 yaml 里改好，这里保持简洁
        # remappings 根据你的实际话题决定是否开启
        # remappings=[('/image_raw', '/camera/image_raw')] 
    )

    # 5. RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # 如果有保存好的配置文件则加载，没有则打开空白 rviz
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen'
    )

    # 6. (可选) 静态 TF 发布器
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_world_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'world', 'camera_optical_frame']
    )

    return LaunchDescription([
        detector_node,
        rviz_node,
        static_tf_node
    ])