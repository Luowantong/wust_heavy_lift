import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 获取包的共享路径
    pkg_share = get_package_share_directory('hik_camera')

    # 2. 定义 Launch 参数（可通过命令行覆盖）
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'camera_params.yaml'),
        description='Path to the ROS2 parameters file'
    )

    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value=f'package://hik_camera/config/camera_info.yaml',
        description='URL to the camera calibration file'
    )

    use_sensor_qos_arg = DeclareLaunchArgument(
        'use_sensor_data_qos',
        default_value='false',
        description='Whether to use SensorDataQoS'
    )

    # 3. 配置 Node 实例
    # 将参数列表提取出来，方便复用或修改
    hik_camera_node = Node(
        package='hik_camera',
        executable='hik_camera_node',
        name='hik_camera',  # 明确节点名称
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('params_file'), # 加载整个 yaml 文件
            {
                'camera_info_url': LaunchConfiguration('camera_info_url'),
                'use_sensor_data_qos': LaunchConfiguration('use_sensor_data_qos'),
            }
        ]
    )

    # 4. 返回 LaunchDescription
    # 这样做的好处是逻辑清晰，你可以轻松地在这里添加更多节点（如检测节点）
    return LaunchDescription([
        params_file_arg,
        camera_info_url_arg,
        use_sensor_qos_arg,
        hik_camera_node
    ])