import os
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction, Shutdown

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('wust_arm_bringup'), 'config', 'launch_params.yaml')))

node_params = os.path.join(
    get_package_share_directory('wust_arm_bringup'), 'config', 'node_params.yaml')

# 统一 MoveIt 配置加载
moveit_config = (
    MoveItConfigsBuilder("wust_seven_axis_arm")
    .robot_description(file_path="config/wust_seven_axis_arm_description.urdf.xacro")
    .robot_description_semantic(file_path="config/wust_seven_axis_arm_description.srdf")
    .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
    .joint_limits(file_path="config/joint_limits.yaml")
    .trajectory_execution(file_path="config/moveit_controllers.yaml")
    .robot_description_kinematics(file_path="config/kinematics.yaml")
    .planning_pipelines(pipelines=["ompl"])
    .to_moveit_configs()
)

robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="both",
    parameters=[moveit_config.robot_description],
)

move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[moveit_config.to_dict(), 
                {"capabilities": "move_group/ExecuteTaskSolutionCapability"}],
)

# 规划节点定义
mtc_place_node = Node(
    package="mtc_place",
    executable="mtc_place_node",
    output="both",
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.planning_pipelines,
        moveit_config.joint_limits,
        {"planning_plugin": "ompl_interface/OMPLPlanner"}
    ],
    ros_arguments=['--log-level', 'mtc_place:='+launch_params['mtc_node_log_level']],
)


#  RViz
rviz_config = PathJoinSubstitution(
    [FindPackageShare("wust_seven_axis_arm_moveit_config"), "launch", LaunchConfiguration("rviz_config")]
)
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config],
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ],
)

# hik_camera
hik_camera_node = ComposableNode(
    package='hik_camera',
    plugin='hik_camera::HikCameraNode',
    name='hik_camera',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}], # 开启进程内通信
)

# detector_node
detector_node = ComposableNode(
    package='detector',
    plugin='exchange_slot::ExchangeSlotDetectorNode',
    name='exchange_slot_detector',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}], # 开启进程内通信
)

# detector_container
detector_container = ComposableNodeContainer(
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