import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # 1. MoveIt Config
    moveit_config = (
        MoveItConfigsBuilder("wust_arm_7axis")
        .robot_description(file_path="config/wust_arm_7axis_description.urdf.xacro")
        .robot_description_semantic(file_path="config/wust_arm_7axis_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 3. ros2_control
    ros2_controllers_path = os.path.join(
        get_package_share_directory("wust_arm_7axis_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # 4. Controllers
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    main_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["main_controller"],
        output="screen",
    )

    # 5. Move Group
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }
    #  核心节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        parameters=[moveit_config.to_dict(), 
                    move_group_capabilities],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # 6. RViz 
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("wust_arm_7axis_moveit_config"), "launch", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="both",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )


    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        main_controller_spawner,
        move_group_node,
        rviz_node
    ])
