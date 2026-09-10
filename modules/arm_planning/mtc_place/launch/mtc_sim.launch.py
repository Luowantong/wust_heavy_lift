import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #  Launch 参数
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value="moveit.rviz", description="RViz configuration file"
    )
    ros2_control_hardware_type_arg = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware type (mock or real)",
    )

    #  MoveIt 配置
    moveit_config = (
        MoveItConfigsBuilder("wust_arm_7axis")
        .robot_description(
            file_path="config/wust_arm_7axis_description.urdf.xacro",
            mappings={"ros2_control_hardware_type": LaunchConfiguration("ros2_control_hardware_type")},
        )
        .robot_description_semantic(file_path="config/wust_arm_7axis_description.srdf")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    #  核心节点
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), 
                    move_group_capabilities],
        arguments=["--ros-args", "--log-level", "info"],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    #  ros2_control (mock components)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("wust_arm_7axis_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    #  控制器 spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )

    main_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "main_controller",
            "-c", "/controller_manager",
            "--controller-type", "joint_trajectory_controller/JointTrajectoryController"
        ],
        output="screen",
    )

    #  RViz
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("wust_arm_7axis_moveit_config"), "launch", LaunchConfiguration("rviz_config")]
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

    #  自定义 mtc_place_node
    mtc_place_node = Node(
        package="mtc_place",
        executable="mtc_place_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "planning_plugin": "ompl_interface/OMPLPlanner"
            }
        ],
        # arguments=[
        #     "--ros-args",
        #     "--log-level",
        #     "debug",
        # ],
    )


    #  LaunchDescription 
    return LaunchDescription([
        rviz_config_arg,
        ros2_control_hardware_type_arg,
        robot_state_publisher_node,
        static_tf_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        main_controller_spawner,
        # hand_controller_spawner,
        move_group_node,
        rviz_node,
        # mtc_place_node
    ])
