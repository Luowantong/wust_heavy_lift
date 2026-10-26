"""公共启动组件。

分成两层：
  * 模块级：与机器人模型无关的东西
  * build()：与模型强相关的节点，必须传 model 才能构造
"""
import os
from dataclasses import dataclass

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# --------------------------------------------------------------------------- #
# 模块级：与模型无关，可以随时 import
# --------------------------------------------------------------------------- #

launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('wust_arm_bringup'), 'config', 'launch_params.yaml')))

node_params = os.path.join(
    get_package_share_directory('wust_arm_bringup'), 'config', 'node_params.yaml')

# --------------------------------------------------------------------------- #
# 模型描述：一个 model 名推导出四样东西
# --------------------------------------------------------------------------- #

@dataclass(frozen=True)
class ModelSpec:
    model: str            # 例如 wust_seven_axis_arm
    description_pkg: str  # {model}_description
    moveit_pkg: str       # {model}_moveit_config
    config_prefix: str    # config/<prefix>.urdf.xacro / .srdf / .ros2_control.xacro
    arm_group: str        # MoveIt 规划组名
    hand_frame: str       # 末端 IK 坐标系
    rviz_file: str     
    planning_frame: str   


#: 目前三套模型的命名还不统一（config 里的文件名带着 _description 后缀），
#: 这里显式登记。等命名统一之后，这个表可以整个删掉，走 _derive() 默认推导。
_MODEL_OVERRIDES = {
    "wust_seven_axis_arm": dict(
        config_prefix="wust_seven_axis_arm_description",
        planning_frame="base_link",
    ),
    "new_wust": dict(planning_frame="base"),
    "wust_heavy_lift": dict(planning_frame="base"),
}


def _derive(model: str) -> ModelSpec:
    over = _MODEL_OVERRIDES.get(model, {})
    return ModelSpec(
        model=model,
        description_pkg=over.get("description_pkg", f"{model}_description"),
        moveit_pkg=over.get("moveit_pkg", f"{model}_moveit_config"),
        # 统一命名时 config_prefix 就等于 model
        config_prefix=over.get("config_prefix", model),
        arm_group=over.get("arm_group", "main"),
        hand_frame=over.get("hand_frame", "LINK_TCP"),
        rviz_file=over.get("rviz_file", "moveit.rviz"),
        planning_frame=over.get("planning_frame", "base_link")
    )


# --------------------------------------------------------------------------- #
# 与模型相关的节点
# --------------------------------------------------------------------------- #

@dataclass
class RobotNodes:
    spec: ModelSpec
    moveit_config: object
    robot_state_publisher: Node
    move_group_node: Node
    mtc_node: Node
    rviz_node: Node


def build(model: str, use_sim_time: bool = False, **overrides) -> RobotNodes:
        
    """按模型名构造全部与模型相关的节点。

    model 决定 description / moveit_config 两个包名和 config 里的文件名前缀；
    overrides 用于临时覆盖（例如 arm_group / hand_frame）。
    """
    spec = _derive(model)
    if overrides:
        spec = ModelSpec(**{**spec.__dict__, **overrides})

    # MoveItConfigsBuilder 的 package_name 参数让它直接去 {model}_moveit_config 找配置，
    # 不需要任何绝对/相对路径拼接。
    moveit_config = (
        MoveItConfigsBuilder(spec.model, package_name=spec.moveit_pkg)
        .robot_description(file_path=f"config/{spec.config_prefix}.urdf.xacro")
        .robot_description_semantic(file_path=f"config/{spec.config_prefix}.srdf")
        # .ros2_control.xacro 由上面那个 .urdf.xacro 内部 xacro:include，不用单独指定
        .planning_scene_monitor(publish_robot_description=True,
                                publish_robot_description_semantic=True)
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    sim = {"use_sim_time": use_sim_time}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,sim],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {"capabilities": "move_group/ExecuteTaskSolutionCapability"},sim],
    )

    mtc_node = Node(
        package="mtc",
        executable="mtc_place_node",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"planning_plugin": "ompl_interface/OMPLPlanner"},
            # ↓ 这两个已经是 ROS 参数了，按模型传即可
            {"arm_group": spec.arm_group},
            {"hand_frame": spec.hand_frame},
            sim,
            {"planning_frame": spec.planning_frame},
            os.path.join(get_package_share_directory('mtc'), 'config', 'mtc_place_node.yaml'),
        ],
        # 注意：--log-level 后面跟的是 logger 名 = 节点名，原来是 'mtc_place:=' 对不上
        ros_arguments=['--log-level',
                       f'mtc_place_node:={launch_params["mtc_node_log_level"]}'],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", PathJoinSubstitution(
            [FindPackageShare(spec.moveit_pkg), "config", spec.rviz_file])],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            sim
        ],
    )

    return RobotNodes(spec=spec, moveit_config=moveit_config,
                      robot_state_publisher=robot_state_publisher,
                      move_group_node=move_group_node,
                      mtc_node=mtc_node, rviz_node=rviz_node)
