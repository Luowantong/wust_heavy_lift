from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("new_wust", package_name="new_wust_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
