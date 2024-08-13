from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_spawn_controllers_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("m0609", package_name="m0609_moveit").to_moveit_configs()
    return generate_spawn_controllers_launch(moveit_config)
