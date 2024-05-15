from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from ur_moveit_config.launch_common import load_yaml

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("open_manipulator_x", package_name="manipulatorx_moveit").to_moveit_configs()
    return generate_demo_launch(moveit_config)
