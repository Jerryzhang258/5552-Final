from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("ur5", package_name="ur5_moveit_config")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
