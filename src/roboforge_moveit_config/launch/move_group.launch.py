from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "MROV",
        package_name="roboforge_moveit_config"
    ).to_moveit_configs()

    moveit_config.trajectory_execution["use_sim_time"] = True
    moveit_config.planning_scene_monitor["use_sim_time"] = True

    return generate_move_group_launch(moveit_config)
