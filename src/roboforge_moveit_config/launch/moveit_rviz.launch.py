from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "MROV",
        package_name="roboforge_moveit_config"
    ).to_moveit_configs()

    moveit_config.planning_pipelines["use_sim_time"] = True
    moveit_config.robot_description_kinematics["use_sim_time"] = True
    moveit_config.joint_limits["use_sim_time"] = True

    return generate_moveit_rviz_launch(moveit_config)
