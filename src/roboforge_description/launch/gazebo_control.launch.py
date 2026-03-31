from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('roboforge_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'MROV.urdf')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen'
    )

    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_file, '-entity', 'roboforge_robot', '-z', '0.3'],
        output='screen'
    )

    # Load controllers
    load_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner',
                    'joint_state_broadcaster'
                ],
                output='screen'
            )
        ]
    )

    load_arm_controller = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner',
                    'arm_controller',
                    '--param-file', controllers_file
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        load_joint_state_broadcaster,
        load_arm_controller
    ])
