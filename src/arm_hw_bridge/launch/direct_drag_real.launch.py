from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'MROV',
        package_name='roboforge_moveit_config',
    ).to_moveit_configs()

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('arm_hw_bridge'),
                'rviz',
                'direct_drag.rviz',
            ]),
        ],
        parameters=[
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('use_serial', default_value='true'),
        DeclareLaunchArgument('serial_debug', default_value='true'),
        DeclareLaunchArgument('serial_status_poll_sec', default_value='1.0'),
        DeclareLaunchArgument('hardware_motion_enabled', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('roboforge_moveit_config'),
                    'launch',
                    'rsp.launch.py',
                ]),
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('roboforge_moveit_config'),
                    'launch',
                    'static_virtual_joint_tfs.launch.py',
                ]),
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('roboforge_moveit_config'),
                    'launch',
                    'move_group.launch.py',
                ]),
            ]),
        ),
        Node(
            package='arm_hw_bridge',
            executable='direct_drag',
            output='screen',
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {
                    'serial_port': LaunchConfiguration('serial_port'),
                    'use_serial': ParameterValue(LaunchConfiguration('use_serial'), value_type=bool),
                    'serial_debug': ParameterValue(LaunchConfiguration('serial_debug'), value_type=bool),
                    'serial_status_poll_sec': ParameterValue(LaunchConfiguration('serial_status_poll_sec'), value_type=float),
                    'hardware_motion_enabled': ParameterValue(LaunchConfiguration('hardware_motion_enabled'), value_type=bool),
                },
            ],
        ),
        rviz_node,
    ])
