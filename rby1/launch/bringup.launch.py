from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('rby1')

    return LaunchDescription([
        # ── 런치 인자 ────────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'robot_address',
            default_value='192.168.12.1:50051',
            description='Robot gRPC address (host:port)',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='a',
            choices=['a', 'm', 'ub'],
            description='Robot model (a / m / ub)',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='original',
            choices=['inspire', 'original'],
            description='End-effector type',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='50.0',
            description='Joint state publish rate [Hz]',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz',
        ),

        # ── description (publish_joints:=false — 하드웨어에서 직접 받음) ──────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_share, 'launch', 'description.launch.py'])
            ]),
            launch_arguments={
                'publish_joints': 'false',
                'rviz': LaunchConfiguration('rviz'),
                'end_effector': LaunchConfiguration('end_effector'),
            }.items(),
        ),

        # ── hardware_node ────────────────────────────────────────────────────────
        Node(
            package='rby1',
            executable='hardware_node',
            name='rby1_hardware',
            output='screen',
            parameters=[{
                'robot_address': LaunchConfiguration('robot_address'),
                'model': LaunchConfiguration('model'),
                'rate': LaunchConfiguration('rate'),
            }],
        ),

        # ── mobility_node ────────────────────────────────────────────────────────
        Node(
            package='rby1',
            executable='mobility_node',
            name='rby1_mobility',
            output='screen',
            parameters=[{
                'robot_address': LaunchConfiguration('robot_address'),
                'model': LaunchConfiguration('model'),
            }],
        ),
    ])
