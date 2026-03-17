from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('rby1')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.12.1:50051',
            description='Robot gRPC address (ip:port)',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg_share, '/launch/description.launch.py']),
            launch_arguments={
                'publish_joints': 'false',
                'rviz': 'true',
            }.items(),
        ),

        Node(
            package='rby1',
            executable='bringup_node',
            name='rby1_bringup_node',
            output='screen',
            parameters=[{'robot_ip': LaunchConfiguration('robot_ip')}],
        ),
    ])
