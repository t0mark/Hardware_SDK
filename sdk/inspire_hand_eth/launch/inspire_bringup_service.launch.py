from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inspire_hand_eth',
            executable='inspire_service_server',
            name='inspire_hand_service_server',
            output='screen',
        ),
    ])
