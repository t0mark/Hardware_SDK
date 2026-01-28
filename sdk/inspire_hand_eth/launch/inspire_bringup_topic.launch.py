from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inspire_hand_eth',
            executable='inspire_topic_node',
            name='inspire_hand_topic_node',
            output='screen',
        ),
    ])
