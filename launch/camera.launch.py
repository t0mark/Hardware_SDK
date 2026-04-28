from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device',
            default_value='0',
            description='V4L2 camera device index',
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Camera capture and publish FPS',
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value='HD720',
            choices=['HD2K', 'HD1080', 'HD720', 'VGA'],
            description='ZED2i UVC side-by-side resolution',
        ),
        Node(
            package='rby1',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'fps': LaunchConfiguration('fps'),
                'resolution': LaunchConfiguration('resolution'),
            }],
        ),
    ])
