"""
RBY1 teleop launch — joystick

Usage:
  ros2 launch rby1 teleop.launch.py
  ros2 launch rby1 teleop.launch.py linear_axis:=1 angular_axis:=0 linear_speed:=0.3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    linear_speed_arg  = DeclareLaunchArgument('linear_speed',  default_value='0.3')
    angular_speed_arg = DeclareLaunchArgument('angular_speed', default_value='0.5')
    linear_axis_arg   = DeclareLaunchArgument('linear_axis',   default_value='1')
    angular_axis_arg  = DeclareLaunchArgument('angular_axis',  default_value='0')
    joy_dev_arg       = DeclareLaunchArgument('joy_dev',       default_value='/dev/input/js0')

    def create_nodes(context, *args, **kwargs):
        return [
            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[{'dev': context.launch_configurations['joy_dev']}],
                output='screen',
            ),
            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy',
                parameters=[{
                    'axis_linear.x':     int(context.launch_configurations['linear_axis']),
                    'axis_angular.yaw':  int(context.launch_configurations['angular_axis']),
                    'scale_linear.x':    float(context.launch_configurations['linear_speed']),
                    'scale_angular.yaw': float(context.launch_configurations['angular_speed']),
                    'require_enable_button': False,
                }],
                remappings=[('cmd_vel', '/cmd_vel')],
                output='screen',
            ),
        ]

    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,
        linear_axis_arg,
        angular_axis_arg,
        joy_dev_arg,
        OpaqueFunction(function=create_nodes),
    ])
