"""
loco_bridge.launch.py

Launch file for the Unitree robot loco bridge node.
Selects the appropriate node based on the 'hardware' argument.

Usage:
  ros2 launch unitree_utils loco_bridge.launch.py hardware:=go2
  ros2 launch unitree_utils loco_bridge.launch.py hardware:=g1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    hardware = LaunchConfiguration("hardware").perform(context)

    if hardware == "go2":
        node = Node(
            package="unitree_utils",
            executable="go2_loco_bridge_node",
            name="go2_loco_bridge_node",
            output="screen",
        )
    elif hardware == "g1":
        node = Node(
            package="unitree_utils",
            executable="g1_loco_bridge_node",
            name="g1_loco_bridge_node",
            output="screen",
        )
    else:
        raise ValueError(f"Unknown hardware: '{hardware}'. Must be 'go2' or 'g1'.")

    return [node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware",
            default_value="g1",
            description="Target robot hardware: 'go2' or 'g1'",
        ),
        OpaqueFunction(function=launch_setup),
    ])
