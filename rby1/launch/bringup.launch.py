"""
RBY1 bringup launch

Starts:
  1. robot_state_publisher  — TF from URDF
  2. ros2_control_node      — hardware interface + controller manager
  3. joint_state_broadcaster
  4. rby1_wholebody_controller

Usage:
  ros2 launch rby1 bringup.launch.py robot_ip:=192.168.30.1:50051
"""

import os

from ament_index_python.packages import get_package_share_directory
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rby1')

    # ------------------------------------------------------------------ args
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.30.1:50051',
        description='Robot gRPC address (host:port)',
    )

    # ------------------------------------------------------------------ URDF
    robot_ip = LaunchConfiguration('robot_ip')

    # xacro can't take LaunchConfiguration directly at parse time, so we use
    # a PythonExpression / OpaqueFunction to delay evaluation.
    from launch.actions import OpaqueFunction

    def create_nodes(context, *args, **kwargs):
        ip = context.launch_configurations['robot_ip']

        urdf_path = os.path.join(pkg, 'urdf', 'rby1_full_ros.urdf.xacro')
        robot_description = xacro.process_file(
            urdf_path, mappings={'robot_ip': ip}
        ).toxml()

        controllers_yaml = os.path.join(pkg, 'config', 'rby1_controllers.yaml')

        # 1. robot_state_publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        )

        # 2. ros2_control_node (controller_manager)
        ctrl_node = Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                controllers_yaml,
            ],
        )

        # 3. joint_state_broadcaster (spawned after controller_manager starts)
        jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )

        # 4. rby1_wholebody_controller (spawned 2 s after controller_manager)
        wb_spawner = TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'rby1_wholebody_controller',
                        '--controller-manager', '/controller_manager',
                    ],
                    output='screen',
                )
            ],
        )

        # Spawn joint_state_broadcaster only after controller_manager is up
        jsb_on_start = RegisterEventHandler(
            OnProcessStart(
                target_action=ctrl_node,
                on_start=[jsb_spawner],
            )
        )

        return [rsp_node, ctrl_node, jsb_on_start, wb_spawner]

    return LaunchDescription([
        robot_ip_arg,
        OpaqueFunction(function=create_nodes),
    ])
