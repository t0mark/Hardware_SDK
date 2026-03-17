import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('rby1')

    urdf_file = os.path.join(pkg_share, 'urdf', 'rby1_full_ros.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'rby1.rviz')

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' end_effector:=', LaunchConfiguration('end_effector')]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz if true',
        ),

        DeclareLaunchArgument(
            'publish_joints',
            default_value='true',
            description='Launch joint_state_publisher_gui if true',
        ),

        DeclareLaunchArgument(
            'end_effector',
            default_value='original',
            choices=['inspire', 'original'],
            description='End-effector type: inspire (RH56E2 dexterous hand) or original (2-finger gripper)',
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
            arguments=['--ros-args', '--log-level', 'warn', '--disable-stdout-logs'] # 경고 이상의 로그만 허용하고 콘솔 출력 차단
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_joints')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
