#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.11.1',
        description='Slamware IP address'
    )
    
    ip_address = LaunchConfiguration('ip_address')
    
    # Slamware node - TF disabled to prevent conflicts with robot_state_publisher
    slamware_node = Node(
        package='slamware_ros_sdk',
        executable='slamware_ros_sdk_server_node',
        name='slamware_ros_sdk_server_node',
        output='both',
        parameters=[
            {'ip_address': ip_address},
            {'angle_compensate': True},
            {'fixed_odom_map_tf': False},  # Disable TF publishing from slamware
            {'raw_ladar_data': False},
            {'robot_frame': 'slamtec_base_link'},
            {'odom_frame': 'odom'},
            {'laser_frame': 'laser'},
            {'map_frame': 'slamware_map'},
            {'robot_pose_frame': 'robot_pose'},
            {'odometry_pub_period': 0.05},
            {'robot_pose_pub_period': 0.05},
            {'scan_pub_period': 0.1},
            {'map_pub_period': 0.2},
            {'path_pub_period': 0.1},
            {'imu_raw_data_period': 0.1},
            {'virtual_walls_pub_period': 1.0},
            {'virtual_tracks_pub_period': 1.0},
            {'basic_sensors_values_pub_period': 1.0},
            {'vel_control_topic': 'cmd_vel'},
            {'ladar_data_clockwise': True},
            {'pub_accumulate_odometry': False},
            {'robot_pose_topic': 'robot_pose'},
        ],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'slamware/odom'),  # Remap to avoid conflict
            ('map', 'slamware_map'),
            ('map_metadata', 'map_metadata'),
            ('global_plan_path', 'global_plan_path'),
        ]
    )
    
    # Static TF: slamware_map -> odom (world frame)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='slamware_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'slamware_map', 'odom']
    )
    
    # Static TF: odom -> robot_pose (connect slamware pose to odom)
    static_tf_odom_robot_pose = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_robot_pose',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'robot_pose']
    )
    
    return LaunchDescription([
        ip_address_arg,
        slamware_node,
        static_tf_map_odom,
        static_tf_odom_robot_pose,
    ])
