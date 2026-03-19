from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── 런치 인자 ─────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'configure_sensor',
            default_value='true',
            description='Send HTTP config to sensor at startup (scanfreq, laser_enable, scan_range)',
        ),
        DeclareLaunchArgument(
            'scanfreq',
            default_value='30',
            description='Scan frequency in Hz (10 / 20 / 25 / 30)',
        ),
        DeclareLaunchArgument(
            'scan_range_start',
            default_value='45',
            description='Scan range start angle in degrees (45–315)',
        ),
        DeclareLaunchArgument(
            'scan_range_stop',
            default_value='315',
            description='Scan range stop angle in degrees (45–315, must be > start)',
        ),

        # ── 좌측 LiDAR (192.168.30.10) ───────────────────────────────────
        Node(
            package='rby1',
            executable='lidar_node',
            name='lidar_left',
            output='screen',
            parameters=[{
                'sensorip':         '192.168.30.10',
                'hostip':           '0.0.0.0',
                'port':             '2368',
                'frame_id':         'lidar_left',
                'output_topic':     'scan_left',
                'inverted':         False,
                'angle_offset':     0,
                'configure_sensor': LaunchConfiguration('configure_sensor'),
                'scanfreq':         LaunchConfiguration('scanfreq'),
                'laser_enable':     'true',
                'scan_range_start': LaunchConfiguration('scan_range_start'),
                'scan_range_stop':  LaunchConfiguration('scan_range_stop'),
            }],
        ),

        # ── 우측 LiDAR (192.168.30.11) ───────────────────────────────────
        Node(
            package='rby1',
            executable='lidar_node',
            name='lidar_right',
            output='screen',
            parameters=[{
                'sensorip':         '192.168.30.11',
                'hostip':           '0.0.0.0',
                'port':             '2369',
                'frame_id':         'lidar_right',
                'output_topic':     'scan_right',
                'inverted':         False,
                'angle_offset':     0,
                'configure_sensor': LaunchConfiguration('configure_sensor'),
                'scanfreq':         LaunchConfiguration('scanfreq'),
                'laser_enable':     'true',
                'scan_range_start': LaunchConfiguration('scan_range_start'),
                'scan_range_stop':  LaunchConfiguration('scan_range_stop'),
            }],
        ),
    ])
