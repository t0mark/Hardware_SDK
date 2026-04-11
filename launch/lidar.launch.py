from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ── 런치 인자 ─────────────────────────────────────────────────────
        # configure_sensor: bool로 C++ 노드에 직접 전달 (LaunchConfiguration OK)
        DeclareLaunchArgument(
            'configure_sensor',
            default_value='true',
            description='Send HTTP config to sensor at startup (scanfreq, laser_enable, scan_range)',
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
                'port':             '2367',
                'frame_id':         'lidar_left',
                'output_topic':     'scan_left',
                'inverted':         False,
                'angle_offset':     0,
                # bool: LaunchConfiguration('true'/'false') → YAML true/false → C++ bool OK
                'configure_sensor': LaunchConfiguration('configure_sensor'),
                # int64_t: Python int 리터럴로 전달 (LaunchConfiguration은 string → type mismatch)
                'scanfreq':         30,
                'laser_enable':     True,
                'scan_range_start': 45,
                'scan_range_stop':  315,
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
                'port':             '2368',
                'frame_id':         'lidar_right',
                'output_topic':     'scan_right',
                'inverted':         False,
                'angle_offset':     0,
                'configure_sensor': LaunchConfiguration('configure_sensor'),
                'scanfreq':         30,
                'laser_enable':     True,
                'scan_range_start': 45,
                'scan_range_stop':  315,
            }],
        ),

        # ── LaserScan 병합 (scan_left + scan_right → scan) ────────────────
        Node(
            package='rby1',
            executable='lidar_merge_node',
            name='lidar_merge_node',
            output='screen',
            parameters=[{
                'scan0_topic':    'scan_left',
                'scan1_topic':    'scan_right',
                'output_topic':   'scan',
                'target_frame':   'base_link',
                'tf_timeout_sec': 0.2,
            }],
        ),
    ])
