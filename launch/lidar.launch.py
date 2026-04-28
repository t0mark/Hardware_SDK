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

        # ── Dual LiDAR: left/right UDP 수신 + base_link 기준 병합 → /scan ──
        Node(
            package='rby1',
            executable='dual_lidar_node',
            name='dual_lidar_node',
            output='screen',
            parameters=[{
                'configure_sensor': LaunchConfiguration('configure_sensor'),
            }],
        ),
    ])
