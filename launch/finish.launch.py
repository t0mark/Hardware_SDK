from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── 런치 인자 ──────────────────────────────────────────────────────────────
    declare_robot_ip = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.3.25:50051',
        description='Robot gRPC address (host:port)',
    )
    declare_model = DeclareLaunchArgument(
        'model',
        default_value='a',
        choices=['a', 'm', 'ub'],
        description='Robot model (a / m / ub)',
    )
    declare_minimum_time = DeclareLaunchArgument(
        'minimum_time',
        default_value='10.0',
        description='Minimum trajectory time for zero pose [s]',
    )

    # ── zero_pose_node ─────────────────────────────────────────────────────────
    zero_pose_node = Node(
        package='rby1',
        executable='zero_pose_node',
        name='zero_pose_node',
        output='screen',
        parameters=[{
            'robot_ip':      LaunchConfiguration('robot_ip'),
            'model':         LaunchConfiguration('model'),
            'minimum_time':  LaunchConfiguration('minimum_time'),
        }],
    )

    # ── turn_off_node (zero_pose_node 종료 후 순차 실행) ───────────────────────
    turn_off_node = Node(
        package='rby1',
        executable='turn_off_node',
        name='turn_off_node',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'),
            'model':    LaunchConfiguration('model'),
        }],
    )

    turn_off_on_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=zero_pose_node,
            on_exit=[turn_off_node],
        )
    )

    return LaunchDescription([
        declare_robot_ip,
        declare_model,
        declare_minimum_time,
        zero_pose_node,
        turn_off_on_exit,
    ])
