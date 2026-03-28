import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share     = FindPackageShare('rby1')
    pkg_share_str = get_package_share_directory('rby1')

    # ── MoveIt 설정 파일 로드 (yaml → dict) ──────────────────────────────────
    # kinematics.yaml은 robot_description_kinematics 네임스페이스로 전달
    # moveit_controllers.yaml은 최상위 파라미터로 직접 전달
    # joint_limits.yaml은 robot_description_planning 네임스페이스로 전달
    srdf_file = os.path.join(pkg_share_str, 'config', 'moveit', 'rby1.srdf')
    kin_file  = os.path.join(pkg_share_str, 'config', 'moveit', 'kinematics.yaml')
    ctrl_file = os.path.join(pkg_share_str, 'config', 'moveit', 'moveit_controllers.yaml')
    jlim_file = os.path.join(pkg_share_str, 'config', 'moveit', 'joint_limits.yaml')

    with open(srdf_file, 'r') as f:
        srdf_content = f.read()
    with open(kin_file, 'r') as f:
        kinematics = yaml.safe_load(f)
    with open(ctrl_file, 'r') as f:
        moveit_controllers = yaml.safe_load(f)
    with open(jlim_file, 'r') as f:
        joint_limits = yaml.safe_load(f)

    return LaunchDescription([
        # ── 런치 인자 ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.3.25:50051',
            description='Robot gRPC address (host:port)',
        ),
        DeclareLaunchArgument(
            'model',
            default_value='a',
            choices=['a', 'm', 'ub'],
            description='Robot model (a / m / ub)',
        ),
        DeclareLaunchArgument(
            'end_effector',
            default_value='original',
            choices=['inspire', 'original'],
            description='End-effector type',
        ),
        DeclareLaunchArgument(
            'rate',
            default_value='50.0',
            description='Joint state publish rate [Hz]',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Launch RViz',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='false',
            description='Launch dual Lakibeam LiDAR nodes (lidar.launch.py)',
        ),

        # ── description (robot_state_publisher만 — rviz는 아래에서 직접 실행) ──
        # GroupAction(scoped=True)으로 'rviz'='false'를 이 include 범위에만 한정
        GroupAction(
            scoped=True,
            actions=[
                SetLaunchConfiguration('rviz', 'false'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_share, 'launch', 'description.launch.py'])
                    ]),
                    launch_arguments={
                        'publish_joints': 'false',
                        'end_effector':   LaunchConfiguration('end_effector'),
                    }.items(),
                ),
            ],
        ),

        # ── hardware_node ─────────────────────────────────────────────────────
        Node(
            package='rby1',
            executable='hardware_node',
            name='rby1_hardware',
            output='screen',
            parameters=[{
                'robot_address': LaunchConfiguration('robot_ip'),
                'model':         LaunchConfiguration('model'),
                'rate':          LaunchConfiguration('rate'),
            }],
        ),

        # ── mobility_node ─────────────────────────────────────────────────────
        Node(
            package='rby1',
            executable='mobility_node',
            name='rby1_mobility',
            output='screen',
            parameters=[{
                'robot_address': LaunchConfiguration('robot_ip'),
                'model':         LaunchConfiguration('model'),
            }],
        ),

        # ── wholebody_control_node ────────────────────────────────────────────
        Node(
            package='rby1',
            executable='wholebody_control_node',
            name='rby1_wholebody_controller',
            output='screen',
            parameters=[{
                'robot_address': LaunchConfiguration('robot_ip'),
            }],
        ),

        # ── LiDAR 노드 (조건부) ───────────────────────────────────────────────
        GroupAction(
            condition=IfCondition(LaunchConfiguration('use_lidar')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([pkg_share, 'launch', 'lidar.launch.py'])
                    ]),
                ),
            ],
        ),

        # ── world → base_link static TF (MoveIt 필수) ────────────────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='virtual_joint_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        ),

        # ── move_group ────────────────────────────────────────────────────────
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            sigterm_timeout='2',
            sigkill_timeout='2',
            parameters=[
                # robot_description: xacro 처리
                {
                    'robot_description': ParameterValue(
                        Command([
                            'xacro ',
                            PathJoinSubstitution([
                                pkg_share, 'urdf', 'rby1_full_ros.urdf.xacro'
                            ]),
                            ' end_effector:=',
                            LaunchConfiguration('end_effector'),
                        ]),
                        value_type=str,
                    ),
                },
                # SRDF 문자열
                {'robot_description_semantic': srdf_content},
                # IK 솔버 설정 (robot_description_kinematics 네임스페이스)
                {'robot_description_kinematics': kinematics},
                # MoveIt 컨트롤러 매니저 (최상위 파라미터로 직접)
                moveit_controllers,
                # 관절 제한 (robot_description_planning 네임스페이스)
                {'robot_description_planning': joint_limits},
                # move_group 옵션
                # deprecated 모드에서 pipeline namespace = "move_group" → 파라미터 경로: move_group.*
                {
                    'move_group': {
                        'planning_plugin': 'ompl_interface/OMPLPlanner',
                        # FixStartStateCollision: 시작 상태가 self-collision일 때 소량 jiggle
                        # FixStartStateBounds:   관절값이 한계를 미세하게 벗어날 때 클리핑
                        'request_adapters': (
                            'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints'
                        ),
                        'start_state_max_bounds_error': 0.1,
                    },
                    'allow_trajectory_execution': True,
                    'publish_robot_description_semantic': True,
                    'monitor_dynamics': False,
                    'planning_scene_monitor_options': {
                        'robot_description': 'robot_description',
                        'joint_state_topic': '/joint_states',
                    },
                },
            ],
        ),

        # ── RViz2 (MoveIt 파라미터 포함) ─────────────────────────────────────
        # rviz2도 robot_description_semantic, robot_description_kinematics가
        # 필요해야 인터랙티브 마커(goal 지정 UI)가 동작함
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share_str, 'rviz', 'bringup.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz')),
            parameters=[
                {'robot_description_semantic': srdf_content},
                {'robot_description_kinematics': kinematics},
            ],
        ),
    ])
