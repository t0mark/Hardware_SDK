import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                             IncludeLaunchDescription, OpaqueFunction,
                             SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share     = FindPackageShare('rby1')
    pkg_share_str = get_package_share_directory('rby1')

    # ── MoveIt 설정 파일 로드 (high 모드에서만 사용) ──────────────────────────
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

    # ── 모드별 노드 구성 (OpaqueFunction으로 런타임 if문 사용) ────────────────
    def launch_setup(context, *args, **kwargs):
        mode  = context.launch_configurations['control_mode']
        ip    = context.launch_configurations['robot_ip']
        model = context.launch_configurations['model']
        ee    = context.launch_configurations['end_effector']

        nodes = []

        # high / medium: cmd_vel_node (바퀴 속도 명령)
        if mode in ('high', 'medium'):
            nodes.append(Node(
                package='rby1',
                executable='control_cmd_vel_node',
                namespace='rby1',
                name='control_cmd_vel_node',
                output='screen',
                parameters=[{
                    'robot_address': ip,
                    'model':         model,
                }],
            ))

        # high: moveit_node + move_group (MoveIt 궤적 제어)
        if mode == 'high':
            nodes.append(Node(
                package='rby1',
                executable='control_moveit_node',
                name='control_moveit_node',
                output='screen',
                parameters=[{
                    'robot_address': ip,
                }],
            ))

            xacro_file = os.path.join(
                get_package_share_directory('rby1'), 'urdf', 'rby1_full_ros.urdf.xacro')

            nodes.append(Node(
                package='moveit_ros_move_group',
                executable='move_group',
                name='move_group',
                output='screen',
                sigterm_timeout='2',
                sigkill_timeout='2',
                parameters=[
                    {
                        'robot_description': ParameterValue(
                            Command(['xacro ', xacro_file, ' end_effector:=', ee]),
                            value_type=str,
                        ),
                    },
                    {'robot_description_semantic': srdf_content},
                    {'robot_description_kinematics': kinematics},
                    moveit_controllers,
                    {'robot_description_planning': joint_limits},
                    {
                        'move_group': {
                            'planning_plugin': 'ompl_interface/OMPLPlanner',
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
            ))

        # medium / low: joint_control_node (직접 관절 위치/속도 명령)
        if mode in ('medium', 'low'):
            nodes.append(Node(
                package='rby1',
                executable='control_joint_node',
                namespace='rby1',
                name='control_joint_node',
                output='screen',
                parameters=[{
                    'robot_address': ip,
                    'model':         model,
                    'control_base':  mode == 'low',
                    'cmd_timeout':   0.5,
                }],
            ))

        return nodes

    return LaunchDescription([
        # ── 런치 인자 ─────────────────────────────────────────────────────────
        DeclareLaunchArgument(
            'control_mode',
            default_value='high',
            choices=['high', 'medium', 'low'],
            description='Control mode: high=MoveIt, medium=cmd_vel+direct joints, low=full direct',
        ),
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
        DeclareLaunchArgument(
            'head_0_p_gain', default_value='200', description='head_0 P gain',
        ),
        DeclareLaunchArgument(
            'head_0_i_gain', default_value='0', description='head_0 I gain',
        ),
        DeclareLaunchArgument(
            'head_0_d_gain', default_value='8000', description='head_0 D gain',
        ),
        DeclareLaunchArgument(
            'head_1_p_gain', default_value='200', description='head_1 P gain',
        ),
        DeclareLaunchArgument(
            'head_1_i_gain', default_value='0', description='head_1 I gain',
        ),
        DeclareLaunchArgument(
            'head_1_d_gain', default_value='8000', description='head_1 D gain',
        ),

        # ── description (robot_state_publisher) ───────────────────────────────
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
            executable='init_hardware_node',
            name='init_hardware_node',
            output='screen',
            parameters=[{
                'robot_address': LaunchConfiguration('robot_ip'),
                'model':         LaunchConfiguration('model'),
                'rate':          LaunchConfiguration('rate'),
            }],
        ),


        # ── home pose 노드 (일회성) ───────────────────────────────────────────────
        Node(
            package='rby1',
            executable='control_home_pose_node',
            name='control_home_pose_node',
            output='screen',
            parameters=[{
                'robot_ip':      LaunchConfiguration('robot_ip'),
                'model':         LaunchConfiguration('model'),
                'minimum_time':  10.0,
            }],
        ),

        # ── head gain 노드 (조건부, 일회성) ──────────────────────────────────────
        Node(
            package='rby1',
            executable='control_head_gain_node',
            name='control_head_gain_node',
            output='screen',
            parameters=[{
                'robot_ip':    LaunchConfiguration('robot_ip'),
                'model':       LaunchConfiguration('model'),
                'head_0_p_gain': LaunchConfiguration('head_0_p_gain'),
                'head_0_i_gain': LaunchConfiguration('head_0_i_gain'),
                'head_0_d_gain': LaunchConfiguration('head_0_d_gain'),
                'head_1_p_gain': LaunchConfiguration('head_1_p_gain'),
                'head_1_i_gain': LaunchConfiguration('head_1_i_gain'),
                'head_1_d_gain': LaunchConfiguration('head_1_d_gain'),
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

        # ── RViz2 ─────────────────────────────────────────────────────────────
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

        # ── 모드별 노드 (control_mode에 따라 동적 구성) ───────────────────────
        OpaqueFunction(function=launch_setup),
    ])
