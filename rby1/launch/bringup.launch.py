"""
RBY1 bringup launch (hardware + controllers + MoveIt2)

Starts:
  1. robot_state_publisher  — TF from URDF
  2. ros2_control_node      — hardware interface + controller manager
  3. joint_state_broadcaster
  4. rby1_wholebody_controller
  5. static_virtual_joint_tf — world → base TF for MoveIt
  6. move_group             — MoveIt2 motion planning server

Usage:
  ros2 launch rby1 bringup.launch.py robot_ip:=192.168.30.1:50051 [end_effector:=inspire|original]
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
import xacro

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    pkg = get_package_share_directory('rby1')

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.30.1:50051',
        description='Robot gRPC address (host:port)',
    )
    end_effector_arg = DeclareLaunchArgument(
        'end_effector',
        default_value='inspire',
        description='End-effector type: "inspire" (RH56E2) or "original" (2-finger gripper)',
    )

    def create_nodes(context, *args, **kwargs):
        ip = context.launch_configurations['robot_ip']
        ee = context.launch_configurations['end_effector']

        # ---- URDF (hardware + full description) ----------------------------
        urdf_path = os.path.join(pkg, 'urdf', 'rby1_full_ros.urdf.xacro')
        robot_description = xacro.process_file(
            urdf_path, mappings={'robot_ip': ip, 'end_effector': ee}
        ).toxml()

        controllers_yaml = os.path.join(pkg, 'config', 'rby1_controllers.yaml')

        # ---- MoveIt2 configs -----------------------------------------------
        srdf_file = 'rby1_orig_ee.srdf' if ee == 'original' else 'rby1.srdf'
        with open(os.path.join(pkg, 'urdf', srdf_file)) as f:
            srdf_content = f.read()

        kinematics = load_yaml(os.path.join(pkg, 'config', 'moveit', 'kinematics.yaml'))
        joint_limits = load_yaml(os.path.join(pkg, 'config', 'moveit', 'joint_limits.yaml'))
        moveit_controllers = load_yaml(os.path.join(pkg, 'config', 'moveit', 'moveit_controllers.yaml'))
        # pilz cartesian limits은 robot_description_planning 아래에 병합
        pilz_limits = load_yaml(os.path.join(pkg, 'config', 'moveit', 'pilz_cartesian_limits.yaml'))
        robot_description_planning = {**joint_limits, **pilz_limits}

        planning_pipelines = {
            'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
            'default_planning_pipeline': 'ompl',
            'ompl': {
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'start_state_max_bounds_error': 0.1,
            },
            'pilz_industrial_motion_planner': {
                'planning_plugin': 'pilz_industrial_motion_planner/CommandPlanner',
            },
        }

        # ---- Nodes ---------------------------------------------------------

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

        # 3. joint_state_broadcaster
        jsb_spawner = Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
            output='screen',
        )

        # 4. rby1_wholebody_controller (2 s 후 spawn)
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

        jsb_on_start = RegisterEventHandler(
            OnProcessStart(
                target_action=ctrl_node,
                on_start=[jsb_spawner],
            )
        )

        # 5. world → base_link 가상 관절 TF (MoveIt 플래닝 기준 프레임)
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_virtual_joint_tf',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        )

        # 6. MoveIt2 move_group (4 s 후 기동 — 컨트롤러가 먼저 준비되도록)
        move_group = TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='moveit_ros_move_group',
                    executable='move_group',
                    output='screen',
                    parameters=[
                        {'robot_description': robot_description},
                        {'robot_description_semantic': srdf_content},
                        {'robot_description_kinematics': kinematics},
                        {'robot_description_planning': robot_description_planning},
                        moveit_controllers,
                        planning_pipelines,
                        {
                            'publish_robot_description_semantic': True,
                            'allow_trajectory_execution': True,
                            'capabilities': '',
                            'disable_capabilities': '',
                            'monitor_dynamics': False,
                        },
                    ],
                )
            ],
        )

        # 7. RViz
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'rviz', 'rby1.rviz')],
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': srdf_content},
                {'robot_description_kinematics': kinematics},
            ],
        )

        return [rsp_node, ctrl_node, jsb_on_start, wb_spawner, static_tf, move_group, rviz_node]

    return LaunchDescription([
        robot_ip_arg,
        end_effector_arg,
        OpaqueFunction(function=create_nodes),
    ])
