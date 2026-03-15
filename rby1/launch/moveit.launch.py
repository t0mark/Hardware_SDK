"""
RBY1 MoveIt2 standalone launch (no moveit_configs_utils dependency)

bringup.launch.py 없이 단독 실행 시 사용 (시뮬레이션/시각화 목적).
하드웨어와 함께 쓸 때는 bringup.launch.py 를 사용할 것.

Usage:
  ros2 launch rby1 moveit.launch.py [use_rviz:=false] [end_effector:=inspire|original]
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _moveit_params(pkg, end_effector='inspire'):
    """Load and return all MoveIt2 parameter dicts."""
    import xacro
    urdf_path = os.path.join(pkg, 'urdf', 'rby1_full_ros.urdf.xacro')
    robot_description_content = xacro.process_file(
        urdf_path, mappings={'end_effector': end_effector}
    ).toxml()

    srdf_file = 'rby1_orig_ee.srdf' if end_effector == 'original' else 'rby1.srdf'
    with open(os.path.join(pkg, 'urdf', srdf_file)) as f:
        srdf_content = f.read()

    kinematics = load_yaml(os.path.join(pkg, 'config', 'moveit', 'kinematics.yaml'))
    joint_limits = load_yaml(os.path.join(pkg, 'config', 'moveit', 'joint_limits.yaml'))
    moveit_controllers = load_yaml(os.path.join(pkg, 'config', 'moveit', 'moveit_controllers.yaml'))
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

    move_group_capabilities = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'capabilities': '',
        'disable_capabilities': '',
        'monitor_dynamics': False,
    }

    return (
        robot_description_content,
        srdf_content,
        [
            {'robot_description': robot_description_content},
            {'robot_description_semantic': srdf_content},
            {'robot_description_kinematics': kinematics},
            {'robot_description_planning': robot_description_planning},
            moveit_controllers,
            planning_pipelines,
            move_group_capabilities,
        ],
    )


def generate_launch_description():

    def create_nodes(context, *args, **kwargs):
        pkg = get_package_share_directory('rby1')
        ee = context.launch_configurations.get('end_effector', 'inspire')
        robot_description_content, srdf_content, move_group_params = _moveit_params(pkg, ee)
        kinematics = load_yaml(os.path.join(pkg, 'config', 'moveit', 'kinematics.yaml'))

        # world → base_link 가상 관절 TF
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_virtual_joint_tf',
            arguments=['--frame-id', 'world', '--child-frame-id', 'base_link'],
        )

        # robot_state_publisher (standalone이므로 RSP 직접 기동)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        )

        # move_group
        move_group = Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=move_group_params,
        )

        nodes = [static_tf, rsp, move_group]

        if context.launch_configurations.get('use_rviz', 'true') == 'true':
            nodes.append(Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='log',
                arguments=['-d', os.path.join(pkg, 'rviz', 'rby1.rviz')],
                parameters=[
                    {'robot_description': robot_description_content},
                    {'robot_description_semantic': srdf_content},
                    {'robot_description_kinematics': kinematics},
                ],
            ))

        return nodes

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'end_effector',
            default_value='inspire',
            description='End-effector type: "inspire" (RH56E2) or "original" (2-finger gripper)',
        ),
        OpaqueFunction(function=create_nodes),
    ])
