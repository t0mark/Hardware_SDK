import os
import yaml
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def cleanup_microstrain_process(context, *args, **kwargs):
    """Kill any existing microstrain processes before starting new one"""
    try:
        subprocess.run(['pkill', '-9', '-f', 'microstrain_inertial_driver_node'], 
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Wait a moment for the port to be released
        subprocess.run(['sleep', '0.5'])
    except Exception:
        pass
    return []


def generate_launch_description():
    # Declare launch arguments
    arm_arg = DeclareLaunchArgument(
        'arm',
        default_value='true',
        description='Enable Kinova arm launch'
    )
    
    moveit_arg = DeclareLaunchArgument(
        'moveit',
        default_value='true',
        description='Enable MoveIt for Kinova arm'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz'
    )
    
    # Get launch configurations
    arm = LaunchConfiguration('arm')
    moveit = LaunchConfiguration('moveit')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # Get package directories
    microstrain_inertial_driver_dir = get_package_share_directory('microstrain_inertial_driver')
    kortex_bringup_dir = get_package_share_directory('kortex_bringup')
    roas2_bringup_dir = get_package_share_directory('roas2_bringup')
    slamware_ros_sdk_dir = get_package_share_directory('slamware_ros_sdk')
    
    # Microstrain sensor parameters
    microstrain_default_params_file = os.path.join(
        microstrain_inertial_driver_dir,
        'microstrain_inertial_driver_common',
        'config',
        'params.yml'
    )
    microstrain_empty_params_file = os.path.join(
        microstrain_inertial_driver_dir,
        'config',
        'empty.yml'
    )
    
    # Microstrain node with auto-restart (respawn)
    microstrain_node = Node(
        package='microstrain_inertial_driver',
        executable='microstrain_inertial_driver_node',
        name='microstrain_inertial_driver',
        namespace='/',
        parameters=[
            yaml.safe_load(open(microstrain_default_params_file, 'r')),
            microstrain_empty_params_file,
            {'debug': False}
        ],
        respawn=True,
        respawn_delay=2.0,
        output='screen'
    )
    
    # Include ROAS MoveIt launch file (when arm=true and moveit=true)
    # Delayed by 3 seconds to allow microstrain to initialize first
    roas_moveit_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(roas2_bringup_dir, 'launch', 'robot.launch.py')
                ),
                launch_arguments={
                    'use_internal_bus_gripper_comm': 'true',
                    'launch_rviz': launch_rviz,
                }.items(),
                condition=IfCondition(
                    PythonExpression(['"', arm, '" == "true" and "', moveit, '" == "true"'])
                )
            )
        ]
    )
    
    # Include Kinova Gen3 6DOF basic launch file (when arm=true and moveit=false)
    # Delayed by 3 seconds to allow microstrain to initialize first
    gen3_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(kortex_bringup_dir, 'launch', 'gen3.launch.py')
                ),
                launch_arguments={
                    'robot_type': 'gen3',
                    'dof': '6',
                    'gripper': 'robotiq_2f_85',
                    'use_internal_bus_gripper_comm': 'true',
                    'launch_rviz': launch_rviz,
                }.items(),
                condition=IfCondition(
                    PythonExpression(['"', arm, '" == "true" and "', moveit, '" == "false"'])
                )
            )
        ]
    )
    
    # Cleanup function to kill any existing microstrain processes
    cleanup_action = OpaqueFunction(function=cleanup_microstrain_process)
    
    # Launch Slamware with custom config (TF disabled)
    slamware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(roas2_bringup_dir, 'launch', 'slamware_custom.launch.py')
        )
    )
    
    # Launch Kinova Vision with 10 second delay
    kinova_vision_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    get_package_share_directory('kinova_vision'),
                    '/launch/kinova_vision.launch.py'
                ]),
                launch_arguments={
                    'launch_depth': 'false',
                }.items()
            )
        ]
    )
    
    return LaunchDescription([
        cleanup_action,
        arm_arg,
        moveit_arg,
        launch_rviz_arg,
        microstrain_node,
        slamware_launch,
        roas_moveit_launch,
        gen3_launch,
        kinova_vision_launch,
    ])
