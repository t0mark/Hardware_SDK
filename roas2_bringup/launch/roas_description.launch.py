#!/usr/bin/env python3
# Copyright (c) 2024 ROAS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper",
            default_value="robotiq_2f_85",
            description="Gripper type to attach to the arm",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "dof",
            default_value="6",
            description="Degrees of freedom of the arm",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_internal_bus_gripper_comm",
            default_value="true",
            description="Use internal bus for gripper communication",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="IP address of the Kinova arm",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_state",
            default_value="true",
            description="Start robot_state_publisher node",
        )
    )

    # Initialize arguments
    gripper = LaunchConfiguration("gripper")
    dof = LaunchConfiguration("dof")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")
    robot_ip = LaunchConfiguration("robot_ip")
    publish_robot_state = LaunchConfiguration("publish_robot_state")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("roas2_bringup"), "urdf", "roas.urdf.xacro"]
            ),
            " ",
            "gripper:=",
            gripper,
            " ",
            "dof:=",
            dof,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "use_internal_bus_gripper_comm:=",
            use_internal_bus_gripper_comm,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "sim_isaac:=false",
            " ",
            "isaac_joint_commands:=/isaac_joint_commands",
            " ",
            "isaac_joint_states:=/isaac_joint_states",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher node
    # This will subscribe to joint_states and publish TF transforms
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
        ],
        condition=IfCondition(publish_robot_state),
    )

    nodes_to_start = [
        robot_state_publisher_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)

