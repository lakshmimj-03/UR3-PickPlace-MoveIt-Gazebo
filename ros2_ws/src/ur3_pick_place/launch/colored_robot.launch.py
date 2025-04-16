#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Get the launch directory
    pkg_share = get_package_share_directory('ur3_pick_place')

    # Robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "ur3_colored.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", "ur3_colored.rviz"])

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # Trajectory visualizer
    trajectory_visualizer = Node(
        name="trajectory_visualizer",
        namespace="",
        package="ur3_pick_place",
        executable="trajectory_visualizer.py",
        parameters=[],
        remappings=[],
        output="screen",
        prefix=["python3 ", PathJoinSubstitution([pkg_share, "scripts/"])],
    )

    # Visual enhancer
    visual_enhancer = Node(
        name="visual_enhancer",
        namespace="",
        package="ur3_pick_place",
        executable="visual_enhancer.py",
        parameters=[],
        remappings=[],
        output="screen",
        prefix=["python3 ", PathJoinSubstitution([pkg_share, "scripts/"])],
    )

    # UR3 Color enhancer
    ur3_color_enhancer = Node(
        name="ur3_color_enhancer",
        namespace="",
        package="ur3_pick_place",
        executable="ur3_color_enhancer.py",
        parameters=[],
        remappings=[],
        output="screen",
        prefix=["python3 ", PathJoinSubstitution([pkg_share, "scripts/"])],
    )

    # Launch description
    return LaunchDescription(
        [
            robot_state_publisher_node,
            joint_state_publisher_node,
            rviz_node,
            trajectory_visualizer,
            visual_enhancer,
            ur3_color_enhancer,
        ]
    )
