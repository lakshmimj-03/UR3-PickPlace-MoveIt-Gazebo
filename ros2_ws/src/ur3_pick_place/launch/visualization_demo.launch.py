#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation (Gazebo) clock if true",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Get package directories
    pick_place_package = FindPackageShare("ur3_pick_place")
    ur_description_package = FindPackageShare("ur_description")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [pick_place_package, "urdf", "ur3_robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Create launch description
    ld = LaunchDescription()

    # Add declared arguments
    for arg in declared_arguments:
        ld.add_action(arg)

    # Start robot state publisher
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        )
    )

    # Start joint state publisher GUI
    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        )
    )

    # Start RViz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pick_place_package, "config", "view_robot.rviz"])],
        )
    )

    # Visualization Node
    ld.add_action(
        Node(
            package="ur3_pick_place",
            executable="visualization_node",
            name="visualization_node",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )
    )

    return ld
