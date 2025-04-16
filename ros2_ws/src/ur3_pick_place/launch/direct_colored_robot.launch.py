#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('ur3_pick_place')
    
    # URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur3_direct_colored.urdf.xacro')
    
    # Generate the URDF content
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'ur3_colored.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Launch the scripts directly
    trajectory_visualizer = ExecuteProcess(
        cmd=['python3', os.path.join('/home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/scripts', 'trajectory_visualizer.py')],
        output='screen'
    )
    
    visual_enhancer = ExecuteProcess(
        cmd=['python3', os.path.join('/home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/scripts', 'visual_enhancer.py')],
        output='screen'
    )
    
    ur3_color_enhancer = ExecuteProcess(
        cmd=['python3', os.path.join('/home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/scripts', 'ur3_color_enhancer.py')],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
        trajectory_visualizer,
        visual_enhancer,
        ur3_color_enhancer
    ])
