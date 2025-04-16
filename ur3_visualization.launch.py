#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get the path to the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            os.path.join('/home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/urdf/ur3_fixed.urdf.xacro')
        ]
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': False}]
    )
    
    # Joint State Publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'rate': 30, 'use_sim_time': False}]
    )
    
    # RViz2 node with our custom config
    rviz_config_file = os.path.join(current_dir, 'ur3_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )
    
    # Robot Mover node
    robot_mover_node = ExecuteProcess(
        cmd=['python3.12', os.path.join(current_dir, 'move_robot.py')],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        robot_mover_node
    ])
