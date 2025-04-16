#!/usr/bin/env python3.12

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the path to the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ExecuteProcess(
                cmd=['xacro', os.path.join('/home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/urdf/ur3_fixed.urdf.xacro')],
                output='screen'
            ).stdout
        }]
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    # Robot Mover node
    robot_mover_node = ExecuteProcess(
        cmd=['python3.12', os.path.join(current_dir, 'move_robot.py')],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        robot_mover_node
    ])
