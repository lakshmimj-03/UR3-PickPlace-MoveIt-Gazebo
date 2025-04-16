#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    ur3_pick_place_src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            os.path.join(ur3_pick_place_src_dir, 'ros2_ws/src/ur3_pick_place/urdf/ur3_fixed.urdf.xacro')
        ]
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Pick and Place Publisher node
    pick_place_publisher_path = os.path.join(
        os.getcwd(),
        'pick_place_publisher.py'
    )
    
    pick_place_publisher_node = Node(
        package='ur3_pick_place',
        executable=pick_place_publisher_path,
        name='pick_place_publisher',
        output='screen'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        pick_place_publisher_node,
        rviz_node
    ])
