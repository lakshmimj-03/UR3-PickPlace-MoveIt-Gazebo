import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # Package paths
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')
    
    # RViz configuration
    rviz_config_file = PathJoinSubstitution([ur3_pick_place_package, 'config', 'view_robot.rviz'])
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    # Create and return launch description
    return LaunchDescription([
        rviz_node
    ])
