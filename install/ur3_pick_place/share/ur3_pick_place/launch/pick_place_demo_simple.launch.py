import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package paths
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')

    # Launch Gazebo simulation first
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur3_pick_place_package, 'launch', 'gazebo_simulation.launch.py'])
        ])
    )

    # Pick and place node (start after a delay to ensure simulation is running)
    pick_place_node = TimerAction(
        period=10.0,  # Wait 10 seconds for simulation to start
        actions=[Node(
            package='ur3_pick_place',
            executable='pick_place_node',
            name='pick_place_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )]
    )

    # Create and return launch description
    return LaunchDescription([
        gazebo_simulation,
        pick_place_node
    ])
