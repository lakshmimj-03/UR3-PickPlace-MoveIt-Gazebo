import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get package paths
    pkg_share = get_package_share_directory('ur3_pick_place')
    
    # Spawn robot using Python script
    spawn_robot_node = Node(
        package='ur3_pick_place',
        executable='scripts/spawn_robot.py',
        name='spawn_robot',
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        spawn_robot_node
    ])
