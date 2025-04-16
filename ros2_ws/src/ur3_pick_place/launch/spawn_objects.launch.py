from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node to spawn objects in Gazebo
    spawn_objects_node = Node(
        package='ur3_pick_place',
        executable='spawn_objects_node',
        name='spawn_objects',
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        spawn_objects_node
    ])
