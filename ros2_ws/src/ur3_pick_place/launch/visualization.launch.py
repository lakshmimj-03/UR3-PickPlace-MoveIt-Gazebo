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
        parameters=[{'use_sim_time': True}]
    )

    # Get the path to the scripts
    trajectory_visualizer_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'scripts',
        'trajectory_visualizer.py'
    )

    ur3_color_enhancer_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
        'scripts',
        'ur3_color_enhancer.py'
    )

    # Trajectory visualizer node
    trajectory_visualizer_node = Node(
        package='ur3_pick_place',
        executable=trajectory_visualizer_path,
        name='trajectory_visualizer',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # UR3 Color Enhancer node
    ur3_color_enhancer_node = Node(
        package='ur3_pick_place',
        executable=ur3_color_enhancer_path,
        name='ur3_color_enhancer',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        rviz_node,
        trajectory_visualizer_node,
        ur3_color_enhancer_node
    ])
