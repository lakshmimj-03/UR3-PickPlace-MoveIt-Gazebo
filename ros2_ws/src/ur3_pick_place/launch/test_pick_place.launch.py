from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get package paths
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')

    # Include the Gazebo simulation launch file
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur3_pick_place_package, 'launch', 'gazebo_simulation.launch.py'])
        ])
    )

    # Create a node to trigger the pick and place operation
    test_pick_place_node = Node(
        package='ur3_pick_place',
        executable='test_pick_place.py',
        name='test_pick_place_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        gazebo_simulation,
        # Wait for the simulation to be fully started before triggering the pick and place
        TimerAction(
            period=30.0,  # Wait 30 seconds for Gazebo and MoveIt to fully start
            actions=[test_pick_place_node]
        )
    ])
