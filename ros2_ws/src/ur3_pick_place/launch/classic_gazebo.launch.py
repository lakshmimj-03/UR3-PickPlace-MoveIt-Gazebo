import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Initialize Arguments
    ur_type = 'ur3'
    safety_limits = 'true'
    safety_pos_margin = '0.15'
    safety_k_position = '20'
    
    # General arguments
    description_package = FindPackageShare('ur_description')
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([ur3_pick_place_package, 'urdf', 'ur3_with_gripper.urdf.xacro']),
            ' ', 'safety_limits:=', safety_limits,
            ' ', 'safety_pos_margin:=', safety_pos_margin,
            ' ', 'safety_k_position:=', safety_k_position,
            ' ', 'name:=ur3',
            ' ', 'ur_type:=', ur_type,
            ' ', 'prefix:=',
            ' ', 'sim_gazebo:=true',
            ' ', 'simulation_controllers:=',
            PathJoinSubstitution([ur3_pick_place_package, 'config', 'ur3_controllers.yaml'])
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo classic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']),
        launch_arguments={
            'world': PathJoinSubstitution([ur3_pick_place_package, 'worlds', 'pick_place.world']),
            'verbose': 'true',
        }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True},
            {'publish_frequency': 30.0}
        ]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ur3',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',  # Adjust height as needed
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        gazebo,
        robot_state_publisher,
        # Add a delay to ensure robot description is published before spawning
        TimerAction(
            period=5.0,
            actions=[spawn_entity]
        )
    ])
