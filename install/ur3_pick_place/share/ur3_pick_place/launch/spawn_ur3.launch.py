import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get package paths
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([ur3_pick_place_package, 'urdf', 'ur3_with_gripper.urdf.xacro']),
            ' ', 'safety_limits:=true',
            ' ', 'safety_pos_margin:=0.15',
            ' ', 'safety_k_position:=20',
            ' ', 'name:=ur3',
            ' ', 'ur_type:=ur3',
            ' ', 'prefix:=',
            ' ', 'sim_gazebo:=true',
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

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
        package='ros_gz_sim',
        executable='create',
        name='spawn_ur3',
        arguments=[
            '-name', 'ur3',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',  # Increased height to make it more visible
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-topic', '/robot_description'
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        spawn_entity
    ])
