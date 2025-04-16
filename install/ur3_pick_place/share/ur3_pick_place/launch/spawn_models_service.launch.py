import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
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

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r -v 4'],
        }.items()
    )

    # Spawn test box model
    spawn_test_box = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'), 'service',
            '-s', '/world/default/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', 'sdf_filename: "/home/lachu/ros2_workspaces/ros2_ws/install/ur3_pick_place/share/ur3_pick_place/models/test_box/model.sdf", name: "test_box", pose: {position: {x: 0, y: 0, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}'
        ],
        output='screen'
    )

    # Spawn UR3 robot model
    spawn_ur3 = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'), 'service',
            '-s', '/world/default/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', 'sdf_filename: "/home/lachu/ros2_workspaces/ros2_ws/install/ur3_pick_place/share/ur3_pick_place/models/ur3/model.sdf", name: "ur3", pose: {position: {x: 1, y: 0, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}'
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        gazebo,
        # Add a delay to ensure Gazebo is fully started before spawning models
        TimerAction(
            period=5.0,
            actions=[
                spawn_test_box,
                TimerAction(
                    period=2.0,
                    actions=[spawn_ur3]
                )
            ]
        )
    ])
