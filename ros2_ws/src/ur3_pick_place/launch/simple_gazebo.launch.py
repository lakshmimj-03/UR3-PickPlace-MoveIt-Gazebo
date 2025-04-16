import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import LaunchConfigurationEquals
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

    # Gazebo
    gazebo_world_file = PathJoinSubstitution([ur3_pick_place_package, 'worlds', 'pick_place.world'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r ', gazebo_world_file, ' --verbose'],
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
        ],
        remappings=[
            ('/robot_description', '/robot_description')
        ]
    )

    # Robot description publisher
    robot_description_publisher = Node(
        package='topic_tools',
        executable='relay',
        name='robot_description_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['/robot_description', '/robot_description']
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', PathJoinSubstitution([ur3_pick_place_package, 'urdf', 'ur3_with_gripper.urdf']),
            '-name', 'ur3',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',  # Adjust height as needed
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    )

    # Bridge between ROS and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[
            {'use_sim_time': True},
        ],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/pick_place_world/model/ur3/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/ur3/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/ur3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/ur3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/pick_place_world/model/ur3/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            '/world/pick_place_world/model/ur3/pose_static@geometry_msgs/msg/PoseStatic[gz.msgs.Pose',
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        gazebo,
        robot_state_publisher,
        robot_description_publisher,
        # Add a delay to ensure robot description is published before spawning
        TimerAction(
            period=5.0,
            actions=[]
        ),
        spawn_entity,
        bridge
    ])
