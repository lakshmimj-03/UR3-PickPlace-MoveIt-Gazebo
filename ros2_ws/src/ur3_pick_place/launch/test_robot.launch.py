import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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

    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([FindPackageShare('ur3_pick_place'), 'urdf', 'ur3_test.urdf.xacro'])
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo
    gazebo_world_file = PathJoinSubstitution([FindPackageShare('ur3_pick_place'), 'worlds', 'pick_place.world'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r ', gazebo_world_file, ' --verbose'],
        }.items()
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'ur3_test',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1',  # Start slightly above ground level
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([FindPackageShare('ur3_pick_place'), 'config', 'view_robot.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ]
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
            {'publish_frequency': 50.0},
            {'frame_prefix': ''}
        ]
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
            '/world/pick_place_world/model/ur3_test/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )

    # Relay to convert joint states from Gazebo to the format expected by robot_state_publisher
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay',
        arguments=[
            '/world/pick_place_world/model/ur3_test/joint_state',
            '/joint_states'
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        rviz_node,
        bridge,
        relay_node
    ])
