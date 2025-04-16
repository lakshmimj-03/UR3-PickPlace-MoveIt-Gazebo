import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
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
    ur_moveit_package = FindPackageShare('ur_moveit_config')
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

    # Get SRDF via xacro
    robot_description_semantic_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([ur3_pick_place_package, 'config', 'ur3.srdf'])
        ]
    )
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # MoveIt configurations
    moveit_config_package = FindPackageShare('ur3_pick_place')
    planning_yaml = PathJoinSubstitution([moveit_config_package, 'config', 'moveit_planning.yaml'])

    # Load kinematics config
    kinematics_yaml_path = os.path.join(get_package_share_directory('ur3_pick_place'), 'config', 'kinematics.yaml')
    with open(kinematics_yaml_path, 'r') as f:
        kinematics_yaml = yaml.safe_load(f)
        kinematics_params = kinematics_yaml['/**']['ros__parameters']['move_group']['robot_description_kinematics']

    # Gazebo
    gazebo_world_file = PathJoinSubstitution([ur3_pick_place_package, 'worlds', 'pick_place.world'])
    gazebo_params_file = PathJoinSubstitution([ur3_pick_place_package, 'config', 'gazebo_params.yaml'])
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
            '-name', 'ur3',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'  # Start at ground level
        ],
        output='screen'
    )

    # Since we're not using gazebo_ros2_control, we don't need to load controllers
    # We'll use ros_gz_sim for control instead
    # The controllers will be handled through the parameter_bridge

    # Placeholder nodes for compatibility with the rest of the launch file
    load_joint_state_controller = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawner_joint_state_broadcaster',
        output='screen'
    )

    load_arm_controller = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawner_arm_controller',
        output='screen'
    )

    load_gripper_controller = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawner_gripper_controller',
        output='screen'
    )

    # Add forward position controller for direct joint control
    load_forward_position_controller = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawner_forward_position_controller',
        output='screen'
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([ur3_pick_place_package, 'config', 'view_robot.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'use_sim_time': True}
        ]
    )

    # MoveIt
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_yaml,
            {'publish_robot_description': True},
            {'publish_robot_description_semantic': True},
            {'use_sim_time': True},
            {'allow_trajectory_execution': True},
            {'capabilities': ''},
            {'disable_capabilities': ''},
            {'publish_planning_scene': True},
            {'publish_geometry_updates': True},
            {'publish_state_updates': True},
            {'publish_transforms_updates': True},
            {'robot_description_kinematics': kinematics_params}
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
        ],
        remappings=[
            ('/robot_description', '/robot_description')
        ]
    )

    # Robot description publisher
    robot_description_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_description_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True},
            {'publish_frequency': 50.0},
            {'frame_prefix': ''}
        ]
    )

    # Delay start of robot control nodes
    delay_rviz_after_gazebo = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[rviz_node]
        )
    )

    delay_controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                load_joint_state_controller,
                load_arm_controller,
                load_gripper_controller,
                load_forward_position_controller
            ]
        )
    )

    # Pick and place node
    pick_place_node = Node(
        package='ur3_pick_place',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_yaml,
            {'use_sim_time': True}
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
            '/world/pick_place_world/model/ur3/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Add more bridge topics as needed
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # Command topics
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
            # Removed problematic topics that might be causing the bridge to crash
            # '/arm_controller/joint_trajectory]trajectory_msgs/msg/JointTrajectory',
            # '/gripper_controller/gripper_cmd]control_msgs/msg/GripperCommand'
        ],
        output='screen'
    )

    # Relay to convert joint states from Gazebo to the format expected by robot_state_publisher
    relay_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay',
        arguments=[
            '/world/pick_place_world/model/ur3/joint_state',
            '/joint_states'
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        gazebo,
        robot_description_publisher,
        robot_state_publisher,
        # Spawn the robot after the robot description is published
        spawn_entity,
        # Add bridge and relay after spawn to ensure proper connection
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[bridge, relay_node]
            )
        ),
        # Event handlers for sequencing
        delay_rviz_after_gazebo,
        delay_controllers_after_spawn,
        # MoveIt node
        move_group_node,
        # Add the pick_place_node at the end after controllers are loaded
        pick_place_node
    ])
