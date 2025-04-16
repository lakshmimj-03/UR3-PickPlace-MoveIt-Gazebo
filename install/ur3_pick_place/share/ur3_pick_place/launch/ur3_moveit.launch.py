import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    # Initialize Arguments
    ur_type = 'ur3'
    safety_limits = 'true'
    safety_pos_margin = '0.15'
    safety_k_position = '20'
    
    # General arguments
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')
    ur_moveit_config_package = FindPackageShare('ur_moveit_config')
    
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
            PathJoinSubstitution([ur3_pick_place_package, 'config', 'ur3.srdf']),
        ]
    )
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}

    # Kinematics
    kinematics_yaml = PathJoinSubstitution([ur3_pick_place_package, 'config', 'kinematics.yaml'])
    
    # Planning
    planning_yaml = PathJoinSubstitution([ur3_pick_place_package, 'config', 'moveit_planning.yaml'])
    
    # Joint limits
    joint_limits_yaml = PathJoinSubstitution([ur3_pick_place_package, 'config', 'joint_limits.yaml'])
    
    # Controllers
    controllers_yaml = PathJoinSubstitution([ur3_pick_place_package, 'config', 'ur3_controllers.yaml'])
    
    # MoveIt controllers
    moveit_controllers_yaml = PathJoinSubstitution([ur3_pick_place_package, 'config', 'moveit_controllers.yaml'])
    
    # Load controllers
    load_controllers = []
    for controller in ['joint_state_controller', 'arm_controller', 'gripper_controller']:
        load_controllers.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller],
                output='screen',
            )
        )

    # MoveIt demo
    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ur_moveit_config_package, '/launch', '/ur_moveit.launch.py']),
        launch_arguments={
            'ur_type': ur_type,
            'use_sim_time': 'true',
            'launch_rviz': 'true',
        }.items()
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        
        # Start MoveIt
        moveit_demo,
        
        # Load controllers after MoveIt is started
        TimerAction(
            period=5.0,
            actions=load_controllers
        ),
    ])
