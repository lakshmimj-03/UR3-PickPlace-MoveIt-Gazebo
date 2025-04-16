import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
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
    
    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args': ['-r -v 4 ', PathJoinSubstitution([ur3_pick_place_package, 'worlds', 'pick_place.world'])],
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
    
    # Spawn UR3 robot model
    spawn_ur3 = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'), 'service', 
            '-s', '/world/default/create', 
            '--reqtype', 'gz.msgs.EntityFactory', 
            '--reptype', 'gz.msgs.Boolean', 
            '--timeout', '1000', 
            '--req', 'sdf_filename: "/home/lachu/ros2_workspaces/ros2_ws/install/ur3_pick_place/share/ur3_pick_place/models/ur3/model.sdf", name: "ur3", pose: {position: {x: 0, y: 0, z: 0.5}, orientation: {x: 0, y: 0, z: 0, w: 1}}'
        ],
        output='screen'
    )
    
    # Spawn red cube
    spawn_red_cube = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'), 'service', 
            '-s', '/world/default/create', 
            '--reqtype', 'gz.msgs.EntityFactory', 
            '--reptype', 'gz.msgs.Boolean', 
            '--timeout', '1000', 
            '--req', 'sdf: "<sdf version=\'1.6\'><model name=\'red_cube\'><static>false</static><link name=\'link\'><inertial><mass>0.1</mass><inertia><ixx>0.000166667</ixx><iyy>0.000166667</iyy><izz>0.000166667</izz></inertia></inertial><collision name=\'collision\'><geometry><box><size>0.05 0.05 0.05</size></box></geometry></collision><visual name=\'visual\'><geometry><box><size>0.05 0.05 0.05</size></box></geometry><material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>", name: "red_cube", pose: {position: {x: 0.5, y: 0.0, z: 0.425}, orientation: {x: 0, y: 0, z: 0, w: 1}}'
        ],
        output='screen'
    )
    
    # Spawn blue cylinder
    spawn_blue_cylinder = ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'), 'service', 
            '-s', '/world/default/create', 
            '--reqtype', 'gz.msgs.EntityFactory', 
            '--reptype', 'gz.msgs.Boolean', 
            '--timeout', '1000', 
            '--req', 'sdf: "<sdf version=\'1.6\'><model name=\'blue_cylinder\'><static>false</static><link name=\'link\'><inertial><mass>0.1</mass><inertia><ixx>0.000166667</ixx><iyy>0.000166667</iyy><izz>0.000166667</izz></inertia></inertial><collision name=\'collision\'><geometry><cylinder><radius>0.025</radius><length>0.05</length></cylinder></geometry></collision><visual name=\'visual\'><geometry><cylinder><radius>0.025</radius><length>0.05</length></cylinder></geometry><material><ambient>0 0 1 1</ambient><diffuse>0 0 1 1</diffuse></material></visual></link></model></sdf>", name: "blue_cylinder", pose: {position: {x: 0.5, y: 0.2, z: 0.425}, orientation: {x: 0, y: 0, z: 0, w: 1}}'
        ],
        output='screen'
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
    
    # Load controllers
    load_controllers = []
    for controller in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']:
        load_controllers.append(
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[controller],
                output='screen',
            )
        )
    
    # Pick and place node
    pick_place_node = Node(
        package='ur3_pick_place',
        executable='pick_place_node',
        name='pick_place_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'use_perception': True}
        ]
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        
        # Start Gazebo
        gazebo,
        
        # Start robot state publisher
        robot_state_publisher,
        
        # Spawn models after Gazebo is started
        TimerAction(
            period=5.0,
            actions=[
                spawn_ur3,
                spawn_red_cube,
                spawn_blue_cylinder
            ]
        ),
        
        # Start MoveIt after models are spawned
        TimerAction(
            period=10.0,
            actions=[moveit_demo]
        ),
        
        # Load controllers after MoveIt is started
        TimerAction(
            period=15.0,
            actions=load_controllers
        ),
        
        # Start pick and place node after controllers are loaded
        TimerAction(
            period=20.0,
            actions=[pick_place_node]
        )
    ])
