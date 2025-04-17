from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading YAML file: {absolute_file_path}")
        return {}

def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur3",
            description="Type/series of used UR robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enable safety limits",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="Safety margin for position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="Safety K-position",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Launch Gazebo simulation with the robot.",
        )
    )

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    use_gazebo = LaunchConfiguration("use_gazebo")

    # Get required paths
    ur_description_package = FindPackageShare("ur_description")
    ur_moveit_config_package = FindPackageShare("ur_moveit_config")
    pick_place_package = FindPackageShare("ur3_pick_place")

    # Robot description
    xacro_file = os.path.join(ur_description_package.find("ur_description"), "urdf", "ur.urdf.xacro")

    # Generate URDF using xacro
    robot_description_content = Command([
        'xacro', ' ', xacro_file,
        ' name:=', 'ur3_robot',
        ' ur_type:=', 'ur3',
        ' use_fake_hardware:=', 'true',
        ' safety_limits:=', 'true',
        ' safety_pos_margin:=', '0.15',
        ' safety_k_position:=', '20'
    ])

    # Generate SRDF using xacro
    srdf_xacro_file = os.path.join(ur_moveit_config_package.find("ur_moveit_config"), "srdf", "ur.srdf.xacro")
    robot_description_semantic_content = Command([
        'xacro', ' ', srdf_xacro_file,
        ' name:=', 'ur3_robot'
    ])

    # Load kinematics.yaml
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")

    # Load joint limits
    joint_limits_yaml = load_yaml("ur3_pick_place", "config/joint_limits.yaml")

    # Load trajectory execution parameters
    moveit_controllers_yaml = load_yaml("ur_moveit_config", "config/moveit_controllers.yaml")

    # Load planning parameters
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")

    # Create MoveIt configuration dictionary
    moveit_config = {
        "robot_description": robot_description_content,
        "robot_description_semantic": robot_description_semantic_content,
        "robot_description_kinematics": kinematics_yaml,
        "robot_description_planning": joint_limits_yaml,
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "ompl": ompl_planning_yaml,
        "planning_pipelines": "ompl",
        "use_sim_time": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "publish_planning_scene": True,
        "allowed_planning_strategies": "ompl",
        "moveit_manage_controllers": True
    }

    # Create launch description
    ld = LaunchDescription(declared_arguments)

    # Robot State Publisher
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description_content}],
        )
    )

    # Joint State Publisher (non-GUI version for MoveIt control)
    ld.add_action(
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            parameters=[{"source_list": []}],  # Empty source list means it will use MoveIt's joint states
        )
    )

    # MoveIt
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config],
    )

    # Add MoveIt node
    ld.add_action(move_group_node)

    # RViz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pick_place_package, "config", "view_robot.rviz"])],
            parameters=[moveit_config],
        )
    )

    # Pick and Place Demo Node
    ld.add_action(
        Node(
            package="ur3_pick_place",
            executable="pick_place_demo",
            name="pick_place_demo",
            output="screen",
            parameters=[moveit_config, {"use_sim_time": True}],
        )
    )

    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'
            ])
        ]),
        condition=IfCondition(use_gazebo)
    )
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'ur3_robot',
        ],
        output='screen',
        condition=IfCondition(use_gazebo)
    )

    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity)

    return ld