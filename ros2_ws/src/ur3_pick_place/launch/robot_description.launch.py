from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return None

def generate_launch_description():
    ur_type = "ur3"
    
    # Paths
    ur_description_package = FindPackageShare('ur_description')
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')
    
    # Joint limits and kinematics
    joint_limit_params = PathJoinSubstitution([ur_description_package, "config", ur_type, "joint_limits.yaml"])
    kinematics_params = PathJoinSubstitution([ur_description_package, "config", ur_type, "default_kinematics.yaml"])
    physical_params = PathJoinSubstitution([ur_description_package, "config", ur_type, "physical_parameters.yaml"])
    visual_params = PathJoinSubstitution([ur_description_package, "config", ur_type, "visual_parameters.yaml"])
    initial_positions_file = PathJoinSubstitution([ur_description_package, "config", "initial_positions.yaml"])

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ',
            PathJoinSubstitution([ur3_pick_place_package, "urdf", "ur3_with_gripper.urdf.xacro"]),
            ' ',
            'name:=ur3',
            ' ',
            'ur_type:=ur3',
            ' ',
            'initial_positions_file:=',
            initial_positions_file,
            ' ',
            'joint_limit_params:=',
            joint_limit_params,
            ' ',
            'kinematics_params:=',
            kinematics_params,
            ' ',
            'physical_params:=',
            physical_params,
            ' ',
            'visual_params:=',
            visual_params,
            ' ',
            'safety_limits:=true',
            ' ',
            'safety_pos_margin:=0.15',
            ' ',
            'safety_k_position:=20',
            ' ',
            'use_mock_hardware:=true',
            ' ',
            'mock_sensor_commands:=true',
        ]
    )

    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint state publisher GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([ur3_pick_place_package, "config", "view_robot.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])