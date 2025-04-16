import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    # Package paths
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')

    # Get URDF via xacro
    urdf_file_path = os.path.join(
        get_package_share_directory('ur3_pick_place'),
        'urdf',
        'ur3_fixed.urdf.xacro'
    )

    # Check if the file exists
    if not os.path.exists(urdf_file_path):
        # Try the source directory
        urdf_file_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'urdf',
            'ur3_fixed.urdf.xacro'
        )

    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            urdf_file_path
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # RViz configuration
    rviz_config_file = PathJoinSubstitution([ur3_pick_place_package, 'config', 'view_robot.rviz'])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': False},
            {'publish_frequency': 50.0},
            {'frame_prefix': ''}
        ]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'rate': 50}
        ]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[
            {'use_sim_time': False}
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            {'use_sim_time': False}
        ],
        output='screen'
    )

    # Create and return launch description
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
