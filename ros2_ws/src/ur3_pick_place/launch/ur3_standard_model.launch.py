import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    ur_description_package = FindPackageShare('ur_description')
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')
    
    # Initialize Arguments
    ur_type = 'ur3'
    safety_limits = 'true'
    safety_pos_margin = '0.15'
    safety_k_position = '20'
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            PathJoinSubstitution([ur_description_package, 'urdf', 'ur.urdf.xacro']),
            ' ', 'safety_limits:=', safety_limits,
            ' ', 'safety_pos_margin:=', safety_pos_margin,
            ' ', 'safety_k_position:=', safety_k_position,
            ' ', 'name:=ur3',
            ' ', 'ur_type:=', ur_type,
        ]
    )
    
    # RViz config file
    rviz_config = PathJoinSubstitution([ur3_pick_place_package, 'config', 'view_robot.rviz'])
    
    # Pick and place script path
    pick_place_script = os.path.join(
        get_package_share_directory('ur3_pick_place'),
        'scripts',
        'simple_pick_place.py'
    )
    
    # Check if the script exists
    if not os.path.exists(pick_place_script):
        # Try the source directory
        pick_place_script = os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'scripts',
            'simple_pick_place.py'
        )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Joint State Publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'rate': 30}]
    )
    
    # Joint State Publisher GUI node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # Pick and Place node
    pick_place_node = Node(
        package='ur3_pick_place',
        executable=pick_place_script,
        name='pick_place_node',
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        pick_place_node
    ])
