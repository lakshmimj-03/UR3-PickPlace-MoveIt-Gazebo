import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ur3_pick_place')
    
    # Check if we're in the installed space, if not use the source directory
    if not os.path.exists(os.path.join(pkg_dir, 'urdf', 'simple_ur3_test.urdf')):
        pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    
    # URDF file path
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_ur3_test.urdf')
    
    # RViz config file
    rviz_config = os.path.join(pkg_dir, 'config', 'view_robot.rviz')
    
    # Read the URDF file content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
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
    
    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
