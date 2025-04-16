import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    ur3_pick_place_src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'), ' ',
            os.path.join(ur3_pick_place_src_dir, 'urdf', 'ur3_fixed.urdf.xacro')
        ]
    )
    
    # Path to the smooth joint publisher script
    smooth_joint_publisher_path = os.path.join(
        ur3_pick_place_src_dir, 
        'scripts', 
        'smooth_joint_publisher.py'
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    # Smooth Joint Publisher node
    smooth_joint_publisher_node = Node(
        package='ur3_pick_place',
        executable=smooth_joint_publisher_path,
        name='smooth_joint_publisher',
        output='screen'
    )
    
    # RViz2 node with specific configuration
    rviz_config_path = os.path.join(ur3_pick_place_src_dir, 'config', 'view_robot.rviz')
    if not os.path.exists(rviz_config_path):
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(get_package_share_directory('rviz2'), 'default.rviz')]
        )
    else:
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    
    # Return the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        smooth_joint_publisher_node,
        rviz_node
    ])
