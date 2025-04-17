from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Get required paths
    ur_description_package = FindPackageShare("ur_description")
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

    # Create launch description
    ld = LaunchDescription()

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

    # RViz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pick_place_package, "config", "view_robot.rviz"])],
        )
    )

    # Direct Robot Control Node
    ld.add_action(
        Node(
            package="ur3_pick_place",
            executable="direct_robot_control",
            name="direct_robot_control",
            output="screen",
        )
    )

    return ld
