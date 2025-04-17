from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
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

    # Joint State Publisher GUI
    ld.add_action(
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        )
    )

    # RViz
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", PathJoinSubstitution([pick_place_package, "config", "markers.rviz"])],
        )
    )

    # Create a node to publish markers
    ld.add_action(
        Node(
            package="ros2",
            executable="topic",
            name="marker_publisher",
            output="screen",
            arguments=["pub", "/visualization_marker_array", "visualization_msgs/msg/MarkerArray", "--once", "{markers: [{header: {frame_id: 'base_link'}, ns: 'objects', id: 1, type: 1, action: 0, pose: {position: {x: 0.4, y: 0.0, z: 0.05}, orientation: {w: 1.0}}, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {r: 1.0, g: 0.0, b: 0.0, a: 1.0}}, {header: {frame_id: 'base_link'}, ns: 'objects', id: 2, type: 2, action: 0, pose: {position: {x: 0.4, y: 0.2, z: 0.05}, orientation: {w: 1.0}}, scale: {x: 0.1, y: 0.1, z: 0.1}, color: {r: 0.0, g: 0.0, b: 1.0, a: 1.0}}, {header: {frame_id: 'base_link'}, ns: 'objects', id: 3, type: 1, action: 0, pose: {position: {x: 0.4, y: -0.2, z: 0.05}, orientation: {w: 1.0}}, scale: {x: 0.2, y: 0.2, z: 0.02}, color: {r: 0.0, g: 1.0, b: 0.0, a: 1.0}}]}"],
        )
    )

    return ld
