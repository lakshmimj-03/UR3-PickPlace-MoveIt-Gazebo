from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Get package paths
    ur_description_package = FindPackageShare("ur_description")
    ur_moveit_package = FindPackageShare("ur_moveit_config")
    ur_pick_place_package = FindPackageShare("ur3_pick_place")

    # Create the launch description
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(use_sim_time)

    # Add robot state publisher
    robot_description = PathJoinSubstitution([ur_description_package, "urdf", "ur3.urdf.xacro"])
    ld.add_action(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            output='screen'
        )
    )

    # Add joint state publisher
    ld.add_action(
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        )
    )

    # Add MoveIt
    moveit_config = PathJoinSubstitution([ur_moveit_package, "config", "ur3.srdf"])
    ld.add_action(
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'semantic_robot_description': moveit_config,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'allow_trajectory_execution': True,
                'publish_monitored_planning_scene': True
            }]
        )
    )

    # Add RViz
    rviz_config = PathJoinSubstitution([ur_pick_place_package, "config", "view_robot.rviz"])
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    )

    # Add pick and place node
    ld.add_action(
        Node(
            package='ur3_pick_place',
            executable='pick_place_node',
            name='pick_place_node',
            output='screen'
        )
    )

    return ld