from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur3_pick_place_package = FindPackageShare('ur3_pick_place')

    # Include robot description launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur3_pick_place_package, 'launch', 'robot_description.launch.py'])
        ])
    )

    return LaunchDescription([
        robot_description_launch
    ])