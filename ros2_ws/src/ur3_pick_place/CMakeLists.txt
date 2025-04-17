cmake_minimum_required(VERSION 3.8)
project(ur3_pick_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(control_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
)

# Declare and build the pick_place_node executable
add_executable(pick_place_node
  src/pick_place_node.cpp
)

ament_target_dependencies(pick_place_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_msgs
  tf2_ros
  tf2_geometry_msgs
  control_msgs
)

# Declare and build the marker_publisher executable
add_executable(marker_publisher
  src/marker_publisher.cpp
)

ament_target_dependencies(marker_publisher
  rclcpp
  geometry_msgs
  visualization_msgs
)

# Install executables
install(TARGETS
  pick_place_node
  marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories while preserving structure
install(DIRECTORY
  urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY
  meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install scripts
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Export includes
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  tf2_geometry_msgs
  control_msgs
)

ament_package()