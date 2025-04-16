cmake_minimum_required(VERSION 3.8)
project(move_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(move_robot_node move_robot.cpp)
ament_target_dependencies(move_robot_node rclcpp sensor_msgs)

install(TARGETS
  move_robot_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
