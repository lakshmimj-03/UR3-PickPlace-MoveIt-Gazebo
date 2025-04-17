# Pick and Place Application for UR3 Robot using ROS 2 Jazzy, MoveIt 2, and Gazebo Simulation

This repository contains a complete ROS 2 workspace implementing a pick and place application using a Universal Robots UR3 robot arm. The application leverages MoveIt 2 for motion planning and Gazebo for simulation, demonstrating a full robotic manipulation pipeline.

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
  - [Running the Simulation](#running-the-simulation)
  - [Running the Pick and Place Demo](#running-the-pick-and-place-demo)
  - [Manual Triggering](#manual-triggering)
- [Package Structure](#package-structure)
- [Configuration](#configuration)
  - [Robot Configuration](#robot-configuration)
  - [Motion Planning Configuration](#motion-planning-configuration)
  - [Controller Configuration](#controller-configuration)
- [Implementation Details](#implementation-details)
  - [Pick and Place Pipeline](#pick-and-place-pipeline)
  - [Object Detection](#object-detection)
  - [Gripper Control](#gripper-control)
  - [Motion Planning](#motion-planning)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Overview

This project demonstrates a complete pick and place application using a UR3 robot arm in a simulated environment. The application showcases the integration of various ROS 2 components to create a functional robotic manipulation system.

## System Architecture

The system consists of the following main components:

1. **UR3 Robot Model**: A Universal Robots UR3 robot arm with an attached gripper
2. **Gazebo Simulation**: Provides physics simulation and visualization
3. **MoveIt 2**: Handles motion planning and collision avoidance
4. **ROS 2 Controllers**: Manage joint and gripper control
5. **Pick and Place Node**: Implements the application logic

The components interact through ROS 2 topics and services, with the pick and place node orchestrating the overall operation.

## Features

- Complete pick and place pipeline with a UR3 robot arm
- Motion planning with MoveIt 2 for collision-free trajectories
- Simulated object detection and manipulation
- Gripper control for grasping objects
- Integration with Gazebo for realistic physics simulation
- Configurable robot and environment parameters
- Multiple launch configurations for different use cases
- Lightweight visualization-only simulation with smooth robot movements
- Single-command scripts for easy launching
- Cubic ease-in/ease-out interpolation for natural robot motion

## Requirements

### Software Requirements

- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 Jazzy Jalisco
- Gazebo Garden (via ros_gz packages)
- MoveIt 2
- Universal Robots ROS 2 packages

### ROS 2 Packages

- `ur_description`: UR robot description
- `ur_moveit_config`: MoveIt configuration for UR robots
- `ros_gz_sim`: ROS 2 - Gazebo integration
- `ros_gz_bridge`: Bridge between ROS 2 and Gazebo
- `moveit_ros_move_group`: MoveIt move group functionality
- `controller_manager`: ROS 2 controller management
- `joint_state_broadcaster`: Joint state broadcasting
- `joint_trajectory_controller`: Joint trajectory control

## Installation

### Prerequisites

1. Install ROS 2 Jazzy Jalisco following the [official instructions](https://docs.ros.org/en/jazzy/Installation.html)

2. Install Gazebo Garden and ROS 2 integration packages:
   ```bash
   sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-interfaces
   ```

3. Install MoveIt 2:
   ```bash
   sudo apt install ros-jazzy-moveit
   ```

4. Install Universal Robots ROS 2 packages:
   ```bash
   sudo apt install ros-jazzy-ur-description ros-jazzy-ur-moveit-config
   ```

### Workspace Setup

1. Create a ROS 2 workspace (if not already created):
   ```bash
   mkdir -p ~/ros2_workspaces/ros2_ws/src
   cd ~/ros2_workspaces/ros2_ws
   ```

2. Clone this repository into your workspace:
   ```bash
   cd ~/ros2_workspaces/ros2_ws/src
   git clone https://github.com/lakshmimj-03/Pick-and-Place-Application-for-UR3-Robot-using-ROS-2-Jazzy-MoveIt-2-and-Gazebo-Simulation.git ur3_pick_place
   ```

3. Clone required dependencies (if not installed via apt):
   ```bash
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
   git clone https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git
   ```

4. Build the workspace:
   ```bash
   cd ~/ros2_workspaces/ros2_ws
   colcon build --symlink-install
   ```

5. Source the workspace:
   ```bash
   source ~/ros2_workspaces/ros2_ws/install/setup.bash
   ```

## Usage

### Running the Simulation

#### Option 1: Using the single-command script (Recommended)

To launch the complete simulation with a single command:

```bash
cd ~/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation.sh
```

This script will launch all necessary components:
- RViz for visualization
- Robot state publisher for publishing the robot's state
- Robot mover script for controlling the robot

Press Ctrl+C to stop all components.

#### Option 2: Using the tmux script

If you have tmux installed, you can use the tmux script to run all components in a single terminal window:

```bash
cd ~/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation_tmux.sh
```

This will open a tmux session with three panes:
- Top-left: RViz
- Bottom-left: Robot state publisher
- Right: Robot mover script

Press Ctrl+B, then D to detach from the tmux session (this will also stop all components).

If you don't have tmux installed, you can install it with:

```bash
sudo apt-get install tmux
```

#### Option 3: Running components manually

If you prefer to run the components manually, you can use the following commands:

```bash
# Terminal 1: Start RViz
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz

# Terminal 2: Start the robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)"

# Terminal 3: Run the robot mover script
python3.12 src/ur3_pick_place/scripts/move_robot.py
```

#### Option 4: Using Gazebo (Full Simulation)

To launch the Gazebo simulation with the UR3 robot:

```bash
ros2 launch ur3_pick_place gazebo_simulation.launch.py
```

This will:
- Start Gazebo with the pick_place world
- Spawn the UR3 robot with a gripper
- Load necessary controllers
- Start RViz for visualization
- Initialize MoveIt for motion planning

### Running the Pick and Place Demo

To run the complete pick and place demo:

```bash
ros2 launch ur3_pick_place test_pick_place.launch.py
```

This launch file:
1. Starts the Gazebo simulation
2. Loads the UR3 robot with a gripper
3. Initializes MoveIt for motion planning
4. Starts the pick and place node
5. Automatically triggers the pick and place operation

### Manual Triggering

You can manually trigger the pick and place operation by publishing to the `/trigger_pick_place` topic:

```bash
ros2 topic pub --once /trigger_pick_place std_msgs/msg/Bool "data: true"
```

### Additional Launch Files

- **`pick_place_demo.launch.py`**: Launches the pick and place demo with more configuration options
- **`ur3_moveit.launch.py`**: Launches only the MoveIt configuration for the UR3 robot
- **`spawn_objects.launch.py`**: Spawns objects in the Gazebo world for manipulation

## Package Structure

```
ur3_pick_place/
├── config/                 # Configuration files
├── include/               # C++ header files
├── launch/                # Launch files
├── meshes/                # 3D models for visualization and collision
├── models/                # Gazebo model definitions
├── scripts/               # Python scripts
├── src/                   # C++ source files
├── ur3_pick_place/        # Python package
├── urdf/                  # Robot description files
└── worlds/                # Gazebo world definitions
```

## Configuration

### Robot Configuration

The robot configuration is defined in the following files:
- `urdf/ur3_with_gripper.urdf.xacro`: Main robot description combining UR3 and gripper
- `urdf/gripper.urdf.xacro`: Gripper description
- `config/ur3.srdf`: Semantic robot description defining planning groups and named positions

### Motion Planning Configuration

Motion planning parameters are defined in:
- `config/moveit_planning.yaml`: MoveIt planning parameters
- `config/kinematics.yaml`: Kinematics solver parameters
- `config/joint_limits.yaml`: Joint limits for the robot

### Controller Configuration

Controller parameters are defined in:
- `config/ur3_controllers.yaml`: Controller configuration for the robot and gripper

## Implementation Details

### Pick and Place Pipeline

The pick and place pipeline consists of the following steps:

1. **Initialization**: Move to a ready position and initialize the system
2. **Object Detection**: Detect objects in the workspace (simulated in this implementation)
3. **Pick Operation**:
   - Plan and move to a pre-grasp position above the object
   - Open the gripper
   - Move down to the grasp position
   - Close the gripper to grasp the object
   - Lift the object
4. **Place Operation**:
   - Plan and move to a pre-place position
   - Move down to the place position
   - Open the gripper to release the object
   - Move back to a safe position
5. **Completion**: Return to the home position

### Lightweight Visualization Simulation

In addition to the full Gazebo simulation, this package includes a lightweight visualization-only simulation that demonstrates the pick and place operation without the overhead of a full physics simulation. This implementation:

1. Uses direct joint state publishing for robot control
2. Implements smooth motion using cubic ease-in/ease-out interpolation
3. Visualizes objects and their attachment to the gripper
4. Provides a complete pick and place sequence
5. Can be launched with a single command

The lightweight simulation is ideal for quick demonstrations and testing of robot movements without requiring the full Gazebo environment.

### Object Detection

In the simulation, object detection is simulated by using known object positions from the Gazebo world. The positions are defined in the `pick_place_node.cpp` file:

```cpp
// For simulation, use hard-coded poses from our Gazebo world
if (object_id == "red_cube") {
    pose.position.x = 0.5;
    pose.position.y = 0.1;
    pose.position.z = 0.435;
    pose.orientation.w = 1.0;
    return true;
}
```

In a real-world scenario, this would be replaced with computer vision-based object detection using cameras and perception algorithms.

### Gripper Control

The gripper is controlled using a GripperCommand message published to the `/gripper_controller/gripper_cmd` topic. The control function is implemented in the `pick_place_node.cpp` file:

```cpp
void PickPlaceNode::controlGripper(double position)
{
    auto gripper_command = std::make_shared<control_msgs::msg::GripperCommand>();
    gripper_command->position = position;  // 0.0 (closed) to 1.0 (open)
    gripper_command->max_effort = 50.0;

    gripper_command_publisher_->publish(*gripper_command);
    rclcpp::sleep_for(std::chrono::seconds(1));
}
```

### Motion Planning

Motion planning is handled by MoveIt 2, which provides collision-free trajectories for the robot. The planning is done using the MoveGroupInterface in the `pick_place_node.cpp` file:

```cpp
// Move to pre-grasp pose
move_group_->setPoseTarget(pre_grasp_pose);
bool success = (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS);
```

For more precise movements, Cartesian paths are used:

```cpp
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(pre_grasp_pose);
waypoints.push_back(grasp_pose);

moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group_->computeCartesianPath(waypoints, 0.01, trajectory);
```

## Troubleshooting

### Common Issues

1. **Gazebo not starting properly**:
   - Make sure you have installed Gazebo Garden and the ROS 2 integration packages
   - Check if the world file exists and is correctly formatted

2. **Robot not spawning in Gazebo**:
   - Verify that the robot description is being published correctly
   - Check the spawn entity node arguments

3. **Controllers not loading**:
   - Ensure the controller configuration is correct
   - Check if the controller manager is running

4. **MoveIt planning errors**:
   - Verify that the SRDF file is correctly configured
   - Check if the kinematics solver is properly set up

5. **Pick and place operation failing**:
   - Check if the object positions are correctly defined
   - Verify that the gripper is functioning properly

### Debugging Tips

1. Use RViz to visualize the robot state and planned trajectories
2. Check ROS 2 topics to verify that messages are being published correctly:
   ```bash
   ros2 topic list
   ros2 topic echo /joint_states
   ```
3. Inspect the controller status:
   ```bash
   ros2 control list_controllers
   ```
4. Enable verbose logging for more detailed information:
   ```bash
   ros2 launch ur3_pick_place gazebo_simulation.launch.py --log-level debug
   ```

## Contributing

Contributions to this project are welcome! Here's how you can contribute:

1. Fork the repository
2. Create a new branch for your feature or bugfix
3. Make your changes
4. Submit a pull request

Please ensure your code follows the ROS 2 coding standards and includes appropriate documentation.

## License

This package is licensed under the Apache License 2.0 - see the LICENSE file for details.
