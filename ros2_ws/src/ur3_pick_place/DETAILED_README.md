# UR3 Robot Pick and Place Simulation: Detailed Documentation

## Table of Contents

1. [Introduction](#introduction)
2. [Theoretical Background](#theoretical-background)
   - [Robot Kinematics](#robot-kinematics)
   - [Trajectory Generation](#trajectory-generation)
   - [Cubic Ease-In/Ease-Out Interpolation](#cubic-ease-inease-out-interpolation)
   - [Robot Visualization](#robot-visualization)
   - [Object Manipulation](#object-manipulation)
3. [System Architecture](#system-architecture)
   - [Component Overview](#component-overview)
   - [Data Flow](#data-flow)
   - [ROS 2 Topics](#ros-2-topics)
4. [Implementation Details](#implementation-details)
   - [Robot State Publisher](#robot-state-publisher)
   - [Joint State Publisher](#joint-state-publisher)
   - [Visualization Markers](#visualization-markers)
   - [Pick and Place Sequence](#pick-and-place-sequence)
   - [Gripper Simulation](#gripper-simulation)
5. [File Structure](#file-structure)
   - [Python Scripts](#python-scripts)
   - [Launch Files](#launch-files)
   - [Configuration Files](#configuration-files)
   - [Shell Scripts](#shell-scripts)
6. [Installation and Setup](#installation-and-setup)
   - [Prerequisites](#prerequisites)
   - [ROS 2 Workspace Setup](#ros-2-workspace-setup)
   - [Package Dependencies](#package-dependencies)
7. [Running the Simulation](#running-the-simulation)
   - [Single-Command Launch](#single-command-launch)
   - [Tmux-Based Launch](#tmux-based-launch)
   - [Manual Component Launch](#manual-component-launch)
8. [Customization](#customization)
   - [Modifying Robot Movements](#modifying-robot-movements)
   - [Changing Object Positions](#changing-object-positions)
   - [Adjusting Timing Parameters](#adjusting-timing-parameters)
9. [Troubleshooting](#troubleshooting)
   - [Common Issues](#common-issues)
   - [Debugging Techniques](#debugging-techniques)
10. [Advanced Topics](#advanced-topics)
    - [Integration with MoveIt](#integration-with-moveit)
    - [Adding Collision Detection](#adding-collision-detection)
    - [Real Robot Integration](#real-robot-integration)
11. [References](#references)

## Introduction

This package provides a comprehensive simulation of a Universal Robots UR3 robot performing pick and place operations. The simulation focuses on smooth, realistic robot movements and proper visualization of object manipulation without requiring a full physics simulation environment like Gazebo.

The implementation demonstrates:
- Direct control of robot joint positions
- Smooth trajectory generation with cubic ease-in/ease-out interpolation
- Visualization of objects and their attachment to the robot gripper
- Complete pick and place sequence with proper object handling
- Simple and efficient launching mechanisms

This detailed documentation explains the theoretical concepts, implementation details, and practical usage of the simulation.

## Theoretical Background

### Robot Kinematics

The UR3 robot is a 6-degree-of-freedom (6-DOF) robotic arm with revolute joints. The robot's kinematics define how the joint positions relate to the end-effector position and orientation in 3D space.

In this simulation, we use forward kinematics to visualize the robot's state based on joint positions. The joint positions are directly controlled to move the robot through a predefined sequence of waypoints.

The UR3 robot's joint configuration consists of:
1. `shoulder_pan_joint`: Rotates the robot base
2. `shoulder_lift_joint`: Controls the shoulder lift
3. `elbow_joint`: Controls the elbow position
4. `wrist_1_joint`: First wrist joint
5. `wrist_2_joint`: Second wrist joint
6. `wrist_3_joint`: Third wrist joint (end-effector rotation)

### Trajectory Generation

Trajectory generation is the process of creating a smooth path for the robot to follow between waypoints. In this simulation, we use linear interpolation between joint positions with a cubic ease-in/ease-out function to create natural-looking movements.

The trajectory generation process:
1. Define waypoints for the robot to move through
2. For each pair of consecutive waypoints:
   - Calculate the difference in joint positions
   - Generate a smooth trajectory between them using interpolation
   - Apply the cubic ease-in/ease-out function to create acceleration and deceleration

### Cubic Ease-In/Ease-Out Interpolation

Cubic ease-in/ease-out interpolation is a technique used to create smooth transitions between positions. It provides gradual acceleration at the beginning of the movement and gradual deceleration at the end, resulting in more natural and realistic robot motion.

The cubic ease-in/ease-out function is defined as:

```
f(t) = t² * (3 - 2t)
```

Where:
- `t` is the normalized time (0 to 1)
- `f(t)` is the interpolation factor

This function has the following properties:
- `f(0) = 0`: Starts at 0
- `f(1) = 1`: Ends at 1
- `f'(0) = 0`: Zero velocity at the start
- `f'(1) = 0`: Zero velocity at the end

These properties ensure that the robot starts and stops smoothly, without abrupt changes in velocity.

### Robot Visualization

The robot is visualized using the URDF (Unified Robot Description Format) model of the UR3 robot. The URDF defines the robot's links, joints, and visual properties.

The visualization process:
1. The robot state publisher reads the URDF model
2. Joint positions are published to the `/joint_states` topic
3. The robot state publisher calculates the transforms between links
4. RViz visualizes the robot based on these transforms

### Object Manipulation

Object manipulation is simulated by:
1. Visualizing objects (red cube, blue cylinder) using RViz markers
2. Attaching objects to the robot gripper when picked up
3. Detaching objects and updating their positions when placed

The attachment process changes the reference frame of the object from the world frame to the end-effector frame, making the object move with the robot.

## System Architecture

### Component Overview

The simulation consists of the following main components:

1. **Robot State Publisher**: Publishes the robot's state based on the URDF model and joint positions
2. **RViz**: Visualizes the robot and objects
3. **Robot Mover Script**: Controls the robot's joint positions and simulates object manipulation

### Data Flow

The data flow in the simulation is as follows:

1. The robot mover script generates joint positions based on the predefined waypoints
2. Joint positions are published to the `/joint_states` topic
3. The robot state publisher calculates transforms based on the joint positions
4. RViz visualizes the robot based on the transforms
5. The robot mover script also publishes visualization markers for objects
6. RViz visualizes the objects based on the markers

### ROS 2 Topics

The simulation uses the following ROS 2 topics:

- `/joint_states`: Joint positions for the robot
- `/visualization_marker_array`: Markers for visualizing objects
- `/tf` and `/tf_static`: Transforms between robot links

## Implementation Details

### Robot State Publisher

The robot state publisher is a ROS 2 node that:
1. Reads the robot's URDF model
2. Subscribes to the `/joint_states` topic
3. Calculates transforms between robot links based on joint positions
4. Publishes these transforms to the `/tf` and `/tf_static` topics

### Joint State Publisher

The joint state publisher functionality is implemented in the robot mover script. It:
1. Defines the joint names for the UR3 robot
2. Generates joint positions based on the predefined waypoints
3. Publishes these positions to the `/joint_states` topic

### Visualization Markers

Visualization markers are used to represent objects in the simulation. The robot mover script:
1. Creates markers for the red cube, blue cylinder, and place location
2. Updates marker positions based on the simulation state
3. Changes marker reference frames when objects are attached to or detached from the gripper
4. Publishes markers to the `/visualization_marker_array` topic

### Pick and Place Sequence

The pick and place sequence is implemented in the robot mover script. It consists of the following steps:

1. **Move to Home Position**: Start at a safe position
2. **Pick Red Cube**:
   - Move above the red cube
   - Move down to the cube
   - Close the gripper (attach the cube to the gripper)
   - Lift the cube
3. **Place Red Cube**:
   - Move above the place location
   - Move down to the place location
   - Open the gripper (detach the cube)
   - Move up from the place location
4. **Pick Blue Cylinder**:
   - Move above the blue cylinder
   - Move down to the cylinder
   - Close the gripper (attach the cylinder to the gripper)
   - Lift the cylinder
5. **Place Blue Cylinder**:
   - Move above the place location
   - Move down to the place location
   - Open the gripper (detach the cylinder)
   - Move up from the place location
6. **Return to Home Position**: End at a safe position

### Gripper Simulation

The gripper is simulated using visualization markers. The gripper state is represented by:
- A value between 0.0 (closed) and 1.0 (open)
- Visual markers that change size based on the gripper state
- Attachment/detachment of objects based on the gripper state

## File Structure

### Python Scripts

- **`move_robot.py`**: Main script for controlling the robot and simulating object manipulation
  - `RobotMover` class: Implements the robot control and object manipulation
  - `move_joints` method: Moves the robot joints with smooth interpolation
  - `pick_place_sequence` method: Implements the pick and place sequence
  - `visualize_objects` method: Creates and updates visualization markers

### Launch Files

- **`move_robot.launch.py`**: Launch file for running the simulation components manually

### Configuration Files

- **`view_robot.rviz`**: RViz configuration file for visualizing the robot and objects

### Shell Scripts

- **`run_simulation.sh`**: Shell script for launching all components with a single command
- **`run_simulation_tmux.sh`**: Shell script for launching all components in a tmux session

## Installation and Setup

### Prerequisites

- Ubuntu 22.04 (or compatible Linux distribution)
- ROS 2 Jazzy Jalisco
- Python 3.12 (recommended) or Python 3
- UR3 robot description package

### ROS 2 Workspace Setup

1. Create a ROS 2 workspace (if not already created):
   ```bash
   mkdir -p ~/ros2_workspaces/ros2_ws/src
   cd ~/ros2_workspaces/ros2_ws
   ```

2. Clone the repository into your workspace:
   ```bash
   cd ~/ros2_workspaces/ros2_ws/src
   git clone <repository-url> ur3_pick_place
   ```

3. Build the workspace:
   ```bash
   cd ~/ros2_workspaces/ros2_ws
   colcon build --symlink-install
   ```

4. Source the workspace:
   ```bash
   source ~/ros2_workspaces/ros2_ws/install/setup.bash
   ```

### Package Dependencies

Install the required ROS 2 packages:

```bash
sudo apt install ros-jazzy-ur-description ros-jazzy-robot-state-publisher ros-jazzy-rviz2
```

## Running the Simulation

### Single-Command Launch

To launch the simulation with a single command:

```bash
cd ~/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation.sh
```

This script:
1. Sources the ROS 2 setup
2. Launches RViz with the appropriate configuration
3. Starts the robot state publisher with the UR3 URDF
4. Runs the robot mover script

Press Ctrl+C to stop all components.

### Tmux-Based Launch

To launch the simulation in a tmux session:

```bash
cd ~/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation_tmux.sh
```

This script:
1. Creates a tmux session with three panes
2. Runs each component in a separate pane
3. Allows you to see all outputs in a single window

Press Ctrl+B, then D to detach from the tmux session (this will also stop all components).

### Manual Component Launch

To launch the components manually:

```bash
# Terminal 1: Start RViz
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz

# Terminal 2: Start the robot state publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)"

# Terminal 3: Run the robot mover script
python3.12 src/ur3_pick_place/scripts/move_robot.py
```

## Customization

### Modifying Robot Movements

To modify the robot's movements, edit the `pick_place_sequence` method in the `move_robot.py` script. The method contains a sequence of `move_joints` calls with target joint positions.

Example of modifying a movement:

```python
# Original movement
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0])

# Modified movement (different shoulder pan angle)
self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
```

### Changing Object Positions

To change the positions of objects, modify the object pose definitions in the `__init__` method of the `RobotMover` class:

```python
# Original red cube position
self.red_cube_pose = Pose(
    position=Point(x=0.4, y=0.0, z=0.05),
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
)

# Modified red cube position
self.red_cube_pose = Pose(
    position=Point(x=0.5, y=0.1, z=0.05),
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
)
```

### Adjusting Timing Parameters

To adjust the timing of movements, modify the `duration` parameter in the `move_joints` method calls:

```python
# Original movement (2.0 seconds)
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0], duration=2.0)

# Modified movement (slower, 3.0 seconds)
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0], duration=3.0)
```

You can also adjust the interpolation steps in the `move_joints` method to change the smoothness of movements:

```python
# Original interpolation steps
self.interpolation_steps = 50

# Modified interpolation steps (smoother movement)
self.interpolation_steps = 100
```

## Troubleshooting

### Common Issues

1. **RViz not showing the robot**:
   - Make sure the robot state publisher is running
   - Check that the URDF model is being loaded correctly
   - Verify that joint states are being published

2. **Objects not appearing**:
   - Check that the visualization markers are being published
   - Verify that the marker reference frames exist
   - Make sure the marker scales and positions are appropriate

3. **Robot not moving**:
   - Check that the robot mover script is running
   - Verify that joint states are being published
   - Check for errors in the terminal output

### Debugging Techniques

1. **Check ROS 2 topics**:
   ```bash
   ros2 topic list
   ros2 topic echo /joint_states
   ros2 topic echo /visualization_marker_array
   ```

2. **Check transforms**:
   ```bash
   ros2 run tf2_ros tf2_echo base_link wrist_3_link
   ```

3. **Run components with verbose output**:
   ```bash
   python3.12 src/ur3_pick_place/scripts/move_robot.py --ros-args --log-level debug
   ```

## Advanced Topics

### Integration with MoveIt

For more advanced motion planning, you can integrate the simulation with MoveIt:

1. Install MoveIt for ROS 2:
   ```bash
   sudo apt install ros-jazzy-moveit
   ```

2. Create a MoveIt configuration for the UR3 robot
3. Modify the robot mover script to use MoveIt for trajectory generation

### Adding Collision Detection

To add collision detection:

1. Define collision objects in the environment
2. Use MoveIt's collision checking capabilities
3. Modify the robot mover script to check for collisions before executing movements

### Real Robot Integration

To integrate with a real UR3 robot:

1. Install the UR3 ROS 2 driver
2. Modify the robot mover script to send commands to the real robot
3. Implement feedback from the real robot's joint states

## References

1. Universal Robots UR3 Documentation: [https://www.universal-robots.com/products/ur3-robot/](https://www.universal-robots.com/products/ur3-robot/)
2. ROS 2 Documentation: [https://docs.ros.org/en/jazzy/](https://docs.ros.org/en/jazzy/)
3. RViz Documentation: [https://github.com/ros2/rviz](https://github.com/ros2/rviz)
4. Robot State Publisher Documentation: [https://github.com/ros/robot_state_publisher](https://github.com/ros/robot_state_publisher)
5. Cubic Ease-In/Ease-Out Interpolation: [https://easings.net/#easeInOutCubic](https://easings.net/#easeInOutCubic)
