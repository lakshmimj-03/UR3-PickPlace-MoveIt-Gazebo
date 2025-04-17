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

# Detailed Explanation of `move_robot.py`

This document provides a comprehensive explanation of the `move_robot.py` script, which implements the UR3 robot pick and place simulation.

## Overview

The `move_robot.py` script is the core component of the UR3 pick and place simulation. It:

1. Controls the robot's joint positions
2. Implements smooth trajectory generation
3. Visualizes objects and their manipulation
4. Executes the pick and place sequence

## Class Structure

The script defines a single class, `RobotMover`, which inherits from the ROS 2 `Node` class:

```python
class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        # ...
```

## Initialization

The `__init__` method initializes the node and sets up the necessary components:

```python
def __init__(self):
    super().__init__('robot_mover')
    
    # Publishers
    self.joint_state_pub = self.create_publisher(
        JointState,
        '/joint_states',
        10)
    
    self.marker_pub = self.create_publisher(
        MarkerArray,
        '/visualization_marker_array',
        10)
    
    # Object positions
    self.red_cube_pose = Pose(
        position=Point(x=0.4, y=0.0, z=0.05),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    
    self.blue_cylinder_pose = Pose(
        position=Point(x=0.4, y=0.2, z=0.05),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    
    self.place_pose = Pose(
        position=Point(x=0.4, y=-0.2, z=0.05),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    )
    
    # Joint names for the UR3 robot
    self.joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]
    
    # Home position
    self.home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    
    # Current joint positions
    self.current_joint_positions = self.home_position.copy()
    
    # Gripper state (0.0 is closed, 1.0 is open)
    self.gripper_state = 1.0
    
    # Attached object (None, 'red_cube', or 'blue_cylinder')
    self.attached_object = None
    
    # Create a timer to publish joint states and markers
    self.timer = self.create_timer(0.02, self.update)  # 50 Hz
    
    # Create a timer to start the pick and place sequence after a delay
    self.get_logger().info('Starting pick and place demo in 3 seconds...')
    self.create_timer(3.0, self.pick_place_sequence)
```

Key components:
- **Publishers**: For joint states and visualization markers
- **Object Poses**: Defines the positions of the red cube, blue cylinder, and place location
- **Joint Names**: Defines the names of the UR3 robot joints
- **Home Position**: Defines the initial joint positions
- **Gripper State**: Represents the state of the gripper (0.0 = closed, 1.0 = open)
- **Attached Object**: Tracks which object is currently attached to the gripper
- **Timers**: For updating the robot state and starting the pick and place sequence

## Update Method

The `update` method is called periodically (50 Hz) to update the robot state:

```python
def update(self):
    """Update the robot state"""
    # Publish joint states
    self.publish_joint_states()
    
    # Visualize objects
    self.visualize_objects()
```

## Joint State Publishing

The `publish_joint_states` method publishes the current joint positions to the `/joint_states` topic:

```python
def publish_joint_states(self):
    """Publish the current joint states"""
    joint_state = JointState()
    joint_state.header.stamp = self.get_clock().now().to_msg()
    joint_state.name = self.joint_names
    joint_state.position = self.current_joint_positions
    
    self.joint_state_pub.publish(joint_state)
```

## Smooth Joint Movement

The `move_joints` method implements smooth joint movement using cubic ease-in/ease-out interpolation:

```python
def move_joints(self, target_positions, duration=2.0):
    """Move the robot joints to the target positions over the specified duration"""
    start_positions = self.current_joint_positions.copy()
    start_time = time.time()
    end_time = start_time + duration
    
    # Use a smoother interpolation function (ease in/out)
    def ease_in_out(t):
        # Cubic ease in/out function: t^2 * (3-2t)
        return t * t * (3.0 - 2.0 * t)
    
    while time.time() < end_time:
        # Calculate interpolation factor (0 to 1)
        t = (time.time() - start_time) / duration
        t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
        
        # Apply easing function for smoother motion
        t_smooth = ease_in_out(t)
        
        # Interpolate joint positions
        for i in range(len(self.current_joint_positions)):
            self.current_joint_positions[i] = start_positions[i] + t_smooth * (target_positions[i] - start_positions[i])
        
        # Publish joint states for immediate feedback
        self.publish_joint_states()
        
        # Visualize objects if we're carrying something
        if self.attached_object is not None:
            self.visualize_objects()
        
        # Sleep a bit (50 Hz update rate)
        time.sleep(0.02)
    
    # Ensure we reach the exact target
    self.current_joint_positions = target_positions.copy()
    
    # Final update of joint states and visualization
    self.publish_joint_states()
    self.visualize_objects()
```

Key components:
- **Cubic Ease-In/Ease-Out**: The `ease_in_out` function implements cubic interpolation
- **Interpolation Loop**: Gradually updates joint positions over the specified duration
- **Immediate Feedback**: Publishes joint states and updates visualization during the movement
- **Final Update**: Ensures the robot reaches the exact target position

## Pick and Place Sequence

The `pick_place_sequence` method implements the complete pick and place sequence:

```python
def pick_place_sequence(self):
    """Execute the pick and place sequence"""
    # Move to home position
    self.get_logger().info('Moving to home position')
    self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
    
    # PICK RED CUBE
    # -----------------------------
    # Move above red cube
    self.get_logger().info('Moving above red cube')
    self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # Move to red cube
    self.get_logger().info('Moving to red cube')
    self.move_joints([0.0, -0.7, 0.5, -1.3, -1.57, 0.0])
    
    # Close gripper on red cube
    self.get_logger().info('Closing gripper on red cube')
    self.gripper_state = 0.0
    self.attached_object = 'red_cube'  # Attach the red cube to the gripper
    self.visualize_objects()  # Update visualization immediately
    time.sleep(1.0)
    
    # Lift red cube
    self.get_logger().info('Lifting red cube')
    self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # PLACE RED CUBE
    # -----------------------------
    # Move above place location
    self.get_logger().info('Moving above place location')
    self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # Move to place location
    self.get_logger().info('Moving to place location')
    self.move_joints([-0.5, -0.7, 0.5, -1.3, -1.57, 0.0])
    
    # Open gripper to release red cube
    self.get_logger().info('Opening gripper to release red cube')
    self.gripper_state = 1.0
    
    # Update red cube position to the place location
    self.red_cube_pose.position.x = self.place_pose.position.x
    self.red_cube_pose.position.y = self.place_pose.position.y
    self.red_cube_pose.position.z = self.place_pose.position.z + 0.04  # Half the height of the cube
    
    self.attached_object = None  # Detach the red cube from the gripper
    self.visualize_objects()  # Update visualization immediately
    time.sleep(1.0)
    
    # Move up from place location
    self.get_logger().info('Moving up from place location')
    self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # Return to home position
    self.get_logger().info('Returning to home position')
    self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
    
    # PICK BLUE CYLINDER
    # -----------------------------
    # Move above blue cylinder
    self.get_logger().info('Moving above blue cylinder')
    self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # Move to blue cylinder
    self.get_logger().info('Moving to blue cylinder')
    self.move_joints([0.5, -0.7, 0.5, -1.3, -1.57, 0.0])
    
    # Close gripper on blue cylinder
    self.get_logger().info('Closing gripper on blue cylinder')
    self.gripper_state = 0.0
    self.attached_object = 'blue_cylinder'  # Attach the blue cylinder to the gripper
    self.visualize_objects()  # Update visualization immediately
    time.sleep(1.0)
    
    # Lift blue cylinder
    self.get_logger().info('Lifting blue cylinder')
    self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # PLACE BLUE CYLINDER
    # -----------------------------
    # Move above place location for cylinder
    self.get_logger().info('Moving above place location for cylinder')
    self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # Move to place location for cylinder (on top of the red cube)
    self.get_logger().info('Moving to place location for cylinder')
    self.move_joints([-0.5, -0.6, 0.5, -1.4, -1.57, 0.0])
    
    # Open gripper to release blue cylinder
    self.get_logger().info('Opening gripper to release blue cylinder')
    self.gripper_state = 1.0
    
    # Update blue cylinder position to be on top of the red cube
    self.blue_cylinder_pose.position.x = self.place_pose.position.x
    self.blue_cylinder_pose.position.y = self.place_pose.position.y
    self.blue_cylinder_pose.position.z = self.place_pose.position.z + 0.12  # Height of cube + half height of cylinder
    
    self.attached_object = None  # Detach the blue cylinder from the gripper
    self.visualize_objects()  # Update visualization immediately
    time.sleep(1.0)
    
    # Move up from place location
    self.get_logger().info('Moving up from place location')
    self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    # Return to home position
    self.get_logger().info('Returning to home position')
    self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
    
    self.get_logger().info('Pick and place sequence completed')
```

Key components:
- **Waypoints**: Defines joint positions for each step of the sequence
- **Gripper Control**: Opens and closes the gripper at appropriate times
- **Object Attachment**: Attaches and detaches objects from the gripper
- **Position Updates**: Updates object positions when they are placed

## Object Visualization

The `visualize_objects` method creates and updates visualization markers for the objects:

```python
def visualize_objects(self):
    """Visualize objects in RViz"""
    marker_array = MarkerArray()
    
    # Get current time for all markers
    now = self.get_clock().now().to_msg()
    
    # Create a text marker to show the frame
    text_marker = Marker()
    text_marker.header.frame_id = 'base_link'
    text_marker.header.stamp = now
    text_marker.ns = 'objects'
    text_marker.id = 0
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.pose.position.x = 0.0
    text_marker.pose.position.y = 0.0
    text_marker.pose.position.z = 0.3
    text_marker.pose.orientation.w = 1.0
    text_marker.scale.z = 0.05  # Text height
    text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
    text_marker.text = "UR3 Pick and Place Demo"
    marker_array.markers.append(text_marker)
    
    # Status text marker to show current action
    status_marker = Marker()
    status_marker.header.frame_id = 'base_link'
    status_marker.header.stamp = now
    status_marker.ns = 'objects'
    status_marker.id = 10
    status_marker.type = Marker.TEXT_VIEW_FACING
    status_marker.action = Marker.ADD
    status_marker.pose.position.x = 0.0
    status_marker.pose.position.y = 0.0
    status_marker.pose.position.z = 0.25
    status_marker.pose.orientation.w = 1.0
    status_marker.scale.z = 0.05  # Text height
    status_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
    if self.attached_object == 'red_cube':
        status_marker.text = "Carrying Red Cube"
    elif self.attached_object == 'blue_cylinder':
        status_marker.text = "Carrying Blue Cylinder"
    else:
        status_marker.text = "Gripper Empty"
    marker_array.markers.append(status_marker)
    
    # Only show objects that are not attached to the gripper
    if self.attached_object != 'red_cube':
        # Red cube marker
        red_cube_marker = Marker()
        red_cube_marker.header.frame_id = 'base_link'
        red_cube_marker.header.stamp = now
        red_cube_marker.ns = 'objects'
        red_cube_marker.id = 1
        red_cube_marker.type = Marker.CUBE
        red_cube_marker.action = Marker.ADD
        red_cube_marker.pose = self.red_cube_pose
        red_cube_marker.scale.x = 0.08  # Larger for better visibility
        red_cube_marker.scale.y = 0.08
        red_cube_marker.scale.z = 0.08
        red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Fully opaque
        marker_array.markers.append(red_cube_marker)
        
        # Red cube text label
        red_text = Marker()
        red_text.header.frame_id = 'base_link'
        red_text.header.stamp = now
        red_text.ns = 'objects'
        red_text.id = 5
        red_text.type = Marker.TEXT_VIEW_FACING
        red_text.action = Marker.ADD
        red_text.pose.position.x = self.red_cube_pose.position.x
        red_text.pose.position.y = self.red_cube_pose.position.y
        red_text.pose.position.z = self.red_cube_pose.position.z + 0.1
        red_text.pose.orientation.w = 1.0
        red_text.scale.z = 0.04  # Text height
        red_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
        red_text.text = "Red Cube"
        marker_array.markers.append(red_text)
    else:
        # If the red cube is attached to the gripper, show it at the end effector
        red_cube_marker = Marker()
        red_cube_marker.header.frame_id = 'wrist_3_link'
        red_cube_marker.header.stamp = now
        red_cube_marker.ns = 'objects'
        red_cube_marker.id = 1
        red_cube_marker.type = Marker.CUBE
        red_cube_marker.action = Marker.ADD
        red_cube_marker.pose.position.x = 0.0
        red_cube_marker.pose.position.y = 0.0
        red_cube_marker.pose.position.z = 0.05
        red_cube_marker.pose.orientation.w = 1.0
        red_cube_marker.scale.x = 0.08
        red_cube_marker.scale.y = 0.08
        red_cube_marker.scale.z = 0.08
        red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker_array.markers.append(red_cube_marker)
        
        # Red cube text label attached to the gripper
        red_text = Marker()
        red_text.header.frame_id = 'wrist_3_link'
        red_text.header.stamp = now
        red_text.ns = 'objects'
        red_text.id = 5
        red_text.type = Marker.TEXT_VIEW_FACING
        red_text.action = Marker.ADD
        red_text.pose.position.x = 0.0
        red_text.pose.position.y = 0.0
        red_text.pose.position.z = 0.1
        red_text.pose.orientation.w = 1.0
        red_text.scale.z = 0.04  # Text height
        red_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
        red_text.text = "Red Cube"
        marker_array.markers.append(red_text)
    
    # Similar code for blue cylinder...
    
    # Place location marker
    place_marker = Marker()
    place_marker.header.frame_id = 'base_link'
    place_marker.header.stamp = now
    place_marker.ns = 'objects'
    place_marker.id = 3
    place_marker.type = Marker.CUBE
    place_marker.action = Marker.ADD
    place_marker.pose = self.place_pose
    place_marker.scale.x = 0.15
    place_marker.scale.y = 0.15
    place_marker.scale.z = 0.01
    place_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Fully opaque
    marker_array.markers.append(place_marker)
    
    # Gripper visualization
    gripper_marker = Marker()
    gripper_marker.header.frame_id = 'wrist_3_link'
    gripper_marker.header.stamp = now
    gripper_marker.ns = 'objects'
    gripper_marker.id = 8
    gripper_marker.type = Marker.CUBE
    gripper_marker.action = Marker.ADD
    gripper_marker.pose.position.x = 0.0
    gripper_marker.pose.position.y = 0.0
    gripper_marker.pose.position.z = 0.02
    gripper_marker.pose.orientation.w = 1.0
    gripper_marker.scale.x = 0.05
    gripper_marker.scale.y = 0.1 * (1.0 - self.gripper_state)  # Scale based on gripper state
    gripper_marker.scale.z = 0.02
    gripper_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray
    marker_array.markers.append(gripper_marker)
    
    # Left gripper finger
    left_finger = Marker()
    left_finger.header.frame_id = 'wrist_3_link'
    left_finger.header.stamp = now
    left_finger.ns = 'objects'
    left_finger.id = 11
    left_finger.type = Marker.CUBE
    left_finger.action = Marker.ADD
    left_finger.pose.position.x = 0.0
    left_finger.pose.position.y = 0.05 * self.gripper_state  # Move based on gripper state
    left_finger.pose.position.z = 0.02
    left_finger.pose.orientation.w = 1.0
    left_finger.scale.x = 0.02
    left_finger.scale.y = 0.01
    left_finger.scale.z = 0.04
    left_finger.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Light gray
    marker_array.markers.append(left_finger)
    
    # Right gripper finger
    right_finger = Marker()
    right_finger.header.frame_id = 'wrist_3_link'
    right_finger.header.stamp = now
    right_finger.ns = 'objects'
    right_finger.id = 12
    right_finger.type = Marker.CUBE
    right_finger.action = Marker.ADD
    right_finger.pose.position.x = 0.0
    right_finger.pose.position.y = -0.05 * self.gripper_state  # Move based on gripper state
    right_finger.pose.position.z = 0.02
    right_finger.pose.orientation.w = 1.0
    right_finger.scale.x = 0.02
    right_finger.scale.y = 0.01
    right_finger.scale.z = 0.04
    right_finger.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Light gray
    marker_array.markers.append(right_finger)
    
    # Current action text
    action_text = Marker()
    action_text.header.frame_id = 'base_link'
    action_text.header.stamp = now
    action_text.ns = 'objects'
    action_text.id = 13
    action_text.type = Marker.TEXT_VIEW_FACING
    action_text.action = Marker.ADD
    action_text.pose.position.x = 0.0
    action_text.pose.position.y = 0.0
    action_text.pose.position.z = 0.2
    action_text.pose.orientation.w = 1.0
    action_text.scale.z = 0.04  # Text height
    action_text.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan
    
    # Set the text based on the gripper state and attached object
    if self.gripper_state < 0.5:  # Gripper is closed
        if self.attached_object == 'red_cube':
            action_text.text = "Picking Red Cube"
        elif self.attached_object == 'blue_cylinder':
            action_text.text = "Picking Blue Cylinder"
        else:
            action_text.text = "Gripper Closed"
    else:  # Gripper is open
        if self.attached_object is None:
            action_text.text = "Gripper Open"
        else:
            action_text.text = "Placing Object"
    
    marker_array.markers.append(action_text)
    
    self.marker_pub.publish(marker_array)
```

Key components:
- **Marker Creation**: Creates visualization markers for objects, text, and the gripper
- **Object Attachment**: Changes the reference frame of objects based on whether they are attached to the gripper
- **Gripper Visualization**: Visualizes the gripper state with moving fingers
- **Status Text**: Displays the current status of the robot and objects

## Main Function

The `main` function initializes the ROS 2 node and starts the robot mover:

```python
def main():
    rclpy.init()
    node = RobotMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Conclusion

The `move_robot.py` script provides a complete implementation of a UR3 robot pick and place simulation. It demonstrates:

1. **Robot Control**: Direct control of robot joint positions
2. **Smooth Motion**: Implementation of cubic ease-in/ease-out interpolation
3. **Object Manipulation**: Visualization and manipulation of objects
4. **Pick and Place Sequence**: Complete implementation of a pick and place sequence

The script is designed to be easy to understand and modify, making it a good starting point for more complex robot simulations.

# Detailed Explanation of Shell Scripts

This document provides a comprehensive explanation of the shell scripts used to launch the UR3 robot pick and place simulation.

## Overview

Two shell scripts are provided to simplify the launching of the simulation:

1. `run_simulation.sh`: Launches all components in separate terminals
2. `run_simulation_tmux.sh`: Launches all components in a single terminal using tmux

## `run_simulation.sh`

The `run_simulation.sh` script launches all necessary components for the simulation in separate processes.

### Script Content

```bash
#!/bin/bash

# Script to run the UR3 pick and place simulation
# This script launches RViz, robot state publisher, and the robot mover script

# Set the workspace path
WORKSPACE_PATH="/home/lachu/ros2_workspaces/ros2_ws"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to display colored text
print_color() {
    COLOR=$1
    TEXT=$2
    echo -e "\033[${COLOR}m${TEXT}\033[0m"
}

# Print header
print_color "1;34" "=== UR3 Pick and Place Simulation ==="
print_color "1;34" "=== Starting all components... ==="
echo ""

# Source ROS setup
print_color "1;33" "Sourcing ROS setup..."
source /opt/ros/jazzy/setup.bash

# Check if we're in the right directory, if not, change to it
if [ "$(pwd)" != "$WORKSPACE_PATH" ]; then
    print_color "1;33" "Changing to workspace directory: $WORKSPACE_PATH"
    cd $WORKSPACE_PATH
fi

# Check if Python 3.12 is available
if command_exists python3.12; then
    PYTHON_CMD="python3.12"
else
    print_color "1;31" "Python 3.12 not found, trying python3..."
    PYTHON_CMD="python3"
fi

# Launch RViz in the background
print_color "1;32" "Starting RViz..."
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz &
RVIZ_PID=$!
sleep 2  # Give RViz time to start

# Launch robot state publisher in the background
print_color "1;32" "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)" &
RSP_PID=$!
sleep 2  # Give robot state publisher time to start

# Launch the robot mover script
print_color "1;32" "Starting robot mover script..."
$PYTHON_CMD src/ur3_pick_place/scripts/move_robot.py &
MOVER_PID=$!

# Print information about the running processes
echo ""
print_color "1;36" "All components started successfully!"
print_color "1;36" "RViz PID: $RVIZ_PID"
print_color "1;36" "Robot State Publisher PID: $RSP_PID"
print_color "1;36" "Robot Mover PID: $MOVER_PID"
echo ""
print_color "1;33" "Press Ctrl+C to stop all components"

# Wait for user to press Ctrl+C
trap 'print_color "1;31" "Stopping all components..."; kill $RVIZ_PID $RSP_PID $MOVER_PID 2>/dev/null; exit' INT
wait $MOVER_PID

# If the robot mover script exits, kill the other processes
print_color "1;31" "Robot mover script exited, stopping other components..."
kill $RVIZ_PID $RSP_PID 2>/dev/null

print_color "1;34" "=== Simulation ended ==="
```

### Key Components

1. **Workspace Path**: Sets the path to the ROS 2 workspace
2. **Helper Functions**:
   - `command_exists`: Checks if a command is available
   - `print_color`: Prints colored text for better readability
3. **Environment Setup**:
   - Sources the ROS 2 setup script
   - Changes to the workspace directory if needed
   - Determines the Python command to use
4. **Component Launch**:
   - Launches RViz with the appropriate configuration
   - Launches the robot state publisher with the UR3 URDF
   - Launches the robot mover script
5. **Process Management**:
   - Stores the process IDs of each component
   - Sets up a trap to handle Ctrl+C
   - Waits for the robot mover script to exit
   - Kills all processes when the script exits

### Usage

To use the script:

```bash
cd /home/lachu/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation.sh
```

Press Ctrl+C to stop all components.

## `run_simulation_tmux.sh`

The `run_simulation_tmux.sh` script launches all necessary components for the simulation in a tmux session.

### Script Content

```bash
#!/bin/bash

# Script to run the UR3 pick and place simulation using tmux
# This script launches RViz, robot state publisher, and the robot mover script in a tmux session

# Set the workspace path
WORKSPACE_PATH="/home/lachu/ros2_workspaces/ros2_ws"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check if tmux is installed
if ! command_exists tmux; then
    echo "tmux is not installed. Please install it with: sudo apt-get install tmux"
    echo "Alternatively, use the regular script: ./run_simulation.sh"
    exit 1
fi

# Check if we're in the right directory, if not, change to it
if [ "$(pwd)" != "$WORKSPACE_PATH" ]; then
    echo "Changing to workspace directory: $WORKSPACE_PATH"
    cd $WORKSPACE_PATH
fi

# Check if Python 3.12 is available
if command_exists python3.12; then
    PYTHON_CMD="python3.12"
else
    echo "Python 3.12 not found, trying python3..."
    PYTHON_CMD="python3"
fi

# Source ROS setup
source /opt/ros/jazzy/setup.bash

# Create a new tmux session
SESSION_NAME="ur3_simulation"

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Create a new session
tmux new-session -d -s $SESSION_NAME -n "UR3 Simulation"

# Split the window into three panes
tmux split-window -h -t $SESSION_NAME
tmux split-window -v -t $SESSION_NAME:0.0

# Run RViz in the first pane
tmux send-keys -t $SESSION_NAME:0.0 "echo 'Starting RViz...'; ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz" C-m

# Run robot state publisher in the second pane
tmux send-keys -t $SESSION_NAME:0.1 "echo 'Starting robot state publisher...'; sleep 2; ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:=\"\$(xacro \$(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)\"" C-m

# Run the robot mover script in the third pane
tmux send-keys -t $SESSION_NAME:0.2 "echo 'Starting robot mover script...'; sleep 4; $PYTHON_CMD src/ur3_pick_place/scripts/move_robot.py" C-m

# Attach to the session
tmux attach-session -t $SESSION_NAME

# When the user detaches from the session (Ctrl+B, D), kill the session
tmux kill-session -t $SESSION_NAME 2>/dev/null
```

### Key Components

1. **Workspace Path**: Sets the path to the ROS 2 workspace
2. **Helper Functions**:
   - `command_exists`: Checks if a command is available
3. **Environment Setup**:
   - Checks if tmux is installed
   - Changes to the workspace directory if needed
   - Determines the Python command to use
   - Sources the ROS 2 setup script
4. **Tmux Session Setup**:
   - Creates a new tmux session
   - Splits the window into three panes
5. **Component Launch**:
   - Runs RViz in the first pane
   - Runs the robot state publisher in the second pane
   - Runs the robot mover script in the third pane
6. **Session Management**:
   - Attaches to the tmux session
   - Kills the session when the user detaches

### Usage

To use the script:

```bash
cd /home/lachu/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation_tmux.sh
```

Press Ctrl+B, then D to detach from the tmux session (this will also stop all components).

## Comparison

### `run_simulation.sh`

**Advantages**:
- Does not require tmux to be installed
- Simpler to understand for users not familiar with tmux
- Provides colored output for better readability

**Disadvantages**:
- Launches components in separate terminals
- May be harder to manage multiple terminal windows

### `run_simulation_tmux.sh`

**Advantages**:
- Launches all components in a single terminal window
- Provides a clean, organized view of all component outputs
- Easier to manage a single terminal window

**Disadvantages**:
- Requires tmux to be installed
- May be harder to use for users not familiar with tmux

## Customization

### Modifying the Workspace Path

If your workspace is in a different location, modify the `WORKSPACE_PATH` variable in both scripts:

```bash
# Set the workspace path
WORKSPACE_PATH="/path/to/your/workspace"
```

### Using a Different Python Version

The scripts automatically check for Python 3.12 and fall back to Python 3 if not found. If you want to use a specific Python version, modify the Python command check:

```bash
# Check if Python 3.12 is available
if command_exists python3.10; then  # Change to your preferred version
    PYTHON_CMD="python3.10"
else
    print_color "1;31" "Python 3.10 not found, trying python3..."
    PYTHON_CMD="python3"
fi
```

### Changing the RViz Configuration

To use a different RViz configuration, modify the RViz command:

```bash
# Launch RViz in the background
print_color "1;32" "Starting RViz..."
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/your_custom_config.rviz &
```

### Modifying the Robot Description

To use a different robot description or parameters, modify the robot state publisher command:

```bash
# Launch robot state publisher in the background
print_color "1;32" "Starting robot state publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=your_robot ur_type:=ur5 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)" &
```

## Conclusion

These shell scripts provide convenient ways to launch the UR3 robot pick and place simulation. The `run_simulation.sh` script is simpler and does not require additional software, while the `run_simulation_tmux.sh` script provides a more organized view of all component outputs in a single terminal window.

Both scripts handle the necessary setup and launch all required components for the simulation, making it easy to start and stop the simulation with a single command.

# Cubic Ease-In/Ease-Out Interpolation: Theory and Implementation

## Introduction

Smooth motion is essential for realistic robot simulations and real-world robot control. Abrupt changes in velocity can cause mechanical stress, vibrations, and unnatural-looking movements. This document explains the theory behind cubic ease-in/ease-out interpolation, which is used in the UR3 pick and place simulation to create smooth, natural robot movements.

## Linear Interpolation vs. Cubic Interpolation

### Linear Interpolation

Linear interpolation is the simplest form of interpolation, where the value changes at a constant rate between two points. For joint positions, this would mean:

```
position(t) = start_position + t * (end_position - start_position)
```

Where:
- `t` is the normalized time (0 to 1)
- `start_position` is the initial joint position
- `end_position` is the target joint position

While linear interpolation is simple, it has a significant drawback: the velocity changes instantaneously at the start and end of the movement. This results in:
- Abrupt starts and stops
- Mechanical stress on the robot
- Unnatural-looking movements

### Cubic Interpolation

Cubic interpolation uses a cubic function to create a smooth transition between two points. The cubic ease-in/ease-out function is defined as:

```
f(t) = t² * (3 - 2t)
```

Where:
- `t` is the normalized time (0 to 1)
- `f(t)` is the interpolation factor

This function has several important properties:
1. `f(0) = 0`: Starts at 0
2. `f(1) = 1`: Ends at 1
3. `f'(0) = 0`: Zero velocity at the start
4. `f'(1) = 0`: Zero velocity at the end

These properties ensure that the robot starts and stops smoothly, without abrupt changes in velocity.

## Mathematical Analysis

### Function Definition

The cubic ease-in/ease-out function is:

```
f(t) = t² * (3 - 2t) = 3t² - 2t³
```

### Derivatives

The first derivative (velocity) is:

```
f'(t) = 6t - 6t² = 6t(1 - t)
```

The second derivative (acceleration) is:

```
f''(t) = 6 - 12t
```

### Key Properties

1. **Position**: `f(t)` varies from 0 to 1 as `t` varies from 0 to 1
2. **Velocity**: `f'(t)` starts at 0, reaches a maximum at `t = 0.5`, and returns to 0 at `t = 1`
3. **Acceleration**: `f''(t)` starts positive, becomes 0 at `t = 0.5`, and becomes negative

This creates a smooth S-shaped curve that:
- Starts with zero velocity
- Accelerates gradually
- Reaches maximum velocity at the midpoint
- Decelerates gradually
- Ends with zero velocity

## Implementation in the UR3 Simulation

In the UR3 pick and place simulation, cubic ease-in/ease-out interpolation is implemented in the `move_joints` method of the `RobotMover` class:

```python
def move_joints(self, target_positions, duration=2.0):
    """Move the robot joints to the target positions over the specified duration"""
    start_positions = self.current_joint_positions.copy()
    start_time = time.time()
    end_time = start_time + duration
    
    # Use a smoother interpolation function (ease in/out)
    def ease_in_out(t):
        # Cubic ease in/out function: t^2 * (3-2t)
        return t * t * (3.0 - 2.0 * t)
    
    while time.time() < end_time:
        # Calculate interpolation factor (0 to 1)
        t = (time.time() - start_time) / duration
        t = max(0.0, min(1.0, t))  # Clamp to [0, 1]
        
        # Apply easing function for smoother motion
        t_smooth = ease_in_out(t)
        
        # Interpolate joint positions
        for i in range(len(self.current_joint_positions)):
            self.current_joint_positions[i] = start_positions[i] + t_smooth * (target_positions[i] - start_positions[i])
        
        # Publish joint states for immediate feedback
        self.publish_joint_states()
        
        # Visualize objects if we're carrying something
        if self.attached_object is not None:
            self.visualize_objects()
        
        # Sleep a bit (50 Hz update rate)
        time.sleep(0.02)
    
    # Ensure we reach the exact target
    self.current_joint_positions = target_positions.copy()
    
    # Final update of joint states and visualization
    self.publish_joint_states()
    self.visualize_objects()
```

The key steps are:
1. Define the `ease_in_out` function that implements the cubic interpolation
2. Calculate the normalized time `t` based on the elapsed time
3. Apply the easing function to get a smooth interpolation factor `t_smooth`
4. Use this factor to interpolate between the start and target joint positions

## Visual Representation

### Position Curve

The position curve for cubic ease-in/ease-out interpolation is S-shaped:

```
1.0 |        -------
    |      /
    |     /
    |    /
    |   /
0.5 |  /
    | /
    |/
0.0 +---------------
    0.0    0.5    1.0
```

### Velocity Curve

The velocity curve is bell-shaped, starting and ending at zero:

```
1.5 |
    |     /\
    |    /  \
    |   /    \
1.0 |  /      \
    | /        \
    |/          \
0.0 +---------------
    0.0    0.5    1.0
```

### Acceleration Curve

The acceleration curve is linear, starting positive and ending negative:

```
6.0 |
    |\
    | \
    |  \
    |   \
0.0 +----\-----------
    |     \
    |      \
    |       \
-6.0 |        \
    0.0    0.5    1.0
```

## Comparison with Other Interpolation Methods

### Linear Interpolation

- **Position**: Linear from 0 to 1
- **Velocity**: Constant (except at endpoints where it's undefined)
- **Acceleration**: Zero (except at endpoints where it's undefined)
- **Result**: Abrupt starts and stops

### Quadratic Ease-In

- **Position**: Starts slow, ends fast
- **Velocity**: Starts at zero, ends at maximum
- **Acceleration**: Constant positive
- **Result**: Smooth start but abrupt stop

### Quadratic Ease-Out

- **Position**: Starts fast, ends slow
- **Velocity**: Starts at maximum, ends at zero
- **Acceleration**: Constant negative
- **Result**: Abrupt start but smooth stop

### Cubic Ease-In/Ease-Out

- **Position**: Starts slow, speeds up in the middle, ends slow
- **Velocity**: Starts at zero, reaches maximum in the middle, ends at zero
- **Acceleration**: Starts positive, becomes negative
- **Result**: Smooth start and stop

## Benefits for Robot Control

Using cubic ease-in/ease-out interpolation for robot control provides several benefits:

1. **Reduced Mechanical Stress**: Smooth acceleration and deceleration reduce stress on the robot's mechanical components
2. **Improved Accuracy**: Gradual changes in velocity allow for more precise control
3. **Natural Movement**: The resulting motion appears more natural and fluid
4. **Energy Efficiency**: Smooth movements typically require less energy than abrupt ones
5. **Reduced Vibration**: Gradual acceleration and deceleration reduce vibrations in the robot

## Practical Considerations

### Timing

The duration of the movement affects the maximum velocity and acceleration:
- Shorter durations result in higher velocities and accelerations
- Longer durations result in lower velocities and accelerations

In the UR3 simulation, the duration is specified as a parameter to the `move_joints` method:

```python
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0], duration=2.0)
```

### Update Rate

The update rate affects the smoothness of the motion:
- Higher update rates result in smoother motion
- Lower update rates may cause jerky motion

In the UR3 simulation, the update rate is set to 50 Hz:

```python
time.sleep(0.02)  # 50 Hz update rate
```

### Interpolation Steps

The number of interpolation steps affects the precision of the motion:
- More steps result in more precise motion
- Fewer steps may cause less precise motion

In the UR3 simulation, the interpolation is continuous based on the elapsed time.

## Conclusion

Cubic ease-in/ease-out interpolation is a powerful technique for creating smooth, natural robot movements. By ensuring zero velocity at the start and end of each movement, it reduces mechanical stress, improves accuracy, and creates more realistic simulations.

The implementation in the UR3 pick and place simulation demonstrates how this technique can be applied to create smooth robot movements with minimal computational complexity.

# Running and Customizing the UR3 Pick and Place Simulation

This document provides detailed instructions for running and customizing the UR3 pick and place simulation.

## Table of Contents

1. [Running the Simulation](#running-the-simulation)
   - [Using the Single-Command Script](#using-the-single-command-script)
   - [Using the Tmux Script](#using-the-tmux-script)
   - [Running Components Manually](#running-components-manually)
2. [Understanding the Simulation](#understanding-the-simulation)
   - [Simulation Components](#simulation-components)
   - [Simulation Sequence](#simulation-sequence)
   - [Visualization](#visualization)
3. [Customizing the Simulation](#customizing-the-simulation)
   - [Modifying Robot Movements](#modifying-robot-movements)
   - [Changing Object Positions](#changing-object-positions)
   - [Adjusting Timing Parameters](#adjusting-timing-parameters)
   - [Customizing Object Appearance](#customizing-object-appearance)
   - [Adding New Objects](#adding-new-objects)
4. [Advanced Customization](#advanced-customization)
   - [Creating Custom Sequences](#creating-custom-sequences)
   - [Implementing Custom Interpolation](#implementing-custom-interpolation)
   - [Adding Collision Detection](#adding-collision-detection)
5. [Troubleshooting](#troubleshooting)
   - [Common Issues](#common-issues)
   - [Debugging Techniques](#debugging-techniques)

## Running the Simulation

### Using the Single-Command Script

The simplest way to run the simulation is using the provided shell script:

```bash
cd ~/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation.sh
```

This script will:
1. Source the ROS 2 setup
2. Launch RViz with the appropriate configuration
3. Start the robot state publisher with the UR3 URDF
4. Run the robot mover script

Press Ctrl+C to stop all components.

### Using the Tmux Script

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

### Running Components Manually

If you prefer to run the components manually, you can use the following commands:

```bash
# Terminal 1: Start RViz
cd ~/ros2_workspaces/ros2_ws
source /opt/ros/jazzy/setup.bash
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz
```

```bash
# Terminal 2: Start the robot state publisher
cd ~/ros2_workspaces/ros2_ws
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)"
```

```bash
# Terminal 3: Run the robot mover script
cd ~/ros2_workspaces/ros2_ws
source /opt/ros/jazzy/setup.bash
python3.12 src/ur3_pick_place/scripts/move_robot.py
```

## Understanding the Simulation

### Simulation Components

The simulation consists of the following main components:

1. **RViz**: Visualizes the robot and objects
2. **Robot State Publisher**: Publishes the robot's state based on the URDF model and joint positions
3. **Robot Mover Script**: Controls the robot's joint positions and simulates object manipulation

### Simulation Sequence

The simulation performs the following sequence:

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

### Visualization

The simulation visualizes:
- The UR3 robot based on its URDF model
- A red cube and a blue cylinder as objects to manipulate
- A green platform as the place location
- The gripper state (open or closed)
- Status text showing the current action

## Customizing the Simulation

### Modifying Robot Movements

To modify the robot's movements, edit the `pick_place_sequence` method in the `move_robot.py` script. The method contains a sequence of `move_joints` calls with target joint positions.

For example, to change the position above the red cube:

```python
# Original movement
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0])

# Modified movement (different shoulder pan angle)
self.move_joints([0.3, -1.0, 0.5, -1.0, -1.57, 0.0])
```

The joint positions are specified in the following order:
1. `shoulder_pan_joint`: Rotates the robot base
2. `shoulder_lift_joint`: Controls the shoulder lift
3. `elbow_joint`: Controls the elbow position
4. `wrist_1_joint`: First wrist joint
5. `wrist_2_joint`: Second wrist joint
6. `wrist_3_joint`: Third wrist joint (end-effector rotation)

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

Remember to also update the robot movements to match the new object positions.

### Adjusting Timing Parameters

To adjust the timing of movements, modify the `duration` parameter in the `move_joints` method calls:

```python
# Original movement (2.0 seconds)
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0], duration=2.0)

# Modified movement (slower, 3.0 seconds)
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0], duration=3.0)
```

You can also adjust the sleep duration between steps in the pick and place sequence:

```python
# Original sleep duration
time.sleep(1.0)

# Modified sleep duration (longer pause)
time.sleep(2.0)
```

### Customizing Object Appearance

To customize the appearance of objects, modify the marker definitions in the `visualize_objects` method:

```python
# Original red cube scale
red_cube_marker.scale.x = 0.08
red_cube_marker.scale.y = 0.08
red_cube_marker.scale.z = 0.08

# Modified red cube scale (larger cube)
red_cube_marker.scale.x = 0.12
red_cube_marker.scale.y = 0.12
red_cube_marker.scale.z = 0.12
```

```python
# Original red cube color
red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

# Modified red cube color (yellow)
red_cube_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
```

### Adding New Objects

To add a new object, add a new pose definition in the `__init__` method:

```python
# Add a green sphere
self.green_sphere_pose = Pose(
    position=Point(x=0.4, y=0.4, z=0.05),
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
)
```

Then add a new marker definition in the `visualize_objects` method:

```python
# Green sphere marker
green_sphere_marker = Marker()
green_sphere_marker.header.frame_id = 'base_link'
green_sphere_marker.header.stamp = now
green_sphere_marker.ns = 'objects'
green_sphere_marker.id = 14  # Use a unique ID
green_sphere_marker.type = Marker.SPHERE
green_sphere_marker.action = Marker.ADD
green_sphere_marker.pose = self.green_sphere_pose
green_sphere_marker.scale.x = 0.08
green_sphere_marker.scale.y = 0.08
green_sphere_marker.scale.z = 0.08
green_sphere_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
marker_array.markers.append(green_sphere_marker)
```

Finally, update the pick and place sequence to include the new object:

```python
# Pick green sphere
self.get_logger().info('Moving above green sphere')
self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

self.get_logger().info('Moving to green sphere')
self.move_joints([0.5, -0.7, 0.5, -1.3, -1.57, 0.0])

self.get_logger().info('Closing gripper on green sphere')
self.gripper_state = 0.0
self.attached_object = 'green_sphere'
self.visualize_objects()
time.sleep(1.0)

# ... rest of the sequence
```

## Advanced Customization

### Creating Custom Sequences

To create a custom pick and place sequence, create a new method in the `RobotMover` class:

```python
def custom_sequence(self):
    """Execute a custom sequence"""
    # Move to home position
    self.get_logger().info('Moving to home position')
    self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
    
    # Custom movements
    self.get_logger().info('Custom movement 1')
    self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])
    
    self.get_logger().info('Custom movement 2')
    self.move_joints([0.5, -0.7, 0.5, -1.3, -1.57, 0.0])
    
    # ... more movements
    
    self.get_logger().info('Custom sequence completed')
```

Then update the timer in the `__init__` method to call your custom sequence:

```python
# Create a timer to start the custom sequence after a delay
self.get_logger().info('Starting custom sequence in 3 seconds...')
self.create_timer(3.0, self.custom_sequence)
```

### Implementing Custom Interpolation

To implement a custom interpolation function, modify the `ease_in_out` function in the `move_joints` method:

```python
# Original cubic ease-in/ease-out function
def ease_in_out(t):
    # Cubic ease in/out function: t^2 * (3-2t)
    return t * t * (3.0 - 2.0 * t)

# Custom quartic ease-in/ease-out function
def ease_in_out(t):
    # Quartic ease in/out function: t^4 * (35 - 84t + 70t^2 - 20t^3)
    if t < 0.5:
        return 8 * t * t * t * t
    else:
        t = 1 - t
        return 1 - 8 * t * t * t * t
```

### Adding Collision Detection

To add basic collision detection, you can implement a simple check before moving the robot:

```python
def check_collision(self, joint_positions):
    """Check if the robot would collide with objects at the given joint positions"""
    # Implement collision detection logic here
    # For example, check if the end effector would be too close to the table
    
    # Get the end effector position (simplified example)
    end_effector_z = 0.1  # Simplified calculation
    
    # Check if the end effector would be below the table
    if end_effector_z < 0.0:
        return True  # Collision detected
    
    return False  # No collision
```

Then use this function before moving the robot:

```python
def move_joints(self, target_positions, duration=2.0):
    """Move the robot joints to the target positions over the specified duration"""
    # Check for collisions
    if self.check_collision(target_positions):
        self.get_logger().error('Collision detected! Movement aborted.')
        return
    
    # Rest of the method...
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

4. **Script not running**:
   - Make sure you have the correct Python version installed
   - Check that all required packages are installed
   - Verify that the script has executable permissions

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

4. **Check for Python errors**:
   ```bash
   python3.12 -m py_compile src/ur3_pick_place/scripts/move_robot.py
   ```

5. **Check ROS 2 nodes**:
   ```bash
   ros2 node list
   ros2 node info /robot_mover
   ```
# UR3 Pick and Place Simulation Documentation

Welcome to the documentation for the UR3 Pick and Place Simulation. This documentation provides comprehensive information about the simulation, including theoretical background, implementation details, and practical usage.

## Table of Contents

1. [Overview](#overview)
2. [Getting Started](#getting-started)
3. [Documentation Files](#documentation-files)
4. [Quick Reference](#quick-reference)

## Overview

The UR3 Pick and Place Simulation demonstrates a Universal Robots UR3 robot performing pick and place operations. The simulation focuses on smooth, realistic robot movements and proper visualization of object manipulation without requiring a full physics simulation environment like Gazebo.

Key features:
- Direct control of robot joint positions
- Smooth trajectory generation with cubic ease-in/ease-out interpolation
- Visualization of objects and their attachment to the robot gripper
- Complete pick and place sequence with proper object handling
- Simple and efficient launching mechanisms

## Getting Started

To run the simulation:

```bash
cd ~/ros2_workspaces/ros2_ws
./src/ur3_pick_place/scripts/run_simulation.sh
```

This will launch RViz, the robot state publisher, and the robot mover script. You should see the UR3 robot performing a pick and place sequence with a red cube and a blue cylinder.

For more detailed instructions, see the [Running and Customizing](running_and_customizing.md) documentation.

## Documentation Files

The documentation is organized into the following files:

- [**DETAILED_README.md**](../DETAILED_README.md): Comprehensive overview of the simulation, including theoretical background, system architecture, implementation details, and usage instructions.
- [**move_robot_explanation.md**](move_robot_explanation.md): Detailed explanation of the `move_robot.py` script, which is the core component of the simulation.
- [**shell_scripts_explanation.md**](shell_scripts_explanation.md): Explanation of the shell scripts used to launch the simulation.
- [**cubic_interpolation_theory.md**](cubic_interpolation_theory.md): Theoretical background on cubic ease-in/ease-out interpolation, which is used to create smooth robot movements.
- [**running_and_customizing.md**](running_and_customizing.md): Practical guide to running and customizing the simulation.

## Quick Reference

### Running the Simulation

```bash
# Option 1: Using the single-command script
./src/ur3_pick_place/scripts/run_simulation.sh

# Option 2: Using the tmux script
./src/ur3_pick_place/scripts/run_simulation_tmux.sh

# Option 3: Running components manually
# Terminal 1
ros2 run rviz2 rviz2 -d src/ur3_pick_place/config/view_robot.rviz

# Terminal 2
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro $(ros2 pkg prefix ur_description)/share/ur_description/urdf/ur.urdf.xacro name:=ur3_robot ur_type:=ur3 use_fake_hardware:=true safety_limits:=true safety_pos_margin:=0.15 safety_k_position:=20)"

# Terminal 3
python3.12 src/ur3_pick_place/scripts/move_robot.py
```

### Key Files

- **`scripts/move_robot.py`**: Main script for controlling the robot and simulating object manipulation
- **`scripts/run_simulation.sh`**: Shell script for launching all components with a single command
- **`scripts/run_simulation_tmux.sh`**: Shell script for launching all components in a tmux session
- **`config/view_robot.rviz`**: RViz configuration file for visualizing the robot and objects

### Customization Examples

#### Changing Object Positions

```python
# Modify in the __init__ method of the RobotMover class
self.red_cube_pose = Pose(
    position=Point(x=0.5, y=0.1, z=0.05),  # Modified position
    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
)
```

#### Modifying Robot Movements

```python
# Modify in the pick_place_sequence method of the RobotMover class
self.move_joints([0.3, -1.0, 0.5, -1.0, -1.57, 0.0])  # Modified joint positions
```

#### Adjusting Timing

```python
# Modify the duration parameter in move_joints calls
self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0], duration=3.0)  # Slower movement
```

For more detailed customization instructions, see the [Running and Customizing](running_and_customizing.md) documentation.
