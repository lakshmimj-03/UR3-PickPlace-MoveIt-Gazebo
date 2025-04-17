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
