# UR3 Robot Pick and Place Simulation - Report 2
## Motion Planning, Trajectory Generation, and ROS2 Architecture

## Table of Contents
- [Motion Planning and Trajectory Generation](#motion-planning-and-trajectory-generation)
- [Minimum Jerk Trajectory](#minimum-jerk-trajectory)
- [Key Pose Interpolation](#key-pose-interpolation)
- [Angle Wrapping Handling](#angle-wrapping-handling)
- [Control Loop Implementation](#control-loop-implementation)
- [ROS2 Architecture](#ros2-architecture)
- [Nodes and Components](#nodes-and-components)
- [Topics and Messages](#topics-and-messages)
- [Parameters](#parameters)
- [Visualization in RViz](#visualization-in-rviz)
- [Pick and Place Sequence](#pick-and-place-sequence)
- [Executable Script Commands](#executable-script-commands)

## Motion Planning and Trajectory Generation

The UR3 robot's motion is controlled using a custom Python node that implements advanced trajectory generation techniques to achieve ultra-smooth, glitch-free motion. This section details the algorithms and approaches used to generate these trajectories.

### Minimum Jerk Trajectory

The core of the smooth motion generation is the minimum jerk trajectory algorithm. This approach generates a trajectory that minimizes the jerk (rate of change of acceleration), resulting in extremely smooth motion with zero velocity and acceleration at endpoints. This is the same type of trajectory used in high-end industrial robots.

The mathematical formula for the minimum jerk trajectory is:

```
s(t) = 10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵
```

Where:
- `s(t)` is the normalized position at time `t`
- `T` is the total duration of the motion

This trajectory has the following properties:
- `s(0) = 0`, `s(T) = 1` (starts at 0, ends at 1)
- `s'(0) = 0`, `s'(T) = 0` (zero velocity at endpoints)
- `s''(0) = 0`, `s''(T) = 0` (zero acceleration at endpoints)
- `s'''(0) ≠ 0`, `s'''(T) ≠ 0` (non-zero jerk at endpoints, but minimized overall)

The implementation in Python is as follows:

```python
def minimum_jerk(self, t):
    """Minimum jerk trajectory function.
    This produces extremely smooth motion with zero velocity and acceleration at endpoints."""
    return 10 * (t**3) - 15 * (t**4) + 6 * (t**5)
```

This function takes a normalized time value `t` (ranging from 0 to 1) and returns a normalized position value (also ranging from 0 to 1). The actual joint positions are then calculated by interpolating between the start and end positions using this normalized value.

### Key Pose Interpolation

The robot moves through a sequence of predefined key poses that define the pick and place operation. These key poses are defined as arrays of joint angles (in radians) for each of the 6 joints of the UR3 robot.

```python
self.key_poses = [
    # Home position
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    
    # Pre-grasp approach
    [0.5, -1.0, 0.5, -1.57, 0.0, 0.0],
    
    # Grasp position
    [0.5, -0.8, 0.8, -1.57, 0.0, 0.0],
    
    # Lift
    [0.5, -1.0, 0.5, -1.57, 0.0, 0.0],
    
    # Transport
    [-0.5, -1.0, 0.5, -1.57, 0.0, 0.0],
    
    # Place position
    [-0.5, -0.8, 0.8, -1.57, 0.0, 0.0],
    
    # Post-place retreat
    [-0.5, -1.0, 0.5, -1.57, 0.0, 0.0],
    
    # Return to home
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
]
```

The motion between these key poses is interpolated using the minimum jerk trajectory with precise timing control. This approach allows for complex motion sequences to be defined using a small number of key poses, while ensuring smooth transitions between them.

### Dwell Time

To eliminate potential glitches during rapid direction changes, the robot pauses at each key pose for a configurable amount of time (default: 2 seconds). This dwell time gives the robot time to settle and ensures that the motion is visually smooth and realistic.

```python
self.dwell_time = 2.0  # seconds to pause at each pose
```

During the dwell time, the robot maintains the exact joint positions of the key pose, ensuring stability and preventing any unwanted motion.

### Angle Wrapping Handling

One of the challenges in controlling revolute joints is handling angle wrapping. For example, moving from 170° to -170° should result in a 20° movement, not a 340° movement in the opposite direction. The trajectory generator properly handles angle wrapping for revolute joints to ensure the robot always takes the shortest path between poses, preventing unnecessary rotations.

```python
# Calculate the shortest path for revolute joints
diff = end_pos - start_pos
if abs(diff) > math.pi:
    if diff > 0:
        diff = diff - 2 * math.pi
    else:
        diff = diff + 2 * math.pi
    end_pos = start_pos + diff
```

This code checks if the difference between the start and end positions is greater than π radians (180°). If it is, it adjusts the end position to ensure the robot takes the shortest path.

## Control Loop Implementation

The control loop that applies the trajectory to the robot's joints is implemented as a high-frequency timer callback function. This loop runs at 100 Hz (10ms cycle time) to ensure ultra-smooth motion with frequent updates to the robot's position.

```python
def control_loop(self):
    """Main control loop for smooth robot motion"""
    with self.lock:  # Thread safety
        # Update timers
        dt = 0.01  # 10ms (100Hz)

        if self.is_dwelling:
            # We're pausing at a pose
            self.dwell_timer += dt
            if self.dwell_timer >= self.dwell_time:
                # Done dwelling, start moving to next pose
                self.is_dwelling = False
                self.motion_time = 0.0
                self.next_pose_index = (self.current_pose_index + 1) % len(self.key_poses)
        else:
            # We're moving between poses
            self.motion_time += dt

            if self.motion_time >= self.motion_duration:
                # Reached the target pose, start dwelling
                self.current_pose_index = self.next_pose_index
                self.is_dwelling = True
                self.dwell_timer = 0.0

                # Set exact pose values to avoid accumulation of floating point errors
                self.current_positions = self.key_poses[self.current_pose_index].copy()
            else:
                # Interpolate between poses
                t = self.motion_time / self.motion_duration
                smooth_t = self.minimum_jerk(t)

                # Interpolate each joint position
                for i in range(len(self.current_positions)):
                    # Calculate the shortest path for revolute joints
                    start_pos = self.key_poses[self.current_pose_index][i]
                    end_pos = self.key_poses[self.next_pose_index][i]

                    # Handle angle wrapping
                    diff = end_pos - start_pos
                    if abs(diff) > math.pi:
                        if diff > 0:
                            diff = diff - 2 * math.pi
                        else:
                            diff = diff + 2 * math.pi
                        end_pos = start_pos + diff

                    # Apply minimum jerk trajectory
                    self.current_positions[i] = start_pos + smooth_t * (end_pos - start_pos)
```

Key features of the control loop implementation include:

1. **Thread Safety**: The implementation uses thread locks to ensure thread safety when accessing shared data between the control loop and the ROS2 callback functions.

2. **Consistent Timing**: The implementation uses a fixed time increment (`dt = 0.01`) to ensure consistent timing between updates, preventing timing-related glitches.

3. **State Machine**: The control loop implements a simple state machine with two states: dwelling at a pose and moving between poses.

4. **Exact Pose Values**: The system sets exact pose values at the end of each motion segment to avoid accumulation of floating point errors over time.

5. **Smooth Interpolation**: The minimum jerk trajectory function is used to generate smooth interpolation between key poses.

6. **Angle Wrapping Handling**: The control loop properly handles angle wrapping to ensure the robot takes the shortest path between poses.

## ROS2 Architecture

The simulation uses ROS2 (Robot Operating System 2) as the underlying framework for communication, visualization, and control. This section details the ROS2 components used in the project.

### Nodes and Components

The simulation uses the following ROS2 nodes:

1. **robot_state_publisher**: A standard ROS2 node that publishes the robot's state (joint positions) to TF2. This node takes the URDF description of the robot and the current joint positions and publishes the corresponding transforms.

2. **smooth_robot_mover**: A custom Python node that generates and publishes smooth joint trajectories. This node implements the trajectory generation algorithms described above and publishes the resulting joint positions to the `/joint_states` topic.

The relationship between these nodes is illustrated in the following diagram:

```
                  +-------------------+
                  |                   |
                  | smooth_robot_mover|
                  |                   |
                  +--------+----------+
                           |
                           | /joint_states
                           v
+---------------+    +-----+------+    +------------+
|               |    |            |    |            |
| URDF          |--->| robot_state|    | RViz       |
| Description   |    | publisher  |--->| Visualization|
|               |    |            |    |            |
+---------------+    +------------+    +------------+
                           |
                           | /tf, /tf_static
                           v
                     +------------+
                     |            |
                     | TF Tree    |
                     |            |
                     +------------+
```

### Topics and Messages

The simulation uses the following ROS2 topics and message types:

1. **/joint_states** (sensor_msgs/JointState): Joint positions published by the smooth_robot_mover node. This topic contains the current position of each joint of the robot.

2. **/robot_description** (std_msgs/String): URDF description of the robot, published as a string parameter. This description is used by the robot_state_publisher to generate the TF transforms.

3. **/tf** and **/tf_static** (tf2_msgs/TFMessage): Transform frames published by the robot_state_publisher. These topics contain the transforms between the different links of the robot, based on the joint positions and the URDF description.

The JointState message structure is as follows:

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort
```

In this project, only the `name` and `position` fields are used, as the simulation focuses on kinematic motion rather than dynamics.

### Parameters

The simulation uses the following ROS2 parameters:

1. **robot_description**: URDF string parameter used by the robot_state_publisher. This parameter contains the complete URDF description of the robot, including all links, joints, visual meshes, collision meshes, and inertial properties.

2. **use_sim_time**: Boolean parameter that indicates whether to use simulation time or real time. In this project, real time is used, so this parameter is set to `False`.

These parameters are set when launching the nodes, either through command-line arguments or through a launch file.

## Visualization in RViz

The robot is visualized in RViz, which provides a 3D view of the robot and its environment. RViz is a powerful visualization tool that is part of the ROS ecosystem and is widely used for robot visualization and debugging.

The visualization includes:

1. **Robot Model**: The 3D mesh models of the robot's links, loaded from the URDF description.

2. **TF Frames**: The coordinate frames of each link, visualized as axes or arrows.

3. **Joint State Information**: The current position of each joint, visualized through the robot model.

RViz is configured through a configuration file (`simple_config.rviz`) that specifies which displays to show and how to configure them. The key displays used in this project are:

1. **RobotModel**: Displays the robot model based on the URDF description and the current joint states.

2. **TF**: Displays the transform frames of the robot.

3. **Grid**: Displays a reference grid to help with spatial orientation.

## Pick and Place Sequence

The simulated pick and place operation consists of the following steps:

1. **Home Position**: The robot starts in its home position, with all joints at their zero positions except for the shoulder lift and wrist 1 joints, which are at -90 degrees (-1.57 radians).

2. **Pre-Grasp Approach**: The robot moves to a position above the object, with the gripper open and aligned with the object.

3. **Grasp Position**: The robot lowers to the object's position, positioning the gripper around the object.

4. **Grasp**: The robot closes its gripper to grasp the object. In this simulation, the grasp is simulated by a rotation of the wrist 3 joint.

5. **Lift**: The robot lifts the object by moving upward, away from the surface.

6. **Transport**: The robot moves the object to the target location, maintaining the grasp.

7. **Place Position**: The robot lowers the object to the target surface, positioning it at the desired location.

8. **Release**: The robot opens its gripper to release the object. Again, this is simulated by a rotation of the wrist 3 joint.

9. **Post-Place Retreat**: The robot lifts away from the placed object, ensuring it doesn't collide with the object.

10. **Return to Home**: The robot returns to its home position, completing the pick and place cycle.

This sequence is defined through the key poses in the `smooth_robot_mover.py` script and is executed continuously, allowing the robot to repeatedly perform the pick and place operation.

## Executable Script Commands

To run the UR3 robot pick and place simulation, several executable script commands are provided. These commands allow you to start the simulation, visualize the robot, and control its behavior.

### Basic Simulation Launch

The most basic way to launch the simulation is using the provided launch script:

```bash
cd ~/ros2_workspaces
./launch_ur3_mesh_visualization.sh
```

This script starts all the necessary components: the robot state publisher, the smooth robot mover, and RViz for visualization.

### Running Components Individually

You can also run each component individually for more control:

Terminal 1 (Robot State Publisher):
```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ur3_absolute_paths_direct.urdf.xacro)"
```

Terminal 2 (Robot Mover):
```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
python3.12 smooth_robot_mover.py
```

Terminal 3 (RViz):
```bash
cd ~/ros2_workspaces
source /opt/ros/jazzy/setup.bash
rviz2 -d simple_config.rviz
```

### Customizing the Simulation

You can customize the simulation by modifying the parameters in the `smooth_robot_mover.py` script:

```bash
# Edit the script
nano ~/ros2_workspaces/smooth_robot_mover.py

# After editing, run the simulation
cd ~/ros2_workspaces
./launch_ur3_mesh_visualization.sh
```

Key parameters you might want to modify include:

- `self.key_poses`: The sequence of joint positions that define the pick and place operation
- `self.motion_duration`: The time (in seconds) to move between poses
- `self.dwell_time`: The time (in seconds) to pause at each pose

### Monitoring ROS2 Topics

You can monitor the ROS2 topics to see what's happening in the simulation:

```bash
# Monitor joint states
ros2 topic echo /joint_states

# List all available topics
ros2 topic list

# Get information about a topic
ros2 topic info /joint_states

# Monitor the TF tree
ros2 run tf2_tools view_frames
```

### Recording the Simulation

You can record the simulation as a video using ffmpeg:

```bash
# Install ffmpeg if not already installed
sudo apt install ffmpeg

# Record the screen
ffmpeg -f x11grab -s 1920x1080 -i :0.0 -r 30 -c:v libx264 -preset ultrafast -crf 0 output.mp4
```

### Checking URDF Validity

You can check the validity of the URDF file using the check_urdf tool:

```bash
# Extract the URDF from the xacro file
xacro ur3_absolute_paths_direct.urdf.xacro > /tmp/ur3.urdf

# Check the URDF
check_urdf /tmp/ur3.urdf
```

These executable script commands provide a comprehensive set of tools for running, customizing, and debugging the UR3 robot pick and place simulation.

---

This report provides a detailed explanation of the motion planning, trajectory generation, and ROS2 architecture used in the UR3 Robot Pick and Place Simulation project. The next report will cover implementation details, setup instructions, and troubleshooting.
