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
