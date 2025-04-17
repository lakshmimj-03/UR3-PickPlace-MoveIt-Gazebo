# UR3 Robot Pick and Place Simulation - Report 1
## Project Overview and Robot Model

## Table of Contents
- [Introduction](#introduction)
- [Project Goals](#project-goals)
- [Robot Model Specifications](#robot-model-specifications)
- [URDF Model Details](#urdf-model-details)
- [Mesh Files](#mesh-files)
- [Joint Configuration](#joint-configuration)
- [Link Structure](#link-structure)
- [Material Properties](#material-properties)
- [Inertial Properties](#inertial-properties)

## Introduction

This report provides a comprehensive overview of the UR3 Robot Pick and Place Simulation project. The project implements a smooth and realistic simulation of a Universal Robots UR3 collaborative robotic arm performing pick and place operations. The simulation is built using ROS2 (Robot Operating System 2) and visualizes the robot's movements in RViz with high-fidelity 3D mesh models.

The primary focus of this project is to achieve ultra-smooth, glitch-free robot motion while maintaining accurate visual representation using detailed mesh files. This is accomplished through advanced trajectory generation techniques and careful attention to the robot's kinematic model.

## Project Goals

The main objectives of this project are:

1. **Ultra-Smooth Motion**: Implement advanced trajectory generation techniques to achieve smooth, glitch-free robot motion with zero jerk at critical points.

2. **Realistic Visualization**: Create a visually accurate representation of the UR3 robot using high-fidelity 3D mesh models.

3. **Complete Pick and Place Cycle**: Demonstrate a full pick and place operation, including approaching an object, grasping it, lifting it, moving it to a new location, placing it down, and returning to the home position.

4. **Optimized Performance**: Ensure the simulation runs efficiently with high update rates for smooth visualization.

5. **Educational Value**: Provide a clear, well-documented example of robot simulation that can be used for learning and experimentation.

## Robot Model Specifications

The simulation uses the Universal Robots UR3 collaborative robot, which is a 6-DOF (Degrees of Freedom) robotic arm with the following specifications:

| Specification | Value |
|---------------|-------|
| **Payload** | 3 kg |
| **Reach** | 500 mm |
| **Weight** | 11 kg |
| **Footprint** | Ø128 mm |
| **Repeatability** | ±0.1 mm |
| **Power Consumption** | Approx. 100-150W during typical operation |
| **Controller Size** | 475 mm × 423 mm × 268 mm |
| **Communication** | TCP/IP 100 Mbit, Ethernet socket & Modbus TCP |

### Joint Ranges

The UR3 robot has 6 revolute joints with the following ranges:

| Joint | Range |
|-------|-------|
| Base (shoulder_pan_joint) | ±360° |
| Shoulder (shoulder_lift_joint) | ±360° |
| Elbow (elbow_joint) | ±360° |
| Wrist 1 (wrist_1_joint) | ±360° |
| Wrist 2 (wrist_2_joint) | ±360° |
| Wrist 3 (wrist_3_joint) | ±360° |

The UR3 is particularly suitable for light assembly tasks, pick and place operations, and applications requiring precise movements in confined spaces. Its compact size and versatile joint ranges make it ideal for tasks that require dexterity in limited spaces.

## URDF Model Details

The robot is defined using URDF (Unified Robot Description Format) with Xacro macros. URDF is an XML-based format used in ROS to describe the physical properties of a robot, including its kinematic structure, visual appearance, collision geometry, and inertial properties.

The URDF file includes the following key components:

1. **Link Definitions**: Each segment of the robot (base, shoulder, upper arm, forearm, wrist 1, wrist 2, wrist 3, and tool0)
2. **Joint Definitions**: The connections between links, including joint types, limits, and dynamics
3. **Visual Meshes**: High-fidelity 3D models (.dae files) for visualization
4. **Collision Meshes**: Simplified 3D models (.stl files) for collision detection
5. **Material Properties**: Colors and textures for visual appearance
6. **Inertial Properties**: Mass, center of mass, and inertia tensors for each link
7. **Joint Limits**: Position, velocity, and effort limits for each joint
8. **Joint Dynamics**: Damping and friction parameters for realistic motion

### File Structure

The URDF model is organized as follows:

```
ur3_pick_place/
├── urdf/
│   └── ur3_absolute_paths_direct.urdf.xacro  # Main URDF file
├── meshes/
│   ├── visual/                  # Visual mesh files (.dae)
│   │   ├── base.dae
│   │   ├── shoulder.dae
│   │   ├── upperarm.dae
│   │   ├── forearm.dae
│   │   ├── wrist1.dae
│   │   ├── wrist2.dae
│   │   └── wrist3.dae
│   └── collision/               # Collision mesh files (.stl)
│       ├── base.stl
│       ├── shoulder.stl
│       ├── upperarm.stl
│       ├── forearm.stl
│       ├── wrist1.stl
│       ├── wrist2.stl
│       └── wrist3.stl
```

## Mesh Files

The project uses high-quality mesh files for realistic visualization. Two types of mesh files are used:

1. **Visual Meshes (.dae)**: COLLADA (COLLAborative Design Activity) files that provide detailed visual representation with support for materials, textures, and complex geometries. These files are used for visualization in RViz.

2. **Collision Meshes (.stl)**: STereoLithography files that provide simplified geometric representations for collision detection. These meshes are typically less detailed than the visual meshes to improve computational efficiency.

The URDF file uses absolute file paths to ensure the mesh files are correctly loaded in RViz:

```xml
<mesh filename="file:///home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes/visual/base.dae"/>
```

Using absolute paths with the `file://` prefix ensures that RViz can locate and load the mesh files correctly, regardless of the current working directory or package path resolution.

## Joint Configuration

The UR3 robot has 6 revolute joints that connect the 7 links of the robot (including the base link). Each joint is defined with specific properties:

```xml
<joint name="shoulder_pan_joint" type="revolute">
  <parent link="base_link"/>
  <child link="shoulder_link"/>
  <origin xyz="0 0 0.1519" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="${-2*pi}" upper="${2*pi}" effort="150.0" velocity="3.15"/>
  <dynamics damping="0.0" friction="0.0"/>
</joint>
```

Key joint properties include:

- **name**: Unique identifier for the joint
- **type**: Type of joint (revolute for rotational joints)
- **parent/child**: Links connected by this joint
- **origin**: Position and orientation of the joint relative to the parent link
- **axis**: Axis of rotation
- **limit**: Range of motion, maximum effort, and velocity
- **dynamics**: Damping and friction parameters

## Link Structure

Each link in the URDF model represents a rigid body segment of the robot. Links are defined with visual, collision, and inertial properties:

```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="file:///home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes/visual/base.dae"/>
    </geometry>
    <material name="UR_Blue"/>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="file:///home/lachu/ros2_workspaces/ros2_ws/src/ur3_pick_place/meshes/collision/base.stl"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="4.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.00443333156" ixy="0.0" ixz="0.0" iyy="0.00443333156" iyz="0.0" izz="0.0072"/>
  </inertial>
</link>
```

The UR3 robot model includes the following links:

1. **base_link**: The stationary base of the robot
2. **shoulder_link**: The first moving segment, connected to the base
3. **upper_arm_link**: The upper arm segment
4. **forearm_link**: The forearm segment
5. **wrist_1_link**: The first wrist segment
6. **wrist_2_link**: The second wrist segment
7. **wrist_3_link**: The third wrist segment
8. **tool0**: The tool mounting point (end effector)

Each link is carefully defined with proper coordinate frames and transformations to ensure accurate kinematics.

## Material Properties

The URDF model includes material definitions to specify the visual appearance of the robot:

```xml
<material name="UR_Blue">
  <color rgba="0.1 0.1 0.8 1.0"/>
</material>
```

Materials can be defined with:

- **color**: RGBA values (red, green, blue, alpha)
- **texture**: Image files for surface textures

The UR3 robot typically uses a blue color scheme for its components, which is reflected in the material definitions.

## Inertial Properties

Inertial properties define the mass distribution of each link, which is important for dynamic simulations:

```xml
<inertial>
  <mass value="4.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.00443333156" ixy="0.0" ixz="0.0" iyy="0.00443333156" iyz="0.0" izz="0.0072"/>
</inertial>
```

Key inertial properties include:

- **mass**: The mass of the link in kilograms
- **origin**: The center of mass relative to the link frame
- **inertia**: The 3x3 inertia tensor (represented as 6 unique values due to symmetry)

Accurate inertial properties are essential for realistic dynamic behavior in physics-based simulations. While the current project focuses on kinematic simulation, these properties are included for potential future extensions to dynamic simulation environments like Gazebo.

---

This report provides a comprehensive overview of the UR3 Robot Pick and Place Simulation project, focusing on the robot model and URDF details. The next report will cover motion planning, trajectory generation, and the ROS2 architecture used in the project.
