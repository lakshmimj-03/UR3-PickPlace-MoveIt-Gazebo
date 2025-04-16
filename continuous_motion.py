#!/usr/bin/env python3

import time
import subprocess
import math

# Define waypoints for a pick and place motion
waypoints = [
    # Home position
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
    
    # Move to pre-grasp position
    [-0.5, -1.0, 0.5, -1.0, -0.5, 0.0],
    
    # Lower to grasp position
    [-0.5, -0.8, 0.8, -1.5, -0.5, 0.0],
    
    # Close gripper (simulated by wrist rotation)
    [-0.5, -0.8, 0.8, -1.5, -0.5, 0.5],
    
    # Lift object
    [-0.5, -1.0, 0.5, -1.0, -0.5, 0.5],
    
    # Move to place position
    [0.5, -1.0, 0.5, -1.0, 0.5, 0.5],
    
    # Lower to place position
    [0.5, -0.8, 0.8, -1.5, 0.5, 0.5],
    
    # Open gripper (simulated by wrist rotation)
    [0.5, -0.8, 0.8, -1.5, 0.5, 0.0],
    
    # Lift after placing
    [0.5, -1.0, 0.5, -1.0, 0.5, 0.0],
    
    # Return to home position
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
]

# Joint names for the UR3 robot
joint_names = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

# Function to publish joint states
def publish_joint_states(positions):
    # Format the positions as a string
    positions_str = ", ".join([str(p) for p in positions])
    
    # Create the command
    cmd = f"ros2 topic pub --once /joint_states sensor_msgs/msg/JointState \"{{header: {{stamp: {{sec: 0, nanosec: 0}}}}, name: {joint_names}, position: [{positions_str}], velocity: [], effort: []}}\""
    
    # Run the command
    subprocess.run(f"source /home/lachu/ros2_workspaces/ros2_ws/install/setup.bash && {cmd}", shell=True, executable="/bin/bash")

# Main loop
try:
    print("Starting continuous motion...")
    
    # Repeat the motion sequence
    while True:
        for i, waypoint in enumerate(waypoints):
            print(f"Moving to waypoint {i}: {waypoint}")
            publish_joint_states(waypoint)
            time.sleep(3)  # Wait for 3 seconds between waypoints
            
except KeyboardInterrupt:
    print("Motion stopped by user")
