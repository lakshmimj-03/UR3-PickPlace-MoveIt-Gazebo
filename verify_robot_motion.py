#!/usr/bin/env python3

import time
import subprocess

# Define a simple motion sequence
positions = [
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home position
    [0.5, -1.0, 0.5, -1.0, 0.5, 0.5],     # Position 1
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]    # Back to home position
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
    print(f"Running command: {cmd}")
    subprocess.run(cmd, shell=True)

# Main function
def main():
    print("Verifying robot motion...")

    # Publish each position in the sequence
    for i, pos in enumerate(positions):
        print(f"Publishing position {i}: {pos}")
        publish_joint_states(pos)
        time.sleep(3)  # Wait for 3 seconds between positions

    print("Verification complete!")

if __name__ == "__main__":
    main()
