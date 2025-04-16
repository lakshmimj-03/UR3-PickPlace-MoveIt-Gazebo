#!/bin/bash

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Function to publish joint states
publish_joint_states() {
    local pan=$1
    local lift=$2
    local elbow=$3
    local wrist1=$4
    local wrist2=$5
    local wrist3=$6
    
    echo "Publishing joint states: [$pan, $lift, $elbow, $wrist1, $wrist2, $wrist3]"
    
    ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{
        header: {stamp: {sec: 0, nanosec: 0}},
        name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
        position: [$pan, $lift, $elbow, $wrist1, $wrist2, $wrist3],
        velocity: [],
        effort: []
    }"
    
    # Wait a bit to let the robot move
    sleep 1
}

# Home position
publish_joint_states 0.0 -1.57 0.0 -1.57 0.0 0.0
sleep 2

# Move to pre-grasp position
publish_joint_states -0.5 -1.0 0.5 -1.0 -0.5 0.0
sleep 2

# Lower to grasp position
publish_joint_states -0.5 -0.8 0.8 -1.5 -0.5 0.0
sleep 2

# Close gripper (simulated by wrist rotation)
publish_joint_states -0.5 -0.8 0.8 -1.5 -0.5 0.5
sleep 2

# Lift object
publish_joint_states -0.5 -1.0 0.5 -1.0 -0.5 0.5
sleep 2

# Move to place position
publish_joint_states 0.5 -1.0 0.5 -1.0 0.5 0.5
sleep 2

# Lower to place position
publish_joint_states 0.5 -0.8 0.8 -1.5 0.5 0.5
sleep 2

# Open gripper (simulated by wrist rotation)
publish_joint_states 0.5 -0.8 0.8 -1.5 0.5 0.0
sleep 2

# Lift after placing
publish_joint_states 0.5 -1.0 0.5 -1.0 0.5 0.0
sleep 2

# Return to home position
publish_joint_states 0.0 -1.57 0.0 -1.57 0.0 0.0
sleep 2

echo "Pick and place motion completed!"
