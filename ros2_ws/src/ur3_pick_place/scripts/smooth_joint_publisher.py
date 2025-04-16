#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
import numpy as np

class SmoothJointPublisher(Node):
    def __init__(self):
        super().__init__('smooth_joint_publisher')
        
        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Joint names for the UR3 robot
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Initial joint positions
        self.current_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Target positions for smooth interpolation
        self.target_positions = self.current_positions.copy()
        
        # Motion parameters
        self.motion_duration = 5.0  # seconds for a complete motion
        self.update_rate = 50.0  # Hz
        self.motion_start_time = self.get_clock().now()
        self.is_moving = False
        
        # Create a timer to publish joint states at high frequency
        self.create_timer(1.0/self.update_rate, self.publish_joint_states)
        
        # Create a timer to update target positions periodically
        self.create_timer(self.motion_duration, self.update_target_positions)
        
        # Define a set of poses to cycle through
        self.poses = [
            [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],  # Home position
            [0.5, -1.0, 0.5, -1.0, 0.5, 0.5],    # Position 1
            [-0.5, -1.2, 1.0, -1.5, -0.5, 0.0],  # Position 2
            [0.0, -0.8, 0.8, -0.8, 0.0, 0.3],    # Position 3
            [0.8, -1.4, 0.2, -0.5, 0.7, -0.3]    # Position 4
        ]
        self.current_pose_index = 0
        
        self.get_logger().info('Smooth joint publisher initialized')
    
    def publish_joint_states(self):
        """Publish the current joint states with smooth interpolation"""
        # If we're in motion, interpolate between current and target positions
        if self.is_moving:
            # Calculate how far we are through the motion (0.0 to 1.0)
            current_time = self.get_clock().now()
            elapsed = (current_time - self.motion_start_time).nanoseconds / 1e9
            
            if elapsed >= self.motion_duration:
                # Motion complete
                self.current_positions = self.target_positions.copy()
                self.is_moving = False
            else:
                # Smooth interpolation using sine function for acceleration/deceleration
                progress = elapsed / self.motion_duration
                # Use sine-based easing function for smooth acceleration/deceleration
                smooth_factor = 0.5 - 0.5 * math.cos(progress * math.pi)
                
                # Interpolate each joint position
                for i in range(len(self.joint_names)):
                    start_pos = self.current_positions[i]
                    end_pos = self.target_positions[i]
                    self.current_positions[i] = start_pos + (end_pos - start_pos) * smooth_factor
        
        # Create and publish joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_positions
        
        self.joint_state_publisher.publish(joint_state_msg)
    
    def update_target_positions(self):
        """Update the target positions to create a continuous motion"""
        # Move to the next pose in the sequence
        self.current_pose_index = (self.current_pose_index + 1) % len(self.poses)
        self.target_positions = self.poses[self.current_pose_index]
        
        # Start the motion
        self.motion_start_time = self.get_clock().now()
        self.is_moving = True
        
        self.get_logger().info(f'Moving to pose {self.current_pose_index}: {self.target_positions}')

def main(args=None):
    rclpy.init(args=args)
    
    smooth_joint_publisher = SmoothJointPublisher()
    
    try:
        rclpy.spin(smooth_joint_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        smooth_joint_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
