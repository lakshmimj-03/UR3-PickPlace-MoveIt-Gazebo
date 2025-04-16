#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class PickPlaceMotionPublisher(Node):
    def __init__(self):
        super().__init__('pick_place_motion_publisher')

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

        # Initial joint positions (home position)
        self.joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Define waypoints for a pick and place motion
        self.waypoints = [
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
        
        self.current_waypoint = 0
        self.interpolation_steps = 50  # Number of steps to interpolate between waypoints
        self.current_step = 0
        self.target_positions = self.waypoints[0]
        
        # Create a timer to publish joint states (100 Hz)
        self.create_timer(0.01, self.publish_joint_states)
        
        # Create a timer to update waypoints (slower rate)
        self.create_timer(0.5, self.update_waypoint)

        self.get_logger().info('Pick and place motion publisher initialized')

    def publish_joint_states(self):
        """Publish the current joint states"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_state_publisher.publish(joint_state_msg)
        
        # Interpolate towards the target position
        if self.current_step < self.interpolation_steps:
            for i in range(len(self.joint_positions)):
                # Linear interpolation
                self.joint_positions[i] += (self.target_positions[i] - self.joint_positions[i]) / (self.interpolation_steps - self.current_step)
            self.current_step += 1

    def update_waypoint(self):
        """Move to the next waypoint in the sequence"""
        if self.current_step >= self.interpolation_steps:
            # Move to the next waypoint
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
            self.target_positions = self.waypoints[self.current_waypoint]
            self.current_step = 0
            self.get_logger().info(f'Moving to waypoint {self.current_waypoint}')

def main(args=None):
    rclpy.init(args=args)

    pick_place_motion_publisher = PickPlaceMotionPublisher()

    try:
        rclpy.spin(pick_place_motion_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        pick_place_motion_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
