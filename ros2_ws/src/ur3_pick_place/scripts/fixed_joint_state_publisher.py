#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class FixedJointStatePublisher(Node):
    def __init__(self):
        super().__init__('fixed_joint_state_publisher')

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
        self.joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

        # Create a timer to publish joint states
        self.create_timer(0.01, self.publish_joint_states)

        # Create a timer to update joint positions
        self.create_timer(5.0, self.update_joint_positions)

        self.get_logger().info('Fixed joint state publisher initialized')

    def publish_joint_states(self):
        """Publish the current joint states"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        self.joint_state_publisher.publish(joint_state_msg)

    def update_joint_positions(self):
        """Update the joint positions to create a simple motion"""
        # Alternate between two positions
        if self.joint_positions[0] == 0.0:
            self.joint_positions = [0.5, -1.0, 0.5, -1.0, -0.5, 0.5]
            self.get_logger().info('Moving to position 1')
        else:
            self.joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            self.get_logger().info('Moving to position 2')

def main(args=None):
    rclpy.init(args=args)

    fixed_joint_state_publisher = FixedJointStatePublisher()

    try:
        rclpy.spin(fixed_joint_state_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        fixed_joint_state_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
