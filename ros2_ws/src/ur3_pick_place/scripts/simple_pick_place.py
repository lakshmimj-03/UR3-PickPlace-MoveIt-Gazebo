#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import time
import math
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')

        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Create a publisher for the cube marker
        self.marker_publisher = self.create_publisher(Marker, '/visualization_marker', 10)

        # Create a TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Joint names for the UR3 robot
        self.joint_names = [
            'ur3_shoulder_pan_joint',
            'ur3_shoulder_lift_joint',
            'ur3_elbow_joint',
            'ur3_wrist_1_joint',
            'ur3_wrist_2_joint',
            'ur3_wrist_3_joint'
        ]

        self.get_logger().info('Pick and place demo initialized')

        # Create a cube marker to represent the object to pick
        self.create_cube_marker()

        # Define pick and place positions
        self.pick_position = [0.3, 0.0, 0.05]  # x, y, z in meters
        self.place_position = [0.3, 0.2, 0.05]  # x, y, z in meters

        # Current joint positions
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Create a timer to continuously publish joint states
        self.create_timer(0.01, self.publish_joint_states)

        # Start the demo after a short delay
        self.create_timer(2.0, self.run_demo)

    def create_cube_marker(self):
        """Create a cube marker to represent the object to pick"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_demo"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Set the position of the cube
        marker.pose.position.x = 0.3
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0

        # Set the scale of the cube
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Set the color of the cube (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        self.marker_publisher.publish(marker)
        self.get_logger().info('Published cube marker')

    def update_cube_position(self, position):
        """Update the position of the cube marker"""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_demo"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.MODIFY

        # Set the position of the cube
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0

        # Set the scale of the cube
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        # Set the color of the cube (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Publish the marker
        self.marker_publisher.publish(marker)

    def publish_joint_states(self):
        """Continuously publish the current joint states"""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_joint_positions
        self.joint_state_publisher.publish(joint_state_msg)

    def move_to_joint_positions(self, positions, duration_sec=2.0):
        """Move the robot to the specified joint positions by publishing to joint_states"""
        self.get_logger().info(f'Moving to joint positions: {positions}')

        # Calculate number of steps for smooth movement
        steps = 50
        start_time = time.time()
        end_time = start_time + duration_sec

        # Store the current joint positions
        current_positions = self.current_joint_positions.copy()

        # Calculate step size for each joint
        step_sizes = [(positions[i] - current_positions[i]) / steps for i in range(len(positions))]

        # Publish joint states in a loop to create smooth motion
        for step in range(steps + 1):
            # Calculate intermediate positions
            intermediate_positions = [current_positions[i] + step * step_sizes[i] for i in range(len(positions))]

            # Update the current joint positions
            self.current_joint_positions = intermediate_positions

            # Sleep to control the speed
            time.sleep(duration_sec / steps)

        # Ensure final position is exactly what was requested
        self.current_joint_positions = positions

        self.get_logger().info('Movement completed')
        return True

    def run_demo(self):
        """Run the pick and place demo"""
        self.get_logger().info('Starting pick and place demo')

        # Move to home position
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.move_to_joint_positions(home_position)

        # Move to pre-grasp position
        pre_grasp = [0.0, -1.0, 0.5, -1.0, -1.57, 0.0]
        self.move_to_joint_positions(pre_grasp)

        # Move to grasp position
        grasp = [0.0, -0.8, 0.7, -1.4, -1.57, 0.0]
        self.move_to_joint_positions(grasp)

        # Simulate grasping the object
        self.get_logger().info('Grasping object')
        time.sleep(1.0)

        # Update the cube position to be at the end effector
        self.update_cube_position([0.3, 0.0, 0.15])

        # Move back to pre-grasp position with the object
        self.move_to_joint_positions(pre_grasp)

        # Move to pre-place position
        pre_place = [0.7, -1.0, 0.5, -1.0, -1.57, 0.0]
        self.move_to_joint_positions(pre_place)

        # Move to place position
        place = [0.7, -0.8, 0.7, -1.4, -1.57, 0.0]
        self.move_to_joint_positions(place)

        # Simulate releasing the object
        self.get_logger().info('Releasing object')
        time.sleep(1.0)

        # Update the cube position to the place location
        self.update_cube_position([0.3, 0.2, 0.05])

        # Move back to pre-place position
        self.move_to_joint_positions(pre_place)

        # Move back to home position
        self.move_to_joint_positions(home_position)

        self.get_logger().info('Pick and place demo completed')

def main(args=None):
    rclpy.init(args=args)

    pick_place_demo = PickAndPlaceDemo()

    try:
        rclpy.spin(pick_place_demo)
    except KeyboardInterrupt:
        pass
    finally:
        pick_place_demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
