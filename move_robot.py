#!/usr/bin/env python3.12

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')

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

        # Initialize current and target waypoints
        self.current_waypoint_index = 0
        self.next_waypoint_index = 1

        # Current joint positions (start at home position)
        self.current_positions = self.waypoints[0].copy()

        # Interpolation parameters
        self.interpolation_steps = 300  # More steps for smoother motion
        self.current_step = 0
        self.is_moving = False

        # Create a timer to publish joint states (50 Hz for smoother motion)
        self.create_timer(1.0/50.0, self.publish_joint_states)

        # Create a timer to start moving to the next waypoint (every 5 seconds)
        self.create_timer(5.0, self.start_next_movement)

        self.get_logger().info('Robot mover initialized')

    def publish_joint_states(self):
        """Publish the current joint states with smooth interpolation"""
        # If we're in motion, interpolate between current and target positions
        if self.is_moving and self.current_step < self.interpolation_steps:
            # Calculate interpolation factor (0 to 1) with smooth acceleration/deceleration
            t = self.current_step / self.interpolation_steps

            # Use smooth step function for acceleration/deceleration
            smooth_t = self.smooth_step(t)  # Smooth step function

            # Interpolate each joint position
            for i in range(len(self.current_positions)):
                start_pos = self.waypoints[self.current_waypoint_index][i]
                end_pos = self.waypoints[self.next_waypoint_index][i]

                # Calculate the shortest path for revolute joints (considering -2π to 2π range)
                diff = end_pos - start_pos
                if abs(diff) > math.pi:
                    # Take the shorter path around the circle
                    if diff > 0:
                        diff = diff - 2 * math.pi
                    else:
                        diff = diff + 2 * math.pi
                    end_pos = start_pos + diff

                self.current_positions[i] = start_pos + smooth_t * (end_pos - start_pos)

            self.current_step += 1

            # If we've reached the target, update indices
            if self.current_step >= self.interpolation_steps:
                self.current_waypoint_index = self.next_waypoint_index
                self.next_waypoint_index = (self.next_waypoint_index + 1) % len(self.waypoints)
                self.is_moving = False
                self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}')

        # Publish the current joint positions
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.current_positions
        self.joint_state_publisher.publish(joint_state_msg)

    def smooth_step(self, t):
        """Smooth step function for acceleration/deceleration"""
        # 5th degree polynomial: 6t^5 - 15t^4 + 10t^3
        # This has zero 1st and 2nd derivatives at t=0 and t=1
        return 6 * t**5 - 15 * t**4 + 10 * t**3

    def start_next_movement(self):
        """Start moving to the next waypoint"""
        if not self.is_moving:
            self.is_moving = True
            self.current_step = 0
            self.get_logger().info(f'Moving from waypoint {self.current_waypoint_index} to {self.next_waypoint_index}')

def main(args=None):
    print("Initializing ROS2...")
    rclpy.init(args=args)

    print("Creating robot mover node...")
    robot_mover = RobotMover()

    print("Spinning node...")
    try:
        rclpy.spin(robot_mover)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down...")
        robot_mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
