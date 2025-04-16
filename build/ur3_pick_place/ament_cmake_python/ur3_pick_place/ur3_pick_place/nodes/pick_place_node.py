#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import numpy as np

class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        
        # Create action client for joint trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Joint names for UR3
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        
        # Initialize demo sequence
        self.create_demo_sequence()

    def create_demo_sequence(self):
        # Example positions (in radians)
        home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        pick_position = [0.5, -1.57, -0.5, -1.57, 0.0, 0.0]
        place_position = [-0.5, -1.57, -0.5, -1.57, 0.0, 0.0]
        
        # Move to positions sequentially
        self.get_logger().info('Moving to home position')
        self.move_to_position(home_position)
        
        self.get_logger().info('Moving to pick position')
        self.move_to_position(pick_position)
        
        self.get_logger().info('Moving to place position')
        self.move_to_position(place_position)
        
        self.get_logger().info('Moving back to home position')
        self.move_to_position(home_position)

    def move_to_position(self, positions, duration=2.0):
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)
        
        trajectory.points.append(point)
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        # Send goal and wait for result
        self.trajectory_client.send_goal_async(goal)
        self.get_logger().info(f'Moving to position: {positions}')

def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()