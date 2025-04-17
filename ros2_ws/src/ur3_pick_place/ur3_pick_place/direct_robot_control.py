#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster
import math
import time
import numpy as np

class DirectRobotControl(Node):
    def __init__(self):
        super().__init__('direct_robot_control')
        
        # Set logging level to INFO for more information
        self.get_logger().info('Initializing direct robot control')
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10)
        
        # TF2 broadcaster for object positions
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Object positions - moved in front of the robot for better visibility
        # Position objects relative to the base_link frame
        self.red_cube_pose = Pose(
            position=Point(x=0.4, y=0.0, z=0.05),  # Cube on the ground in front of robot
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        self.blue_cylinder_pose = Pose(
            position=Point(x=0.4, y=0.2, z=0.05),  # Cylinder on the ground to the right
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        self.place_pose = Pose(
            position=Point(x=0.4, y=-0.2, z=0.05),  # Place location on the ground to the left
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        
        # Print object positions for debugging
        self.get_logger().info(f'Red cube position: {self.red_cube_pose.position.x}, {self.red_cube_pose.position.y}, {self.red_cube_pose.position.z}')
        self.get_logger().info(f'Blue cylinder position: {self.blue_cylinder_pose.position.x}, {self.blue_cylinder_pose.position.y}, {self.blue_cylinder_pose.position.z}')
        self.get_logger().info(f'Place position: {self.place_pose.position.x}, {self.place_pose.position.y}, {self.place_pose.position.z}')
        
        # Joint names for the UR3 robot
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        
        # Home position
        self.home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        
        # Current joint positions
        self.current_joint_positions = self.home_position.copy()
        
        # Waypoints for the pick and place sequence
        self.waypoints = [
            # 0: Home position
            {'joints': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Home position'},
            
            # 1: Move above red cube
            {'joints': [0.0, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Above red cube'},
            
            # 2: Move to red cube
            {'joints': [0.0, -0.7, 0.5, -1.3, -1.57, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'At red cube'},
            
            # 3: Close gripper on red cube
            {'joints': [0.0, -0.7, 0.5, -1.3, -1.57, 0.0], 'gripper': 0.0, 'time': 1.0, 'name': 'Grasping red cube', 'attach': 'red_cube'},
            
            # 4: Lift red cube
            {'joints': [0.0, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 0.0, 'time': 2.0, 'name': 'Lifting red cube'},
            
            # 5: Move above place location
            {'joints': [-0.5, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 0.0, 'time': 2.0, 'name': 'Above place location'},
            
            # 6: Move to place location
            {'joints': [-0.5, -0.7, 0.5, -1.3, -1.57, 0.0], 'gripper': 0.0, 'time': 2.0, 'name': 'At place location'},
            
            # 7: Open gripper to release red cube
            {'joints': [-0.5, -0.7, 0.5, -1.3, -1.57, 0.0], 'gripper': 1.0, 'time': 1.0, 'name': 'Releasing red cube', 'detach': 'red_cube'},
            
            # 8: Move up from place location
            {'joints': [-0.5, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Moving up from place location'},
            
            # 9: Return to home position
            {'joints': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Back to home position'},
            
            # 10: Move above blue cylinder
            {'joints': [0.5, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Above blue cylinder'},
            
            # 11: Move to blue cylinder
            {'joints': [0.5, -0.7, 0.5, -1.3, -1.57, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'At blue cylinder'},
            
            # 12: Close gripper on blue cylinder
            {'joints': [0.5, -0.7, 0.5, -1.3, -1.57, 0.0], 'gripper': 0.0, 'time': 1.0, 'name': 'Grasping blue cylinder', 'attach': 'blue_cylinder'},
            
            # 13: Lift blue cylinder
            {'joints': [0.5, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 0.0, 'time': 2.0, 'name': 'Lifting blue cylinder'},
            
            # 14: Move above place location for cylinder
            {'joints': [-0.5, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 0.0, 'time': 2.0, 'name': 'Above place location for cylinder'},
            
            # 15: Move to place location for cylinder
            {'joints': [-0.5, -0.6, 0.5, -1.4, -1.57, 0.0], 'gripper': 0.0, 'time': 2.0, 'name': 'At place location for cylinder'},
            
            # 16: Open gripper to release blue cylinder
            {'joints': [-0.5, -0.6, 0.5, -1.4, -1.57, 0.0], 'gripper': 1.0, 'time': 1.0, 'name': 'Releasing blue cylinder', 'detach': 'blue_cylinder'},
            
            # 17: Move up from place location
            {'joints': [-0.5, -1.0, 0.5, -1.0, -1.57, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Moving up from place location'},
            
            # 18: Return to home position
            {'joints': [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], 'gripper': 1.0, 'time': 2.0, 'name': 'Back to home position'}
        ]
        
        # Current waypoint index
        self.current_waypoint = 0
        
        # Interpolation parameters
        self.interpolation_step = 0.0
        self.interpolation_steps = 50  # Number of steps for interpolation
        
        # Previous joint positions (for interpolation)
        self.prev_joint_positions = self.home_position.copy()
        
        # Target joint positions (for interpolation)
        self.target_joint_positions = self.home_position.copy()
        
        # Gripper state (0.0 is closed, 1.0 is open)
        self.gripper_state = 1.0  # Start with open gripper
        self.target_gripper_state = 1.0
        
        # Attached object (None, 'red_cube', or 'blue_cylinder')
        self.attached_object = None
        
        # Visualize objects
        self.visualize_objects()
        
        # Create a timer to publish joint states and markers
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz
        
        # Start the pick and place sequence after a delay
        self.get_logger().info('Starting pick and place demo in 3 seconds...')
        self.create_timer(3.0, self.start_sequence)
    
    def start_sequence(self):
        """Start the pick and place sequence"""
        self.get_logger().info('Starting pick and place sequence')
        self.current_waypoint = 0
        self.interpolation_step = 0.0
        self.prev_joint_positions = self.current_joint_positions.copy()
        self.target_joint_positions = self.waypoints[0]['joints']
        self.target_gripper_state = self.waypoints[0]['gripper']
    
    def update(self):
        """Update the robot state"""
        # Interpolate between waypoints
        if self.interpolation_step < 1.0:
            # Interpolate joint positions
            for i in range(len(self.current_joint_positions)):
                self.current_joint_positions[i] = self.prev_joint_positions[i] + self.interpolation_step * (self.target_joint_positions[i] - self.prev_joint_positions[i])
            
            # Interpolate gripper state
            self.gripper_state = self.gripper_state + self.interpolation_step * (self.target_gripper_state - self.gripper_state)
            
            # Increment interpolation step
            self.interpolation_step += 1.0 / self.interpolation_steps
        else:
            # We've reached the target, move to the next waypoint
            self.interpolation_step = 0.0
            self.prev_joint_positions = self.current_joint_positions.copy()
            
            # Check for attach/detach operations
            if 'attach' in self.waypoints[self.current_waypoint]:
                self.attached_object = self.waypoints[self.current_waypoint]['attach']
                self.get_logger().info(f"Attached {self.attached_object}")
            
            if 'detach' in self.waypoints[self.current_waypoint]:
                # Update the position of the object being detached
                if self.attached_object == 'red_cube':
                    # Update red cube position to be at the place location
                    self.red_cube_pose.position.x = self.place_pose.position.x
                    self.red_cube_pose.position.y = self.place_pose.position.y
                    self.red_cube_pose.position.z = self.place_pose.position.z + 0.04  # Half the height of the cube
                elif self.attached_object == 'blue_cylinder':
                    # Update blue cylinder position to be on top of the red cube
                    self.blue_cylinder_pose.position.x = self.place_pose.position.x
                    self.blue_cylinder_pose.position.y = self.place_pose.position.y
                    self.blue_cylinder_pose.position.z = self.place_pose.position.z + 0.12  # Height of cube + half height of cylinder
                
                self.attached_object = None
                self.get_logger().info("Detached object")
            
            # Move to the next waypoint
            self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
            self.target_joint_positions = self.waypoints[self.current_waypoint]['joints']
            self.target_gripper_state = self.waypoints[self.current_waypoint]['gripper']
            self.get_logger().info(f"Moving to waypoint {self.current_waypoint}: {self.waypoints[self.current_waypoint]['name']}")
        
        # Publish joint states
        self.publish_joint_states()
        
        # Visualize objects
        self.visualize_objects()
    
    def publish_joint_states(self):
        """Publish the current joint states"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.current_joint_positions
        
        self.joint_state_pub.publish(joint_state)
    
    def visualize_objects(self):
        """Visualize objects in RViz"""
        marker_array = MarkerArray()
        
        # Get current time for all markers
        now = self.get_clock().now().to_msg()
        
        # Create a text marker to show the frame
        text_marker = Marker()
        text_marker.header.frame_id = 'base_link'
        text_marker.header.stamp = now
        text_marker.ns = 'objects'
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 0.3
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.05  # Text height
        text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
        text_marker.text = "UR3 Pick and Place Demo"
        marker_array.markers.append(text_marker)
        
        # Status text marker
        status_marker = Marker()
        status_marker.header.frame_id = 'base_link'
        status_marker.header.stamp = now
        status_marker.ns = 'objects'
        status_marker.id = 9
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = 0.0
        status_marker.pose.position.y = 0.0
        status_marker.pose.position.z = 0.25
        status_marker.pose.orientation.w = 1.0
        status_marker.scale.z = 0.04  # Text height
        status_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
        status_marker.text = f"Step: {self.waypoints[self.current_waypoint]['name']}"
        marker_array.markers.append(status_marker)
        
        # Only show objects that are not attached to the gripper
        if self.attached_object != 'red_cube':
            # Red cube marker
            red_cube_marker = Marker()
            red_cube_marker.header.frame_id = 'base_link'
            red_cube_marker.header.stamp = now
            red_cube_marker.ns = 'objects'
            red_cube_marker.id = 1
            red_cube_marker.type = Marker.CUBE
            red_cube_marker.action = Marker.ADD
            red_cube_marker.pose = self.red_cube_pose
            red_cube_marker.scale.x = 0.08  # Larger for better visibility
            red_cube_marker.scale.y = 0.08
            red_cube_marker.scale.z = 0.08
            red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Fully opaque
            marker_array.markers.append(red_cube_marker)
            
            # Red cube text label
            red_text = Marker()
            red_text.header.frame_id = 'base_link'
            red_text.header.stamp = now
            red_text.ns = 'objects'
            red_text.id = 5
            red_text.type = Marker.TEXT_VIEW_FACING
            red_text.action = Marker.ADD
            red_text.pose.position.x = self.red_cube_pose.position.x
            red_text.pose.position.y = self.red_cube_pose.position.y
            red_text.pose.position.z = self.red_cube_pose.position.z + 0.1
            red_text.pose.orientation.w = 1.0
            red_text.scale.z = 0.04  # Text height
            red_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            red_text.text = "Red Cube"
            marker_array.markers.append(red_text)
        else:
            # If the red cube is attached to the gripper, show it at the end effector
            red_cube_marker = Marker()
            red_cube_marker.header.frame_id = 'wrist_3_link'
            red_cube_marker.header.stamp = now
            red_cube_marker.ns = 'objects'
            red_cube_marker.id = 1
            red_cube_marker.type = Marker.CUBE
            red_cube_marker.action = Marker.ADD
            red_cube_marker.pose.position.x = 0.0
            red_cube_marker.pose.position.y = 0.0
            red_cube_marker.pose.position.z = 0.05
            red_cube_marker.pose.orientation.w = 1.0
            red_cube_marker.scale.x = 0.08
            red_cube_marker.scale.y = 0.08
            red_cube_marker.scale.z = 0.08
            red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            marker_array.markers.append(red_cube_marker)
        
        if self.attached_object != 'blue_cylinder':
            # Blue cylinder marker
            blue_cylinder_marker = Marker()
            blue_cylinder_marker.header.frame_id = 'base_link'
            blue_cylinder_marker.header.stamp = now
            blue_cylinder_marker.ns = 'objects'
            blue_cylinder_marker.id = 2
            blue_cylinder_marker.type = Marker.CYLINDER
            blue_cylinder_marker.action = Marker.ADD
            blue_cylinder_marker.pose = self.blue_cylinder_pose
            blue_cylinder_marker.scale.x = 0.08
            blue_cylinder_marker.scale.y = 0.08
            blue_cylinder_marker.scale.z = 0.08
            blue_cylinder_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Fully opaque
            marker_array.markers.append(blue_cylinder_marker)
            
            # Blue cylinder text label
            blue_text = Marker()
            blue_text.header.frame_id = 'base_link'
            blue_text.header.stamp = now
            blue_text.ns = 'objects'
            blue_text.id = 6
            blue_text.type = Marker.TEXT_VIEW_FACING
            blue_text.action = Marker.ADD
            blue_text.pose.position.x = self.blue_cylinder_pose.position.x
            blue_text.pose.position.y = self.blue_cylinder_pose.position.y
            blue_text.pose.position.z = self.blue_cylinder_pose.position.z + 0.1
            blue_text.pose.orientation.w = 1.0
            blue_text.scale.z = 0.04  # Text height
            blue_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            blue_text.text = "Blue Cylinder"
            marker_array.markers.append(blue_text)
        else:
            # If the blue cylinder is attached to the gripper, show it at the end effector
            blue_cylinder_marker = Marker()
            blue_cylinder_marker.header.frame_id = 'wrist_3_link'
            blue_cylinder_marker.header.stamp = now
            blue_cylinder_marker.ns = 'objects'
            blue_cylinder_marker.id = 2
            blue_cylinder_marker.type = Marker.CYLINDER
            blue_cylinder_marker.action = Marker.ADD
            blue_cylinder_marker.pose.position.x = 0.0
            blue_cylinder_marker.pose.position.y = 0.0
            blue_cylinder_marker.pose.position.z = 0.05
            blue_cylinder_marker.pose.orientation.w = 1.0
            blue_cylinder_marker.scale.x = 0.08
            blue_cylinder_marker.scale.y = 0.08
            blue_cylinder_marker.scale.z = 0.08
            blue_cylinder_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            marker_array.markers.append(blue_cylinder_marker)
        
        # Place location marker
        place_marker = Marker()
        place_marker.header.frame_id = 'base_link'
        place_marker.header.stamp = now
        place_marker.ns = 'objects'
        place_marker.id = 3
        place_marker.type = Marker.CUBE
        place_marker.action = Marker.ADD
        place_marker.pose = self.place_pose
        place_marker.scale.x = 0.15
        place_marker.scale.y = 0.15
        place_marker.scale.z = 0.01
        place_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Fully opaque
        marker_array.markers.append(place_marker)
        
        # Place location text label
        place_text = Marker()
        place_text.header.frame_id = 'base_link'
        place_text.header.stamp = now
        place_text.ns = 'objects'
        place_text.id = 7
        place_text.type = Marker.TEXT_VIEW_FACING
        place_text.action = Marker.ADD
        place_text.pose.position.x = self.place_pose.position.x
        place_text.pose.position.y = self.place_pose.position.y
        place_text.pose.position.z = self.place_pose.position.z + 0.05
        place_text.pose.orientation.w = 1.0
        place_text.scale.z = 0.04  # Text height
        place_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
        place_text.text = "Place Location"
        marker_array.markers.append(place_text)
        
        # Ground plane marker for better visibility
        ground_marker = Marker()
        ground_marker.header.frame_id = 'base_link'
        ground_marker.header.stamp = now
        ground_marker.ns = 'objects'
        ground_marker.id = 4
        ground_marker.type = Marker.CUBE
        ground_marker.action = Marker.ADD
        ground_marker.pose.position.x = 0.0
        ground_marker.pose.position.y = 0.0
        ground_marker.pose.position.z = -0.01  # Just below the ground
        ground_marker.pose.orientation.w = 1.0
        ground_marker.scale.x = 2.0
        ground_marker.scale.y = 2.0
        ground_marker.scale.z = 0.01
        ground_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.5)  # Light gray, more visible
        marker_array.markers.append(ground_marker)
        
        # Gripper visualization
        gripper_marker = Marker()
        gripper_marker.header.frame_id = 'wrist_3_link'
        gripper_marker.header.stamp = now
        gripper_marker.ns = 'objects'
        gripper_marker.id = 8
        gripper_marker.type = Marker.CUBE
        gripper_marker.action = Marker.ADD
        gripper_marker.pose.position.x = 0.0
        gripper_marker.pose.position.y = 0.0
        gripper_marker.pose.position.z = 0.02
        gripper_marker.pose.orientation.w = 1.0
        gripper_marker.scale.x = 0.05
        gripper_marker.scale.y = 0.1 * (1.0 - self.gripper_state)  # Scale based on gripper state
        gripper_marker.scale.z = 0.02
        gripper_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0)  # Gray
        marker_array.markers.append(gripper_marker)
        
        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = DirectRobotControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
