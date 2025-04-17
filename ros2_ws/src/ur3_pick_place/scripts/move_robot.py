#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
import numpy as np

class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10)

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

        # Object positions
        self.red_cube_pose = Pose(
            position=Point(x=0.4, y=0.0, z=0.05),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.blue_cylinder_pose = Pose(
            position=Point(x=0.4, y=0.2, z=0.05),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.place_pose = Pose(
            position=Point(x=0.4, y=-0.2, z=0.05),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        # Gripper state (0.0 is closed, 1.0 is open)
        self.gripper_state = 1.0

        # Attached object (None, 'red_cube', or 'blue_cylinder')
        self.attached_object = None

        # Create a timer to publish joint states and markers
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        # Create a timer to start the pick and place sequence after a delay
        self.get_logger().info('Starting pick and place demo in 3 seconds...')
        self.create_timer(3.0, self.pick_place_sequence)

    def update(self):
        """Update the robot state"""
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

    def move_joints(self, target_positions, duration=2.0):
        """Move the robot joints to the target positions over the specified duration"""
        start_positions = self.current_joint_positions.copy()
        start_time = time.time()
        end_time = start_time + duration

        # Use a smoother interpolation function (ease in/out)
        def ease_in_out(t):
            # Cubic ease in/out function: t^2 * (3-2t)
            return t * t * (3.0 - 2.0 * t)

        while time.time() < end_time:
            # Calculate interpolation factor (0 to 1)
            t = (time.time() - start_time) / duration
            t = max(0.0, min(1.0, t))  # Clamp to [0, 1]

            # Apply easing function for smoother motion
            t_smooth = ease_in_out(t)

            # Interpolate joint positions
            for i in range(len(self.current_joint_positions)):
                self.current_joint_positions[i] = start_positions[i] + t_smooth * (target_positions[i] - start_positions[i])

            # Publish joint states for immediate feedback
            self.publish_joint_states()

            # Visualize objects if we're carrying something
            if self.attached_object is not None:
                self.visualize_objects()

            # Sleep a bit (50 Hz update rate)
            time.sleep(0.02)

        # Ensure we reach the exact target
        self.current_joint_positions = target_positions.copy()

        # Final update of joint states and visualization
        self.publish_joint_states()
        self.visualize_objects()

    def pick_place_sequence(self):
        """Execute the pick and place sequence"""
        # Move to home position
        self.get_logger().info('Moving to home position')
        self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

        # PICK RED CUBE
        # -----------------------------
        # Move above red cube
        self.get_logger().info('Moving above red cube')
        self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0])

        # Move to red cube
        self.get_logger().info('Moving to red cube')
        self.move_joints([0.0, -0.7, 0.5, -1.3, -1.57, 0.0])

        # Close gripper on red cube
        self.get_logger().info('Closing gripper on red cube')
        self.gripper_state = 0.0
        self.attached_object = 'red_cube'  # Attach the red cube to the gripper
        self.visualize_objects()  # Update visualization immediately
        time.sleep(1.0)

        # Lift red cube
        self.get_logger().info('Lifting red cube')
        self.move_joints([0.0, -1.0, 0.5, -1.0, -1.57, 0.0])

        # PLACE RED CUBE
        # -----------------------------
        # Move above place location
        self.get_logger().info('Moving above place location')
        self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

        # Move to place location
        self.get_logger().info('Moving to place location')
        self.move_joints([-0.5, -0.7, 0.5, -1.3, -1.57, 0.0])

        # Open gripper to release red cube
        self.get_logger().info('Opening gripper to release red cube')
        self.gripper_state = 1.0

        # Update red cube position to the place location
        self.red_cube_pose.position.x = self.place_pose.position.x
        self.red_cube_pose.position.y = self.place_pose.position.y
        self.red_cube_pose.position.z = self.place_pose.position.z + 0.04  # Half the height of the cube

        self.attached_object = None  # Detach the red cube from the gripper
        self.visualize_objects()  # Update visualization immediately
        time.sleep(1.0)

        # Move up from place location
        self.get_logger().info('Moving up from place location')
        self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

        # Return to home position
        self.get_logger().info('Returning to home position')
        self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

        # PICK BLUE CYLINDER
        # -----------------------------
        # Move above blue cylinder
        self.get_logger().info('Moving above blue cylinder')
        self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

        # Move to blue cylinder
        self.get_logger().info('Moving to blue cylinder')
        self.move_joints([0.5, -0.7, 0.5, -1.3, -1.57, 0.0])

        # Close gripper on blue cylinder
        self.get_logger().info('Closing gripper on blue cylinder')
        self.gripper_state = 0.0
        self.attached_object = 'blue_cylinder'  # Attach the blue cylinder to the gripper
        self.visualize_objects()  # Update visualization immediately
        time.sleep(1.0)

        # Lift blue cylinder
        self.get_logger().info('Lifting blue cylinder')
        self.move_joints([0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

        # PLACE BLUE CYLINDER
        # -----------------------------
        # Move above place location for cylinder
        self.get_logger().info('Moving above place location for cylinder')
        self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

        # Move to place location for cylinder (on top of the red cube)
        self.get_logger().info('Moving to place location for cylinder')
        self.move_joints([-0.5, -0.6, 0.5, -1.4, -1.57, 0.0])

        # Open gripper to release blue cylinder
        self.get_logger().info('Opening gripper to release blue cylinder')
        self.gripper_state = 1.0

        # Update blue cylinder position to be on top of the red cube
        self.blue_cylinder_pose.position.x = self.place_pose.position.x
        self.blue_cylinder_pose.position.y = self.place_pose.position.y
        self.blue_cylinder_pose.position.z = self.place_pose.position.z + 0.12  # Height of cube + half height of cylinder

        self.attached_object = None  # Detach the blue cylinder from the gripper
        self.visualize_objects()  # Update visualization immediately
        time.sleep(1.0)

        # Move up from place location
        self.get_logger().info('Moving up from place location')
        self.move_joints([-0.5, -1.0, 0.5, -1.0, -1.57, 0.0])

        # Return to home position
        self.get_logger().info('Returning to home position')
        self.move_joints([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])

        self.get_logger().info('Pick and place sequence completed')

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

        # Status text marker to show current action
        status_marker = Marker()
        status_marker.header.frame_id = 'base_link'
        status_marker.header.stamp = now
        status_marker.ns = 'objects'
        status_marker.id = 10
        status_marker.type = Marker.TEXT_VIEW_FACING
        status_marker.action = Marker.ADD
        status_marker.pose.position.x = 0.0
        status_marker.pose.position.y = 0.0
        status_marker.pose.position.z = 0.25
        status_marker.pose.orientation.w = 1.0
        status_marker.scale.z = 0.05  # Text height
        status_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow
        if self.attached_object == 'red_cube':
            status_marker.text = "Carrying Red Cube"
        elif self.attached_object == 'blue_cylinder':
            status_marker.text = "Carrying Blue Cylinder"
        else:
            status_marker.text = "Gripper Empty"
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

            # Red cube text label attached to the gripper
            red_text = Marker()
            red_text.header.frame_id = 'wrist_3_link'
            red_text.header.stamp = now
            red_text.ns = 'objects'
            red_text.id = 5
            red_text.type = Marker.TEXT_VIEW_FACING
            red_text.action = Marker.ADD
            red_text.pose.position.x = 0.0
            red_text.pose.position.y = 0.0
            red_text.pose.position.z = 0.1
            red_text.pose.orientation.w = 1.0
            red_text.scale.z = 0.04  # Text height
            red_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            red_text.text = "Red Cube"
            marker_array.markers.append(red_text)

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

            # Blue cylinder text label attached to the gripper
            blue_text = Marker()
            blue_text.header.frame_id = 'wrist_3_link'
            blue_text.header.stamp = now
            blue_text.ns = 'objects'
            blue_text.id = 6
            blue_text.type = Marker.TEXT_VIEW_FACING
            blue_text.action = Marker.ADD
            blue_text.pose.position.x = 0.0
            blue_text.pose.position.y = 0.0
            blue_text.pose.position.z = 0.1
            blue_text.pose.orientation.w = 1.0
            blue_text.scale.z = 0.04  # Text height
            blue_text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            blue_text.text = "Blue Cylinder"
            marker_array.markers.append(blue_text)

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

        # Left gripper finger
        left_finger = Marker()
        left_finger.header.frame_id = 'wrist_3_link'
        left_finger.header.stamp = now
        left_finger.ns = 'objects'
        left_finger.id = 11
        left_finger.type = Marker.CUBE
        left_finger.action = Marker.ADD
        left_finger.pose.position.x = 0.0
        left_finger.pose.position.y = 0.05 * self.gripper_state  # Move based on gripper state
        left_finger.pose.position.z = 0.02
        left_finger.pose.orientation.w = 1.0
        left_finger.scale.x = 0.02
        left_finger.scale.y = 0.01
        left_finger.scale.z = 0.04
        left_finger.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Light gray
        marker_array.markers.append(left_finger)

        # Right gripper finger
        right_finger = Marker()
        right_finger.header.frame_id = 'wrist_3_link'
        right_finger.header.stamp = now
        right_finger.ns = 'objects'
        right_finger.id = 12
        right_finger.type = Marker.CUBE
        right_finger.action = Marker.ADD
        right_finger.pose.position.x = 0.0
        right_finger.pose.position.y = -0.05 * self.gripper_state  # Move based on gripper state
        right_finger.pose.position.z = 0.02
        right_finger.pose.orientation.w = 1.0
        right_finger.scale.x = 0.02
        right_finger.scale.y = 0.01
        right_finger.scale.z = 0.04
        right_finger.color = ColorRGBA(r=0.7, g=0.7, b=0.7, a=1.0)  # Light gray
        marker_array.markers.append(right_finger)

        # Current action text
        action_text = Marker()
        action_text.header.frame_id = 'base_link'
        action_text.header.stamp = now
        action_text.ns = 'objects'
        action_text.id = 13
        action_text.type = Marker.TEXT_VIEW_FACING
        action_text.action = Marker.ADD
        action_text.pose.position.x = 0.0
        action_text.pose.position.y = 0.0
        action_text.pose.position.z = 0.2
        action_text.pose.orientation.w = 1.0
        action_text.scale.z = 0.04  # Text height
        action_text.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan

        # Set the text based on the gripper state and attached object
        if self.gripper_state < 0.5:  # Gripper is closed
            if self.attached_object == 'red_cube':
                action_text.text = "Picking Red Cube"
            elif self.attached_object == 'blue_cylinder':
                action_text.text = "Picking Blue Cylinder"
            else:
                action_text.text = "Gripper Closed"
        else:  # Gripper is open
            if self.attached_object is None:
                action_text.text = "Gripper Open"
            else:
                action_text.text = "Placing Object"

        marker_array.markers.append(action_text)

        self.marker_pub.publish(marker_array)

def main():
    rclpy.init()
    node = RobotMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
