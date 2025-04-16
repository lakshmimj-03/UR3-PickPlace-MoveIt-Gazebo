#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, Vector3
from std_msgs.msg import ColorRGBA
import math
import numpy as np
from tf2_ros import TransformListener, Buffer
from rclpy.time import Time

class UR3ColorEnhancer(Node):
    def __init__(self):
        super().__init__('ur3_color_enhancer')

        # Create a publisher for the marker array
        self.marker_pub = self.create_publisher(MarkerArray, '/ur3_color_enhancements', 10)

        # Create a timer to publish the markers
        self.timer = self.create_timer(0.1, self.publish_markers)

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize time counter
        self.t = 0.0

        self.get_logger().info('UR3 Color Enhancer initialized')

    def publish_markers(self):
        marker_array = MarkerArray()

        # Add silver body parts
        self.add_silver_parts(marker_array)

        # Add blue joint caps
        self.add_blue_joint_caps(marker_array)

        # Publish the marker array
        self.marker_pub.publish(marker_array)

        # Update time counter
        self.t += 0.05

    def add_silver_parts(self, marker_array):
        # Main body parts
        links = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link"
        ]

        # Dimensions for each link (approximate)
        dimensions = [
            Vector3(x=0.12, y=0.12, z=0.05),  # base
            Vector3(x=0.08, y=0.08, z=0.1),   # shoulder
            Vector3(x=0.08, y=0.08, z=0.25),  # upper_arm
            Vector3(x=0.06, y=0.06, z=0.2),   # forearm
            Vector3(x=0.05, y=0.05, z=0.08),  # wrist_1
            Vector3(x=0.05, y=0.05, z=0.08),  # wrist_2
            Vector3(x=0.05, y=0.05, z=0.08)   # wrist_3
        ]

        # Shapes for each link
        shapes = [
            Marker.CYLINDER,  # base
            Marker.CYLINDER,  # shoulder
            Marker.CYLINDER,  # upper_arm
            Marker.CYLINDER,  # forearm
            Marker.CYLINDER,  # wrist_1
            Marker.CYLINDER,  # wrist_2
            Marker.CYLINDER   # wrist_3
        ]

        for i, link in enumerate(links):
            marker = Marker()
            marker.header.frame_id = link
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "silver_parts"
            marker.id = i
            marker.type = shapes[i]
            marker.action = Marker.ADD

            # Set the position
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale
            marker.scale = dimensions[i]

            # Set the color (silver)
            marker.color.r = 0.85
            marker.color.g = 0.85
            marker.color.b = 0.87
            marker.color.a = 0.9  # Semi-transparent to overlay on the robot

            marker_array.markers.append(marker)

    def add_blue_joint_caps(self, marker_array):
        # Joint positions
        joints = [
            "shoulder_link",
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link"
        ]

        for i, joint in enumerate(joints):
            marker = Marker()
            marker.header.frame_id = joint
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "blue_caps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Set the position
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0

            # Set the scale (size of the sphere)
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.03

            # Set the color (UR blue)
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 0.9
            marker.color.a = 1.0

            marker_array.markers.append(marker)

def main(args=None):
    rclpy.init(args=args)
    enhancer = UR3ColorEnhancer()
    rclpy.spin(enhancer)
    enhancer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
