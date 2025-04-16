#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import numpy as np

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # Create a publisher for the trajectory marker
        self.marker_pub = self.create_publisher(Marker, '/trajectory_marker', 10)

        # Create a timer to publish the trajectory
        self.timer = self.create_timer(0.1, self.publish_trajectory)

        # Initialize time counter
        self.t = 0.0

        self.get_logger().info('Trajectory visualizer initialized')

    def publish_trajectory(self):
        # Create a marker message
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the scale of the marker
        marker.scale.x = 0.015  # Line width

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Create a circular trajectory
        radius = 0.3
        height = 0.5
        points = []
        colors = []

        for i in range(100):
            angle = 2.0 * math.pi * i / 100.0 + self.t

            # Create a point
            p = Point()
            p.x = radius * math.cos(angle)
            p.y = radius * math.sin(angle)
            p.z = height + 0.15 * math.sin(4.0 * angle)

            points.append(p)

            # Create a color for this point
            c = ColorRGBA()
            c.r = 0.7 + 0.3 * math.sin(angle)
            c.g = 0.7 + 0.3 * math.cos(angle)
            c.b = 0.7 + 0.3 * math.sin(angle + math.pi/2)
            c.a = 1.0

            colors.append(c)

        # Add the points and colors to the marker
        marker.points = points
        marker.colors = colors

        # Publish the marker
        self.marker_pub.publish(marker)

        # Update time counter
        self.t += 0.05

def main(args=None):
    rclpy.init(args=args)
    visualizer = TrajectoryVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
