#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math
import numpy as np

class VisualEnhancer(Node):
    def __init__(self):
        super().__init__('visual_enhancer')
        
        # Create a publisher for the marker array
        self.marker_pub = self.create_publisher(MarkerArray, '/ur3_visual_enhancements', 10)
        
        # Create a timer to publish the markers
        self.timer = self.create_timer(0.1, self.publish_markers)
        
        # Initialize time counter
        self.t = 0.0
        
        self.get_logger().info('Visual enhancer initialized')
        
    def publish_markers(self):
        marker_array = MarkerArray()
        
        # Add blue caps to the joints
        self.add_joint_caps(marker_array)
        
        # Add highlight to the tool
        self.add_tool_highlight(marker_array)
        
        # Publish the marker array
        self.marker_pub.publish(marker_array)
        
        # Update time counter
        self.t += 0.05
        
    def add_joint_caps(self, marker_array):
        # Joint positions (approximate)
        joint_positions = [
            [0.0, 0.0, 0.1],  # Base
            [0.0, 0.0, 0.15],  # Shoulder
            [0.0, 0.0, 0.4],  # Elbow
            [0.0, 0.0, 0.6],  # Wrist 1
            [0.0, 0.0, 0.7],  # Wrist 2
            [0.0, 0.0, 0.8],  # Wrist 3
        ]
        
        for i, pos in enumerate(joint_positions):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "joint_caps"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set the position
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0
            
            # Set the scale (size of the sphere)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            
            # Set the color (UR blue)
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 0.8
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
    
    def add_tool_highlight(self, marker_array):
        # Tool highlight
        marker = Marker()
        marker.header.frame_id = "tool0"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tool_highlight"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Set the position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.02
        
        # Set the color (pulsing blue)
        blue_intensity = 0.5 + 0.5 * math.sin(self.t * 2.0)
        marker.color.r = 0.0
        marker.color.g = 0.3 * blue_intensity
        marker.color.b = blue_intensity
        marker.color.a = 0.7
        
        marker_array.markers.append(marker)
        
def main(args=None):
    rclpy.init(args=args)
    enhancer = VisualEnhancer()
    rclpy.spin(enhancer)
    enhancer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
