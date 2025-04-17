#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        # Set logging level to INFO for more information
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().info('Initializing visualization_node')
        
        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10)
        
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
        
        # Create a timer to publish markers continuously
        self.marker_timer = self.create_timer(0.5, self.visualize_objects)
        
        # Immediately visualize objects
        self.visualize_objects()
    
    def visualize_objects(self):
        self.get_logger().info('Visualizing objects')
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
        
        # Red cube marker
        red_cube_marker = Marker()
        red_cube_marker.header.frame_id = 'base_link'
        red_cube_marker.header.stamp = now
        red_cube_marker.ns = 'objects'
        red_cube_marker.id = 1
        red_cube_marker.type = Marker.CUBE
        red_cube_marker.action = Marker.ADD
        red_cube_marker.pose = self.red_cube_pose
        red_cube_marker.scale.x = 0.1  # Larger for better visibility
        red_cube_marker.scale.y = 0.1
        red_cube_marker.scale.z = 0.1
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
        
        # Blue cylinder marker
        blue_cylinder_marker = Marker()
        blue_cylinder_marker.header.frame_id = 'base_link'
        blue_cylinder_marker.header.stamp = now
        blue_cylinder_marker.ns = 'objects'
        blue_cylinder_marker.id = 2
        blue_cylinder_marker.type = Marker.CYLINDER
        blue_cylinder_marker.action = Marker.ADD
        blue_cylinder_marker.pose = self.blue_cylinder_pose
        blue_cylinder_marker.scale.x = 0.1
        blue_cylinder_marker.scale.y = 0.1
        blue_cylinder_marker.scale.z = 0.1
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
        
        # Place location marker
        place_marker = Marker()
        place_marker.header.frame_id = 'base_link'
        place_marker.header.stamp = now
        place_marker.ns = 'objects'
        place_marker.id = 3
        place_marker.type = Marker.CUBE
        place_marker.action = Marker.ADD
        place_marker.pose = self.place_pose
        place_marker.scale.x = 0.2
        place_marker.scale.y = 0.2
        place_marker.scale.z = 0.02
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
        
        self.marker_pub.publish(marker_array)
        self.get_logger().info('Published visualization markers')

def main():
    rclpy.init()
    node = VisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
