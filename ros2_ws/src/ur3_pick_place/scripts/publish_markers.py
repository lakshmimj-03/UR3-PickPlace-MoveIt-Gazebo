#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        
        self.publisher_ = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
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
        
        self.get_logger().info('Marker publisher initialized')
    
    def timer_callback(self):
        marker_array = MarkerArray()
        
        # Get current time for all markers
        now = self.get_clock().now().to_msg()
        
        # Red cube marker
        red_cube_marker = Marker()
        red_cube_marker.header.frame_id = 'base_link'
        red_cube_marker.header.stamp = now
        red_cube_marker.ns = 'objects'
        red_cube_marker.id = 1
        red_cube_marker.type = Marker.CUBE
        red_cube_marker.action = Marker.ADD
        red_cube_marker.pose = self.red_cube_pose
        red_cube_marker.scale.x = 0.1
        red_cube_marker.scale.y = 0.1
        red_cube_marker.scale.z = 0.1
        red_cube_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        marker_array.markers.append(red_cube_marker)
        
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
        place_marker.scale.x = 0.2
        place_marker.scale.y = 0.2
        place_marker.scale.z = 0.02
        place_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        marker_array.markers.append(place_marker)
        
        # Ground plane marker
        ground_marker = Marker()
        ground_marker.header.frame_id = 'base_link'
        ground_marker.header.stamp = now
        ground_marker.ns = 'objects'
        ground_marker.id = 4
        ground_marker.type = Marker.CUBE
        ground_marker.action = Marker.ADD
        ground_marker.pose.position.x = 0.0
        ground_marker.pose.position.y = 0.0
        ground_marker.pose.position.z = -0.01
        ground_marker.pose.orientation.w = 1.0
        ground_marker.scale.x = 2.0
        ground_marker.scale.y = 2.0
        ground_marker.scale.z = 0.01
        ground_marker.color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.5)
        marker_array.markers.append(ground_marker)
        
        self.publisher_.publish(marker_array)
        self.get_logger().info('Published markers')

def main(args=None):
    rclpy.init(args=args)
    marker_publisher = MarkerPublisher()
    rclpy.spin(marker_publisher)
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
