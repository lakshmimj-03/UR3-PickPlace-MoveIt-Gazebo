#!/usr/bin/env python3

import os
import sys
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import xacro

class SpawnRobot(Node):
    def __init__(self):
        super().__init__('spawn_robot')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        # Get the URDF file path
        pkg_share = get_package_share_directory('ur3_pick_place')
        urdf_file = os.path.join(pkg_share, 'urdf', 'simple_ur3.urdf')
        
        # Read the URDF file
        self.get_logger().info(f'Loading URDF from: {urdf_file}')
        with open(urdf_file, 'r') as file:
            robot_description = file.read()
        
        # Create the request
        request = SpawnEntity.Request()
        request.name = 'ur3'
        request.xml = robot_description
        request.robot_namespace = ''
        request.reference_frame = 'world'
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.5
        
        # Send the request
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.callback)
    
    def callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        finally:
            rclpy.shutdown()

def main():
    rclpy.init()
    node = SpawnRobot()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
