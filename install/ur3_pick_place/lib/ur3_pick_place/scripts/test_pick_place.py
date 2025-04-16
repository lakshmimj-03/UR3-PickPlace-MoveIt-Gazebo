#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class TestPickPlaceNode(Node):
    def __init__(self):
        super().__init__('test_pick_place_node')
        
        # Create a publisher to trigger the pick and place operation
        self.trigger_pub = self.create_publisher(Bool, '/trigger_pick_place', 10)
        
        # Wait a bit for connections to establish
        self.get_logger().info('Waiting for connections...')
        time.sleep(5)
        
        # Trigger the pick and place operation
        self.trigger_pick_place()
        
    def trigger_pick_place(self):
        msg = Bool()
        msg.data = True
        self.trigger_pub.publish(msg)
        self.get_logger().info('Triggered pick and place operation')

def main():
    rclpy.init()
    node = TestPickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
