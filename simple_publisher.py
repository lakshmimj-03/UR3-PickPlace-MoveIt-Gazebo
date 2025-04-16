#!/usr/bin/env python3

import sys
import time

# Check Python version
print(f"Python version: {sys.version}")

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    print("ROS2 Python packages imported successfully")
except ImportError as e:
    print(f"Error importing ROS2 Python packages: {e}")
    sys.exit(1)

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.i += 1

def main(args=None):
    print("Initializing ROS2...")
    rclpy.init(args=args)
    
    print("Creating node...")
    simple_publisher = SimplePublisher()
    
    print("Spinning node...")
    try:
        rclpy.spin(simple_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        print("Shutting down...")
        simple_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
