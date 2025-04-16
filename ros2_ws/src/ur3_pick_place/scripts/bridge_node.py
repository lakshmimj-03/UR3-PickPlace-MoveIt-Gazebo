#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        
        # Create a publisher for joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Create a publisher for TF
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        
        # Create a subscriber for joint states from Gazebo
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/world/pick_place_world/model/ur3/joint_state',
            self.joint_state_callback,
            10)
        
        # Create a subscriber for clock
        self.clock_sub = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)
        
        self.get_logger().info('Bridge node initialized')
        
    def joint_state_callback(self, msg):
        # Forward joint states
        self.joint_state_pub.publish(msg)
        
        # Create TF messages from joint states
        tf_msg = TFMessage()
        
        # Add a transform for each joint
        for i, name in enumerate(msg.name):
            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp
            transform.header.frame_id = 'world'
            transform.child_frame_id = name
            
            # Set identity transform for now
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            
            tf_msg.transforms.append(transform)
        
        # Publish TF
        self.tf_pub.publish(tf_msg)
        
    def clock_callback(self, msg):
        # We don't need to do anything with the clock, it's already being published
        pass

def main(args=None):
    rclpy.init(args=args)
    bridge_node = BridgeNode()
    rclpy.spin(bridge_node)
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
