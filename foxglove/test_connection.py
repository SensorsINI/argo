#!/usr/bin/env python3
"""
Test script to verify Foxglove connection
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('foxglove_test_publisher')
        self.publisher = self.create_publisher(String, 'foxglove_test', 10)
        self.timer = self.create_timer(1.0, self.publish_test_message)
        self.counter = 0
        
    def publish_test_message(self):
        msg = String()
        msg.data = f'Foxglove test message {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
