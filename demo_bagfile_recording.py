#!/usr/bin/env python3

"""
Demo script showing how other nodes can interact with argo_power_control
This simulates a bagfile recording node that publishes status updates
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class BagfileRecordingDemo(Node):
    def __init__(self):
        super().__init__('bagfile_recording_demo')
        
        # Publisher for bagfile recording status
        self.bagfile_status_pub = self.create_publisher(
            Bool, 
            'argo/recording/bagfile_status', 
            10
        )
        
        # Timer to simulate recording start/stop
        self.timer = self.create_timer(10.0, self.toggle_recording_status)
        
        self.recording = False
        self.get_logger().info("Bagfile recording demo started")
        self.get_logger().info("This will simulate recording start/stop every 10 seconds")
        self.get_logger().info("Watch the green LED frequency change!")
    
    def toggle_recording_status(self):
        self.recording = not self.recording
        
        msg = Bool()
        msg.data = self.recording
        
        self.bagfile_status_pub.publish(msg)
        
        if self.recording:
            self.get_logger().info("🎬 Recording STARTED - Green LED should blink at 2Hz")
        else:
            self.get_logger().info("⏹️  Recording STOPPED - Green LED should blink at 1Hz")

def main():
    rclpy.init()
    
    try:
        demo = BagfileRecordingDemo()
        rclpy.spin(demo)
    except KeyboardInterrupt:
        demo.get_logger().info("Demo stopped by user")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()

