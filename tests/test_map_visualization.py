#!/usr/bin/env python3
"""
Test script to verify sailing area map visualization in Foxglove
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
import time

class MapVisualizationTester(Node):
    def __init__(self):
        super().__init__('map_visualization_tester')
        
        # Create publishers for boat position simulation
        self.gps_velocity_pub = self.create_publisher(Vector3, '/gps_velocity', 10)
        self.pose_pub = self.create_publisher(Vector3, '/pose', 10)
        self.human_controlled_pub = self.create_publisher(Bool, '/human_controlled', 10)
        
        # Create timer for publishing test data
        self.timer = self.create_timer(1.0, self.publish_test_data)
        
        # Test coordinates (Irchel pond center)
        self.test_lon = 8.5448386
        self.test_lat = 47.3981555
        self.heading = 0.0
        
        self.get_logger().info("Map visualization tester started")
        self.get_logger().info("Publishing test boat position at Irchel pond")
    
    def publish_test_data(self):
        """Publish simulated boat data for testing"""
        # Simulate boat movement in a small circle
        import math
        
        # Update heading (slow rotation)
        self.heading += 0.1
        if self.heading > 2 * math.pi:
            self.heading = 0.0
        
        # Calculate position offset (small circle around center)
        radius = 0.0001  # Small radius in degrees
        offset_lon = radius * math.cos(self.heading)
        offset_lat = radius * math.sin(self.heading)
        
        current_lon = self.test_lon + offset_lon
        current_lat = self.test_lat + offset_lat
        
        # Publish GPS velocity (simulated movement)
        gps_vel = Vector3()
        gps_vel.x = current_lon  # longitude
        gps_vel.y = current_lat  # latitude
        gps_vel.z = 2.0  # speed in knots
        self.gps_velocity_pub.publish(gps_vel)
        
        # Publish pose (heading)
        pose = Vector3()
        pose.x = current_lon
        pose.y = current_lat
        pose.z = math.degrees(self.heading)  # heading in degrees
        self.pose_pub.publish(pose)
        
        # Publish control status
        human_controlled = Bool()
        human_controlled.data = False  # Autonomous mode
        self.human_controlled_pub.publish(human_controlled)
        
        self.get_logger().info(f"Published test position: {current_lat:.6f}, {current_lon:.6f}, heading: {math.degrees(self.heading):.1f}°")

def main(args=None):
    rclpy.init(args=args)
    
    tester = MapVisualizationTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
