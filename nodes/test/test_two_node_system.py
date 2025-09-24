#!/usr/bin/env python3
# Test script for the two-node Argo control system
# Simulates sensor data and radio input to validate the system

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
import time
import math
import threading

class TwoNodeSystemTester(Node):
    """Test node that simulates sensor data and radio input to test the two-node system."""
    
    def __init__(self):
        super().__init__('two_node_system_tester')
        self.get_logger().info('Two-node system tester starting...')
        
        # Test state
        self.test_time = 0.0
        self.test_scenario = "startup"
        
        # Publishers for sensor simulation
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_gps_cog = self.create_publisher(Float64, '/gps_cog', 10)
        self.pub_gps_sog = self.create_publisher(Float64, '/gps_sog', 10)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        self.pub_wind = self.create_publisher(Vector3, '/anem_speed_angle_temp', 10)
        self.pub_radio = self.create_publisher(Vector3, '/rudder_sail_radio', 10)
        
        # Subscribers to monitor system responses
        self.create_subscription(Bool, '/human_controlled', self.human_controlled_callback, 10)
        self.create_subscription(Vector3, '/control_authority', self.control_authority_callback, 10)
        self.create_subscription(Vector3, '/rudder_sail_cmd', self.auto_cmd_callback, 10)
        self.create_subscription(Vector3, '/rudder_sail_servo', self.servo_cmd_callback, 10)
        
        # Test monitoring
        self.current_human_control = None
        self.current_auto_cmd = None
        self.current_servo_cmd = None
        
        # Test timer
        self.test_timer = self.create_timer(0.1, self.test_timer_callback)  # 10Hz
        
        # Print header
        self.print_test_header()
    
    def print_test_header(self):
        """Print test information header."""
        print("\n" + "="*80)
        print("ARGO TWO-NODE SYSTEM INTEGRATION TEST")
        print("="*80)
        print("Testing: controller.py + rudder_sail_radio.py")
        print("\nTest Scenarios:")
        print("  1. Startup with sensor data")
        print("  2. Human control simulation")
        print("  3. Transition to robot control")
        print("  4. Human override during robot control")
        print("  5. Return to robot control")
        print("\nMonitoring topics:")
        print("  /human_controlled - Control authority status")
        print("  /rudder_sail_cmd - Autonomous commands from controller.py")
        print("  /rudder_sail_servo - Final commands to hardware")
        print("="*80)
    
    def test_timer_callback(self):
        """Main test loop with different scenarios."""
        self.test_time += 0.1
        
        # Always publish sensor data
        self.publish_sensor_data()
        
        # Run test scenarios based on time
        if self.test_time < 5.0:
            self.test_scenario = "startup"
            self.test_startup()
        elif self.test_time < 10.0:
            self.test_scenario = "human_control"
            self.test_human_control()
        elif self.test_time < 15.0:
            self.test_scenario = "robot_control"
            self.test_robot_control()
        elif self.test_time < 20.0:
            self.test_scenario = "human_override"
            self.test_human_override()
        elif self.test_time < 25.0:
            self.test_scenario = "return_to_robot"
            self.test_return_to_robot()
        else:
            self.test_scenario = "complete"
            self.test_complete()
    
    def publish_sensor_data(self):
        """Publish simulated sensor data."""
        # Simulated compass heading (slowly changing)
        heading = 45.0 + 10.0 * math.sin(self.test_time * 0.1)
        
        # Pose (IMU data with compass in z)
        pose_msg = Vector3(x=0.0, y=0.0, z=heading)
        self.pub_pose.publish(pose_msg)
        
        # GPS data
        gps_cog_msg = Float64(data=heading + 5.0)  # Slightly different from compass
        self.pub_gps_cog.publish(gps_cog_msg)
        
        gps_sog_msg = Float64(data=4.5)  # 4.5 knots
        self.pub_gps_sog.publish(gps_sog_msg)
        
        # Raw compass
        compass_msg = Vector3(x=0.0, y=0.0, z=heading)
        self.pub_compass.publish(compass_msg)
        
        # Wind data
        wind_speed = 8.0 + 2.0 * math.sin(self.test_time * 0.2)
        wind_angle = 120.0 + 20.0 * math.cos(self.test_time * 0.15)
        wind_msg = Vector3(x=wind_speed, y=wind_angle, z=22.5)  # temp = 22.5°C
        self.pub_wind.publish(wind_msg)
    
    def test_startup(self):
        """Test startup with no radio input."""
        # No radio input - should default to human control
        radio_msg = Vector3(x=0.0, y=0.0, z=0.0)
        self.pub_radio.publish(radio_msg)
        
        if int(self.test_time * 2) % 2 == 0:  # Every 0.5 seconds
            print(f"[{self.test_time:5.1f}s] STARTUP: Sensor data active, no radio input")
    
    def test_human_control(self):
        """Test human control with active radio input."""
        # Simulate human moving controls
        rudder = 0.3 * math.sin(self.test_time * 2.0)
        sail = 0.2 * math.cos(self.test_time * 1.5)
        
        radio_msg = Vector3(x=rudder, y=sail, z=0.0)
        self.pub_radio.publish(radio_msg)
        
        if int(self.test_time * 2) % 2 == 0:
            print(f"[{self.test_time:5.1f}s] HUMAN: Radio active (rudder={rudder:.2f}, sail={sail:.2f})")
    
    def test_robot_control(self):
        """Test transition to robot control."""
        # Stop radio input to allow robot control
        radio_msg = Vector3(x=0.0, y=0.0, z=0.0)
        self.pub_radio.publish(radio_msg)
        
        if int(self.test_time * 2) % 2 == 0:
            print(f"[{self.test_time:5.1f}s] ROBOT: No radio input, waiting for robot control...")
    
    def test_human_override(self):
        """Test human override during robot control."""
        # Sudden human input to test override
        rudder = 0.5 if (self.test_time % 1.0) < 0.5 else -0.3
        sail = 0.4
        
        radio_msg = Vector3(x=rudder, y=sail, z=0.0)
        self.pub_radio.publish(radio_msg)
        
        if int(self.test_time * 2) % 2 == 0:
            print(f"[{self.test_time:5.1f}s] OVERRIDE: Human takes control (rudder={rudder:.2f})")
    
    def test_return_to_robot(self):
        """Test return to robot control after human activity."""
        # Stop radio input again
        radio_msg = Vector3(x=0.0, y=0.0, z=0.0)
        self.pub_radio.publish(radio_msg)
        
        if int(self.test_time * 2) % 2 == 0:
            print(f"[{self.test_time:5.1f}s] RETURN: No radio input, returning to robot control...")
    
    def test_complete(self):
        """Test complete."""
        if self.test_time == 25.1:  # First time only
            print(f"\n[{self.test_time:5.1f}s] TEST COMPLETE")
            print("="*80)
            print("SUMMARY:")
            print(f"  Final control state: {'HUMAN' if self.current_human_control else 'ROBOT'}")
            if self.current_auto_cmd:
                print(f"  Last auto command: rudder={self.current_auto_cmd.x:.3f}, sail={self.current_auto_cmd.y:.3f}")
            if self.current_servo_cmd:
                print(f"  Last servo command: rudder={self.current_servo_cmd.x:.3f}, sail={self.current_servo_cmd.y:.3f}")
            print("\nTo stop test: Ctrl+C")
            print("="*80)
    
    def human_controlled_callback(self, msg):
        """Monitor human control status."""
        if self.current_human_control != msg.data:
            self.current_human_control = msg.data
            status = "HUMAN" if msg.data else "ROBOT"
            print(f"[{self.test_time:5.1f}s] *** CONTROL AUTHORITY: {status} ***")
    
    def control_authority_callback(self, msg):
        """Monitor detailed control authority."""
        # msg.x = authority, msg.y = time_since_human, msg.z = time_since_auto
        pass  # Could add detailed monitoring if needed
    
    def auto_cmd_callback(self, msg):
        """Monitor autonomous commands from controller.py."""
        self.current_auto_cmd = msg
        if not self.current_human_control:  # Only log during robot control
            print(f"[{self.test_time:5.1f}s] AUTO_CMD: rudder={msg.x:.3f}, sail={msg.y:.3f}")
    
    def servo_cmd_callback(self, msg):
        """Monitor final servo commands to hardware."""
        self.current_servo_cmd = msg
        # Print occasionally to avoid spam
        if int(self.test_time * 4) % 8 == 0:  # Every 2 seconds
            source = "HUMAN" if self.current_human_control else "ROBOT"
            print(f"[{self.test_time:5.1f}s] SERVO_OUT ({source}): rudder={msg.x:.3f}, sail={msg.y:.3f}")

def main(args=None):
    print("Starting Argo Two-Node System Integration Test...")
    print("Make sure both nodes are running:")
    print("  1. python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo_two_node.yaml")
    print("  2. python3 nodes/controller.py --ros-args --params-file nodes/argo_two_node.yaml")
    print("\nPress Enter to start test...")
    input()
    
    rclpy.init(args=args)
    tester = None
    try:
        tester = TwoNodeSystemTester()
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\nTest stopped by user.")
    finally:
        if tester:
            tester.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
