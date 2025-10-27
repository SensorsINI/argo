#!/usr/bin/env python3
"""
Test script for the Argo pause/unpause system

This script demonstrates how to use the toggle_pause service to pause and unpause
all managed Argo nodes through the lifecycle manager.

Usage:
    python3 scripts/test_pause_system.py

The script will:
1. Check if the lifecycle manager is running
2. Call the toggle_pause service to pause all nodes
3. Wait a few seconds
4. Call the toggle_pause service again to unpause all nodes
5. Show the results
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time
import sys


class PauseSystemTester(Node):
    def __init__(self):
        super().__init__('pause_system_tester')
        self.get_logger().info('Pause System Tester starting...')
        
        # Create service client for toggle_pause
        self.toggle_pause_client = self.create_client(Trigger, 'toggle_pause')
        
        # Wait for service to be available
        self.get_logger().info('Waiting for toggle_pause service...')
        if not self.toggle_pause_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('toggle_pause service not available. Is the lifecycle manager running?')
            sys.exit(1)
        
        self.get_logger().info('toggle_pause service found!')
    
    def call_toggle_pause(self):
        """Call the toggle_pause service and return the response"""
        request = Trigger.Request()
        
        self.get_logger().info('Calling toggle_pause service...')
        future = self.toggle_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Success: {response.message}')
            else:
                self.get_logger().error(f'❌ Failed: {response.message}')
            return response
        else:
            self.get_logger().error('❌ Service call timed out')
            return None


def main():
    print("🧪 Argo Pause System Test")
    print("=" * 50)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create tester node
        tester = PauseSystemTester()
        
        print("\n1️⃣  Testing pause functionality...")
        response1 = tester.call_toggle_pause()
        
        if response1 and response1.success:
            print(f"\n⏸️  Nodes paused: {response1.message}")
            print("   Waiting 5 seconds...")
            time.sleep(5)
            
            print("\n2️⃣  Testing unpause functionality...")
            response2 = tester.call_toggle_pause()
            
            if response2 and response2.success:
                print(f"\n▶️  Nodes unpaused: {response2.message}")
                print("\n✅ Pause system test completed successfully!")
            else:
                print("\n❌ Unpause test failed")
        else:
            print("\n❌ Pause test failed")
    
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
    finally:
        try:
            tester.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

