#!/usr/bin/env python3
"""
Remote Simulator Launch Script
Launches the simulator bridge on a remote machine via SSH
"""

import subprocess
import sys
import os
import time
import signal
import argparse

# Load centralized configuration
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from load_config import load_config

# Get configuration
config = load_config()
REMOTE_HOST = config['remote']['host']
REMOTE_USER = config['remote']['user']
REMOTE_ARGO_DIR = config['remote']['argo_dir']
ROS_DOMAIN_ID = config['ros2']['domain_id']

class RemoteSimulatorLauncher:
    def __init__(self, remote_host=REMOTE_HOST, remote_user=REMOTE_USER):
        self.remote_host = remote_host
        self.remote_user = remote_user
        self.remote_process = None
        self.argo_dir = REMOTE_ARGO_DIR
        
    def check_remote_setup(self):
        """Check if remote machine has the required setup"""
        print("🔍 Checking remote machine setup...")
        
        # Check if remote machine has the argo directory
        check_cmd = [
            "ssh", f"{self.remote_user}@{self.remote_host}",
            f"test -d {self.argo_dir} && echo 'argo directory exists' || echo 'argo directory missing'"
        ]
        
        try:
            result = subprocess.run(check_cmd, capture_output=True, text=True, timeout=10)
            if "missing" in result.stdout:
                print("❌ Remote machine doesn't have argo directory")
                print(f"   Expected: {self.argo_dir}")
                print("   Solution: Copy argo directory to remote machine or use rsync")
                return False
            print("✅ Remote argo directory found")
        except subprocess.TimeoutExpired:
            print("❌ SSH connection timeout")
            return False
        except Exception as e:
            print(f"❌ SSH check failed: {e}")
            return False
            
        # Check if ROS2 is available on remote machine
        ros_check_cmd = [
            "ssh", f"{self.remote_user}@{self.remote_host}",
            "source /opt/ros/humble/setup.bash && python3 -c 'import rclpy; print(\"ROS2 available\")'"
        ]
        
        try:
            result = subprocess.run(ros_check_cmd, capture_output=True, text=True, timeout=10)
            if "ROS2 available" in result.stdout:
                print("✅ Remote machine has ROS2")
            else:
                print("❌ Remote machine missing ROS2")
                print("   Install with: sudo apt install ros-humble-desktop")
                return False
        except Exception as e:
            print(f"❌ ROS2 check failed: {e}")
            return False
            
        return True
    
    def start_remote_simulator(self):
        """Start the simulator bridge on remote machine"""
        print("🚀 Starting remote simulator bridge...")
        
        # Create the remote command
        remote_cmd = f"""
        cd {self.argo_dir} && \
        source /opt/ros/humble/setup.bash && \
        export ROS_DOMAIN_ID={ROS_DOMAIN_ID} && \
        python3 nodes/argo_simulator_bridge.py
        """
        
        # Start remote process
        ssh_cmd = [
            "ssh", f"{self.remote_user}@{self.remote_host}",
            remote_cmd
        ]
        
        try:
            self.remote_process = subprocess.Popen(
                ssh_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )
            
            print("✅ Remote simulator started")
            print("   Monitoring remote output...")
            print("   Press Ctrl+C to stop")
            print("")
            
            # Monitor output
            for line in iter(self.remote_process.stdout.readline, ''):
                print(f"[REMOTE] {line.rstrip()}")
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping remote simulator...")
            self.stop_remote_simulator()
        except Exception as e:
            print(f"❌ Failed to start remote simulator: {e}")
            return False
            
        return True
    
    def stop_remote_simulator(self):
        """Stop the remote simulator"""
        if self.remote_process:
            print("🛑 Stopping remote simulator...")
            self.remote_process.terminate()
            try:
                self.remote_process.wait(timeout=5)
                print("✅ Remote simulator stopped")
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing remote simulator...")
                self.remote_process.kill()
                self.remote_process.wait()
                print("✅ Remote simulator force stopped")
    
    def run(self):
        """Main execution"""
        if not self.check_remote_setup():
            return False
            
        return self.start_remote_simulator()

def main():
    parser = argparse.ArgumentParser(description='Launch Argo simulator on remote machine')
    parser.add_argument('--remote-host', default=REMOTE_HOST,
                       help='Remote host to run simulator on')
    parser.add_argument('--remote-user', default=REMOTE_USER,
                       help='Remote user for SSH connection')
    
    args = parser.parse_args()
    
    print("🚢 Argo Remote Simulator Launcher")
    print("=================================")
    print(f"Remote host: {args.remote_host}")
    print(f"Remote user: {args.remote_user}")
    print("")
    
    launcher = RemoteSimulatorLauncher(args.remote_host, args.remote_user)
    
    try:
        success = launcher.run()
        if not success:
            sys.exit(1)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user")
        launcher.stop_remote_simulator()
    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
