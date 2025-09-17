#!/usr/bin/env python3
"""
Argo Desktop Status Monitor - Simple Version
===========================================

A simplified desktop notification-based status monitor for Argo ROS2 services and nodes.
This version is optimized for systems that don't support notification replacement.

Features:
- Single notification that updates less frequently
- No notification clearing (avoids compatibility issues)
- Very efficient polling (only when changes detected)
- Works with all notification systems
"""

import os
import sys
import time
import subprocess
import signal
import threading
from datetime import datetime
from typing import Dict, List, Optional, Set
import json

# Import centralized node utilities
from argo_node_utils import ArgoNodeManager

class ArgoDesktopStatusSimple:
    def __init__(self):
        self.running = True
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # Notification state
        self.last_status_hash = None
        self.notification_closed = False
        
        # ROS2 monitoring
        self.ros2_available = False
        self.monitoring_thread = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("🚢 Argo Desktop Status Monitor (Simple) starting...")
        print(f"📁 Argo directory: {self.argo_dir}")
        
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\n🛑 Received signal {signum}, shutting down...")
        self.running = False
        sys.exit(0)
    
    def _check_ros2_availability(self) -> bool:
        """Check if ROS2 is available and working"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=5)
            return result.returncode == 0
        except:
            return False
    
    def _get_service_status(self) -> Dict[str, str]:
        """Get systemd service status"""
        status = {}
        try:
            # Check argo-launch service
            result = subprocess.run(['systemctl', '--user', 'is-active', 'argo-launch.service'], 
                                  capture_output=True, text=True, timeout=3)
            status['launch'] = result.stdout.strip() if result.returncode == 0 else 'inactive'
        except:
            status['launch'] = 'unknown'
        
        # Check ROS2 recording service availability
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=3)
            status['recording'] = 'available' if '/argo/recording/start' in result.stdout else 'unavailable'
        except:
            status['recording'] = 'unavailable'
        
        return status
    
    def _get_recording_status(self) -> Dict[str, any]:
        """Get recording status from ROS2 topics"""
        try:
            # Check recording status
            result = subprocess.run(['ros2', 'topic', 'echo', '/argo/recording/status', '--once'], 
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0 and 'data: true' in result.stdout:
                return {'active': True, 'status': 'Recording'}
            else:
                return {'active': False, 'status': 'Stopped'}
        except:
            return {'active': False, 'status': 'Unknown'}
    
    def _get_system_info(self) -> Dict[str, str]:
        """Get basic system information"""
        try:
            # CPU temperature
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = int(f.read().strip()) / 1000
            cpu_temp = f"{temp:.1f}°C"
        except:
            cpu_temp = "Unknown"
        
        try:
            # Memory usage
            result = subprocess.run(['free', '-m'], capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                lines = result.stdout.split('\n')
                mem_line = lines[1].split()
                total = int(mem_line[1])
                used = int(mem_line[2])
                mem_percent = (used / total) * 100
                memory = f"{mem_percent:.1f}%"
            else:
                memory = "Unknown"
        except:
            memory = "Unknown"
        
        return {
            'cpu_temp': cpu_temp,
            'memory': memory
        }
    
    def _format_status_message(self) -> str:
        """Format the status message for notification"""
        # Get service status
        service_status = self._get_service_status()
        
        # Get node status
        node_summary = self.node_manager.get_node_summary()
        
        # Get recording status
        recording_status = self._get_recording_status()
        
        # Get system info
        system_info = self._get_system_info()
        
        # Format message
        lines = []
        lines.append("🚢 <b>Argo Status</b>")
        lines.append("")
        
        # Service status
        launch_icon = "🟢" if service_status['launch'] == 'active' else "🔴"
        lines.append(f"{launch_icon} <b>Launch:</b> {service_status['launch']}")
        
        record_icon = "🟢" if recording_status['active'] else "🔴"
        lines.append(f"{record_icon} <b>Recording:</b> {recording_status['status']}")
        
        lines.append("")
        
        # Node status
        lines.append(f"🤖 <b>Nodes:</b> {node_summary['running_nodes']}/{node_summary['total_nodes']}")
        
        if node_summary['stopped_nodes'] > 0:
            stopped_list = ', '.join(node_summary['stopped_node_list'])
            lines.append(f"   <i>Stopped: {stopped_list}</i>")
        
        lines.append("")
        
        # System info
        lines.append(f"🌡️ <b>CPU:</b> {system_info['cpu_temp']}")
        lines.append(f"💾 <b>Memory:</b> {system_info['memory']}")
        
        # Resource usage
        if node_summary['total_cpu'] > 0:
            lines.append(f"⚡ <b>Node CPU:</b> {node_summary['total_cpu']:.1f}%")
        
        # Add timestamp
        timestamp = datetime.now().strftime("%H:%M:%S")
        lines.append(f"🕐 <i>Updated: {timestamp}</i>")
        
        return "\n".join(lines)
    
    def _get_status_hash(self) -> str:
        """Get a hash of the current status for change detection"""
        try:
            service_status = self._get_service_status()
            node_summary = self.node_manager.get_node_summary()
            recording_status = self._get_recording_status()
            
            # Create a simple hash of key status indicators
            status_string = f"{service_status['launch']}_{node_summary['running_nodes']}_{recording_status['active']}"
            return str(hash(status_string))
        except:
            return "error"
    
    def _send_notification(self, message: str):
        """Send desktop notification"""
        try:
            cmd = [
                'notify-send',
                '--app-name=Argo',
                '--icon=boat',
                '--urgency=normal',
                '--expire-time=0',  # Don't auto-expire
                'Argo Status',
                message
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                return True
            else:
                print(f"Notification failed: {result.stderr}")
                return False
        except Exception as e:
            print(f"Error sending notification: {e}")
            return False
    
    def _monitor_ros2_events(self):
        """Monitor ROS2 events for efficient updates"""
        print("🔍 Starting ROS2 event monitoring...")
        
        while self.running and not self.notification_closed:
            try:
                # Check if ROS2 is available
                if not self._check_ros2_availability():
                    time.sleep(30)  # Check less frequently when ROS2 unavailable
                    continue
                
                self.ros2_available = True
                
                # Get current status hash
                current_hash = self._get_status_hash()
                
                # Only update if status has changed
                if current_hash != self.last_status_hash:
                    self.last_status_hash = current_hash
                    self._update_notification()
                
                # Check every 30 seconds (much less frequent)
                time.sleep(30)
                
            except Exception as e:
                print(f"Error in ROS2 monitoring: {e}")
                time.sleep(30)
    
    def _update_notification(self):
        """Update the notification with current status"""
        if self.notification_closed:
            return
        
        try:
            message = self._format_status_message()
            self._send_notification(message)
            print(f"📱 Status updated at {datetime.now().strftime('%H:%M:%S')}")
        except Exception as e:
            print(f"Error updating notification: {e}")
    
    def run(self):
        """Main run loop"""
        print("🚀 Starting Argo Desktop Status Monitor (Simple)...")
        
        # Initial status check
        self._update_notification()
        
        # Start ROS2 monitoring in background thread
        self.monitoring_thread = threading.Thread(target=self._monitor_ros2_events, daemon=True)
        self.monitoring_thread.start()
        
        # Main loop - just wait
        while self.running and not self.notification_closed:
            try:
                time.sleep(60)  # Check every minute
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(10)
        
        print("🛑 Argo Desktop Status Monitor stopped")


def main():
    """Main function"""
    # Check if running in desktop environment
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        print("❌ Error: No desktop environment detected")
        print("   This script requires a desktop environment with notification support")
        sys.exit(1)
    
    # Check if notify-send is available
    try:
        subprocess.run(['notify-send', '--version'], capture_output=True, timeout=2)
    except:
        print("❌ Error: notify-send not found")
        print("   Install with: sudo apt install libnotify-bin")
        sys.exit(1)
    
    # Start the monitor
    monitor = ArgoDesktopStatusSimple()
    monitor.run()


if __name__ == '__main__':
    main()
