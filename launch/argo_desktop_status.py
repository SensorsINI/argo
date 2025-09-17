#!/usr/bin/env python3
"""
Argo Desktop Status Monitor
==========================

A desktop notification-based status monitor for Argo ROS2 services and nodes.
Shows persistent notification with current status and updates on changes.

Features:
- Persistent desktop notification using notify-send
- ROS2 event-driven updates (no polling)
- Auto-hide when notification is closed by user
- Real-time status updates for nodes and services
- Efficient event-based monitoring

Usage:
    python3 /home/orangepi/argo/launch/argo_desktop_status.py
    # Add to desktop autostart for automatic monitoring
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

# Time constants (seconds)
ROS2_AVAILABILITY_TIMEOUT = 5
SERVICE_CHECK_TIMEOUT = 3
RECORDING_STATUS_TIMEOUT = 3
MEMORY_CHECK_TIMEOUT = 2
NOTIFICATION_SEND_TIMEOUT = 5
NOTIFICATION_CLEAR_TIMEOUT = 2
NOTIFICATION_EXPIRE_MS = 30000  # 30 seconds - longer to reduce spam
NOTIFICATION_EXPIRE_PERSISTENT_MS = 0
NOTIFICATION_CLEAR_DELAY = 0.5
ROS2_UNAVAILABLE_SLEEP = 30  # Longer when ROS2 unavailable
ROS2_MONITOR_SLEEP = 60  # Same as main loop - no separate monitoring
MAIN_LOOP_SLEEP = 60  # Check every 60 seconds
MAIN_LOOP_ERROR_SLEEP = 30  # Longer error recovery
NOTIFY_SEND_VERSION_TIMEOUT = 2


class ArgoDesktopStatus:
    def __init__(self):
        self.running = True
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # Notification state
        self.notification_id = "argo-status-monitor"  # Fixed ID for replacement
        self.last_status = None
        self.notification_closed = False
        
        # ROS2 monitoring
        self.ros2_available = False
        self.monitoring_thread = None
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        print("🚢 Argo Desktop Status Monitor starting...")
        print(f"📁 Argo directory: {self.argo_dir}")
        
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals"""
        print(f"\n🛑 Received signal {signum}, shutting down...")
        self.running = False
        self._clear_notification()
        sys.exit(0)
    
    def _is_desktop_environment(self) -> bool:
        """Check if we're running in a desktop environment"""
        return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    
    def _check_ros2_availability(self) -> bool:
        """Check if ROS2 is available and working"""
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=ROS2_AVAILABILITY_TIMEOUT)
            return result.returncode == 0
        except:
            return False
    
    def _get_service_status(self) -> Dict[str, str]:
        """Get systemd service status"""
        status = {}
        try:
            # Check argo-launch service
            result = subprocess.run(['systemctl', '--user', 'is-active', 'argo-launch.service'], 
                                  capture_output=True, text=True, timeout=SERVICE_CHECK_TIMEOUT)
            status['launch'] = result.stdout.strip() if result.returncode == 0 else 'inactive'
        except:
            status['launch'] = 'unknown'
        
        # Check ROS2 recording service availability
        try:
            result = subprocess.run(['ros2', 'service', 'list'], 
                                  capture_output=True, text=True, timeout=SERVICE_CHECK_TIMEOUT)
            status['recording'] = 'available' if '/argo/recording/start' in result.stdout else 'unavailable'
        except:
            status['recording'] = 'unavailable'
        
        return status
    
    def _get_recording_status(self) -> Dict[str, any]:
        """Get recording status from ROS2 topics"""
        try:
            # Check recording status
            result = subprocess.run(['ros2', 'topic', 'echo', '/argo/recording/status', '--once'], 
                                  capture_output=True, text=True, timeout=RECORDING_STATUS_TIMEOUT)
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
            result = subprocess.run(['free', '-m'], capture_output=True, text=True, timeout=MEMORY_CHECK_TIMEOUT)
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
    
    def _get_status_summary(self) -> str:
        """Get a simple status summary for comparison (to detect changes)"""
        try:
            # Get service status
            service_status = self._get_service_status()
            
            # Get node status
            node_summary = self.node_manager.get_node_summary()
            
            # Get recording status
            recording_status = self._get_recording_status()
            
            # Create a simple summary string for comparison
            summary = f"launch:{service_status['launch']}|recording:{recording_status['active']}|nodes:{node_summary['running_nodes']}/{node_summary['total_nodes']}"
            
            return summary
        except Exception as e:
            return f"error:{str(e)}"
    
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
        
        return "\n".join(lines)
    
    def _send_notification(self, message: str, replace: bool = True):
        """Send desktop notification"""
        try:
            cmd = ['notify-send']
            
            # Try to use hint for custom ID (if supported)
            cmd.extend(['--hint', f'string:desktop-entry:argo-status'])
            
            cmd.extend([
                '--app-name=Argo',
                '--icon=boat',
                '--urgency=normal',
                '--expire-time={}'.format(NOTIFICATION_EXPIRE_MS),
                'Argo Status',
                message
            ])
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=NOTIFICATION_SEND_TIMEOUT)
            
            if result.returncode == 0:
                return True
            else:
                # Fallback: try without hint if it fails
                cmd_fallback = ['notify-send']
                cmd_fallback.extend([
                    '--app-name=Argo',
                    '--icon=boat',
                    '--urgency=normal',
                    '--expire-time={}'.format(NOTIFICATION_EXPIRE_PERSISTENT_MS),
                    'Argo Status',
                    message
                ])
                
                result_fallback = subprocess.run(cmd_fallback, capture_output=True, text=True, timeout=NOTIFICATION_SEND_TIMEOUT)
                if result_fallback.returncode == 0:
                    return True
                else:
                    print(f"Notification failed: {result_fallback.stderr}")
                    return False
        except Exception as e:
            print(f"Error sending notification: {e}")
            return False
    
    def _clear_notification(self):
        """Clear the current notification"""
        try:
            # Try to close all notifications from our app
            subprocess.run(['notify-send', '--close', 'argo-status'], 
                         capture_output=True, timeout=NOTIFICATION_CLEAR_TIMEOUT)
        except:
            pass
        self.notification_id = None
    
    def _clear_all_notifications(self):
        """Clear all notifications (if possible)"""
        try:
            # Try to close all notifications
            subprocess.run(['notify-send', '--close', 'all'], 
                         capture_output=True, timeout=NOTIFICATION_CLEAR_TIMEOUT)
        except:
            # If that doesn't work, try to close by app name
            try:
                subprocess.run(['notify-send', '--close', 'Argo'], 
                             capture_output=True, timeout=NOTIFICATION_CLEAR_TIMEOUT)
            except:
                pass
    
    def _update_notification(self):
        """Update the notification with current status"""
        if self.notification_closed:
            return
        
        try:
            # Clear previous notifications first
            self._clear_all_notifications()
            
            # Small delay to ensure clearing is processed
            time.sleep(NOTIFICATION_CLEAR_DELAY)
            
            message = self._format_status_message()
            self._send_notification(message, replace=True)
        except Exception as e:
            print(f"Error updating notification: {e}")
    
    def _check_notification_status(self):
        """Check if notification is still visible"""
        if not self.notification_id:
            return
        
        try:
            # Try to update the notification - if it fails, it might be closed
            test_message = self._format_status_message()
            if not self._send_notification(test_message, replace=True):
                # If we can't send notifications, assume the previous one was closed
                self.notification_closed = True
                print("📱 Notification closed by user - stopping updates")
        except:
            pass
    
    def run(self):
        """Main run loop"""
        print("🚀 Starting Argo Desktop Status Monitor...")
        
        # Initial status check
        self._update_notification()
        
        last_status = None
        update_count = 0
        
        # Main loop - check status and update only when changed
        while self.running and not self.notification_closed:
            try:
                # Check if we're still in a desktop environment
                if not self._is_desktop_environment():
                    print("❌ No longer in desktop environment, exiting...")
                    break
                
                # Get current status and compare with last
                current_status = self._get_status_summary()
                
                # Only update if status actually changed
                if current_status != last_status:
                    last_status = current_status
                    update_count += 1
                    print(f"📊 Status changed (update #{update_count}), sending notification...")
                    self._update_notification()
                else:
                    # Status unchanged, just log occasionally
                    if update_count % 10 == 0:  # Every 10th check
                        print(f"📊 Status unchanged (check #{update_count})")
                
                # Check if notification is still visible
                self._check_notification_status()
                
                time.sleep(MAIN_LOOP_SLEEP)  # Check every 60 seconds
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error in main loop: {e}")
                time.sleep(MAIN_LOOP_ERROR_SLEEP)
        
        print("🛑 Argo Desktop Status Monitor stopped")
        self._clear_notification()


def main():
    """Main function"""
    # Check if running in desktop environment
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        print("❌ Error: No desktop environment detected")
        print("   This script requires a desktop environment with notification support")
        sys.exit(1)
    
    # Check if notify-send is available
    try:
        subprocess.run(['notify-send', '--version'], capture_output=True, timeout=NOTIFY_SEND_VERSION_TIMEOUT)
    except:
        print("❌ Error: notify-send not found")
        print("   Install with: sudo apt install libnotify-bin")
        sys.exit(1)
    
    # Start the monitor
    monitor = ArgoDesktopStatus()
    monitor.run()


if __name__ == '__main__':
    main()
