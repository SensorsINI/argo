#!/usr/bin/env python3
"""
Argo Storage Notification System
Sends GUI notifications for storage warnings
"""

import subprocess
import shutil
import time
import os
import sys
from pathlib import Path

# Recording parameters
RECORDING_RATE_MB_PER_HOUR = 13.5  # MB per hour
RECORDING_RATE_GB_PER_HOUR = RECORDING_RATE_MB_PER_HOUR / 1024.0

# Notification thresholds
CRITICAL_HOURS = 10.0  # Critical warning below 10 hours
WARNING_HOURS = 24.0   # Warning below 24 hours

def get_disk_usage(path="/"):
    """Get disk usage statistics for the given path"""
    try:
        total, used, free = shutil.disk_usage(path)
        return {
            'total_gb': total / (1024**3),
            'used_gb': used / (1024**3),
            'free_gb': free / (1024**3),
            'used_percent': (used / total) * 100
        }
    except Exception as e:
        print(f"Error getting disk usage: {e}")
        return None

def calculate_recording_hours(free_space_gb):
    """Calculate how many hours of recording the free space can hold"""
    if free_space_gb <= 0:
        return 0
    return free_space_gb / RECORDING_RATE_GB_PER_HOUR

def send_notification(title, message, urgency="normal", icon="dialog-information"):
    """Send a desktop notification using notify-send"""
    try:
        # Check if notify-send is available
        subprocess.run(["which", "notify-send"], check=True, capture_output=True)
        
        # Send notification
        cmd = [
            "notify-send",
            "-i", icon,
            "-u", urgency,
            "-t", "10000",  # 10 second timeout
            title,
            message
        ]
        
        subprocess.run(cmd, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        # notify-send not available or failed
        return False

def send_startup_notification():
    """Send notification on system startup"""
    disk_info = get_disk_usage()
    if not disk_info:
        return False
    
    free_gb = disk_info['free_gb']
    recording_hours = calculate_recording_hours(free_gb)
    
    title = "🚢 Argo Autonomous Sailboat"
    message = f"System ready! Can record for {recording_hours:.1f} hours ({free_gb:.1f} GB free)"
    
    return send_notification(title, message, urgency="low", icon="dialog-information")

def send_critical_notification():
    """Send critical notification when storage is low"""
    disk_info = get_disk_usage()
    if not disk_info:
        return False
    
    free_gb = disk_info['free_gb']
    recording_hours = calculate_recording_hours(free_gb)
    
    title = "🔴 ARGO STORAGE CRITICAL"
    message = f"Only {recording_hours:.1f} hours of recording space left! ({free_gb:.1f} GB free)\nConsider stopping recording soon."
    
    return send_notification(title, message, urgency="critical", icon="dialog-warning")

def send_warning_notification():
    """Send warning notification when storage is getting low"""
    disk_info = get_disk_usage()
    if not disk_info:
        return False
    
    free_gb = disk_info['free_gb']
    recording_hours = calculate_recording_hours(free_gb)
    
    title = "🟡 Argo Storage Warning"
    message = f"Storage getting low: {recording_hours:.1f} hours remaining ({free_gb:.1f} GB free)"
    
    return send_notification(title, message, urgency="normal", icon="dialog-warning")

def check_and_notify():
    """Check storage and send appropriate notification"""
    disk_info = get_disk_usage()
    if not disk_info:
        return False
    
    free_gb = disk_info['free_gb']
    recording_hours = calculate_recording_hours(free_gb)
    
    if recording_hours < CRITICAL_HOURS:
        return send_critical_notification()
    elif recording_hours < WARNING_HOURS:
        return send_warning_notification()
    
    return True

def main():
    """Main function for command line usage"""
    if len(sys.argv) < 2:
        print("Argo Storage Notification System")
        print("Usage:")
        print("  python3 storage_notifications.py startup    - Send startup notification")
        print("  python3 storage_notifications.py check      - Check and send warning if needed")
        print("  python3 storage_notifications.py critical   - Send critical notification")
        print("  python3 storage_notifications.py warning    - Send warning notification")
        return
    
    command = sys.argv[1].lower()
    
    if command == "startup":
        success = send_startup_notification()
        if not success:
            print("Failed to send startup notification (notify-send not available)")
    elif command == "check":
        success = check_and_notify()
        if not success:
            print("Failed to check storage or send notification")
    elif command == "critical":
        success = send_critical_notification()
        if not success:
            print("Failed to send critical notification")
    elif command == "warning":
        success = send_warning_notification()
        if not success:
            print("Failed to send warning notification")
    else:
        print(f"Unknown command: {command}")

if __name__ == "__main__":
    main()
