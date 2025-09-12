#!/usr/bin/env python3
"""
Storage Monitor for Argo ROS2 Recording
Computes remaining recording hours and displays storage warnings
"""

import shutil
import os
import sys
from pathlib import Path

# Recording parameters (from our analysis)
RECORDING_RATE_MB_PER_HOUR = 13.5  # MB per hour
RECORDING_RATE_GB_PER_HOUR = RECORDING_RATE_MB_PER_HOUR / 1024.0

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

def format_size_gb(size_gb):
    """Format size in GB with appropriate units"""
    if size_gb >= 1:
        return f"{size_gb:.1f} GB"
    else:
        return f"{size_gb * 1024:.0f} MB"

def calculate_recording_hours(free_space_gb):
    """Calculate how many hours of recording the free space can hold"""
    if free_space_gb <= 0:
        return 0
    return free_space_gb / RECORDING_RATE_GB_PER_HOUR

def get_storage_warning():
    """Get storage warning message"""
    disk_info = get_disk_usage()
    if not disk_info:
        return "⚠️  Unable to check storage space"
    
    free_gb = disk_info['free_gb']
    used_percent = disk_info['used_percent']
    recording_hours = calculate_recording_hours(free_gb)
    
    # Warning thresholds
    critical_hours = 2.0  # Less than 2 hours
    warning_hours = 8.0   # Less than 8 hours
    
    if recording_hours < critical_hours:
        status = "🔴 CRITICAL"
        message = f"SD card will be FULL in {recording_hours:.1f} hours!"
    elif recording_hours < warning_hours:
        status = "🟡 WARNING"
        message = f"SD card will be full in {recording_hours:.1f} hours"
    else:
        status = "🟢 OK"
        message = f"Can record for {recording_hours:.1f} hours"
    
    return f"""
{status} Storage Status:
  Free space: {format_size_gb(free_gb)} ({100-used_percent:.1f}% free)
  Recording rate: {RECORDING_RATE_MB_PER_HOUR:.1f} MB/hour
  {message}
  Used: {format_size_gb(disk_info['used_gb'])} ({used_percent:.1f}%)
  Total: {format_size_gb(disk_info['total_gb'])}
"""

def print_storage_warning():
    """Print storage warning to terminal"""
    warning = get_storage_warning()
    print(warning)

def main():
    """Main function for command line usage"""
    if len(sys.argv) > 1 and sys.argv[1] == "--check":
        print_storage_warning()
    else:
        print("Argo Storage Monitor")
        print("Usage: python3 storage_monitor.py --check")
        print_storage_warning()

if __name__ == "__main__":
    main()
