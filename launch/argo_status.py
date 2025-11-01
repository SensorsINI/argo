#!/usr/bin/env python3
"""
Argo Status Reporter
====================

Command-line tool for querying Argo system status using standard ROS2 commands.
Replaces the custom status reporting with standard ROS2 interfaces.

Usage:
    python3 launch/argo_status.py           # Full status
    python3 launch/argo_status.py --quick   # Quick one-line status
"""

import argparse
import subprocess
import json
import sys
import time
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


def call_ros2_service(service_name: str, service_type: str = 'std_srvs/srv/Trigger', timeout: float = 10.0) -> tuple[bool, str]:
    """Call a ROS2 service and return (success, message)"""
    try:
        result = subprocess.run(
            ['ros2', 'service', 'call', service_name, service_type],
            capture_output=True,
            text=True,
            timeout=timeout
        )
        
        if result.returncode == 0:
            # Parse response
            output = result.stdout.strip()
            if 'success: True' in output or 'success: 1' in output:
                # Extract message if available
                if 'message:' in output:
                    message = output.split('message:')[1].strip()
                    return True, message
                return True, output
            return False, output
        else:
            return False, result.stderr or result.stdout
        
    except subprocess.TimeoutExpired:
        return False, "Service call timed out"
    except Exception as e:
        return False, str(e)


def get_node_list() -> list[str]:
    """Get list of running ROS2 nodes"""
    try:
        result = subprocess.run(
            ['ros2', 'node', 'list'],
            capture_output=True,
            text=True,
            timeout=5.0
        )
        
        if result.returncode == 0:
            return [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        return []
    except Exception:
        return []


def get_topic_list() -> list[str]:
    """Get list of available ROS2 topics"""
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5.0
        )
        
        if result.returncode == 0:
            return [line.strip() for line in result.stdout.strip().split('\n') if line.strip()]
        return []
    except Exception:
        return []


def get_health_status() -> dict:
    """Get health status from health monitor service"""
    success, message = call_ros2_service('/argo/health/status')
    
    if success:
        try:
            return json.loads(message)
        except json.JSONDecodeError:
            return {'raw': message}
    else:
        return {'error': message}


def print_full_status():
    """Print comprehensive system status"""
    print(f"🚢 ARGO STATUS - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)
    
    # Check systemd services
    print("\n📋 SYSTEMD SERVICES:")
    services = [
        'argo_launch_standard.service',
        'argo_power_control.service',
        'argo_battery_water.service',
        'argo_bno085.service'
    ]
    
    for service in services:
        try:
            result = subprocess.run(
                ['systemctl', 'is-active', service],
                capture_output=True,
                text=True,
                timeout=1.0
            )
            status = "🟢 RUNNING" if result.returncode == 0 else "🔴 STOPPED"
            print(f"  {service}: {status}")
        except Exception:
            print(f"  {service}: ❓ UNKNOWN")
    
    # Get ROS2 node status
    print("\n🤖 ROS2 NODES:")
    nodes = get_node_list()
    
    if nodes:
        print(f"  Running: {len(nodes)} nodes")
        for node in sorted(nodes):
            print(f"    - {node}")
    else:
        print("  No ROS2 nodes detected")
    
    # Get health status if available
    print("\n🏥 HEALTH STATUS:")
    health = get_health_status()
    
    if 'error' in health:
        print(f"  ⚠️  Health monitor unavailable: {health['error']}")
    elif 'nodes' in health:
        healthy_count = sum(1 for n in health['nodes'].values() if n.get('healthy') is True)
        unhealthy_count = sum(1 for n in health['nodes'].values() if n.get('healthy') is False)
        total_count = len(health['nodes'])
        
        print(f"  Nodes: {healthy_count}/{total_count} healthy, {unhealthy_count} unhealthy")
        
        # Show unhealthy nodes
        unhealthy_nodes = [name for name, info in health['nodes'].items() if info.get('healthy') is False]
        if unhealthy_nodes:
            print(f"  ⚠️  Unhealthy nodes: {', '.join(unhealthy_nodes)}")
    else:
        print("  ⚠️  Health status unavailable")
    
    # Topic count
    print("\n📡 ROS2 TOPICS:")
    topics = get_topic_list()
    print(f"  Available: {len(topics)} topics")
    
    print("=" * 60)


def print_quick_status():
    """Print condensed one-line status"""
    nodes = get_node_list()
    node_count = len(nodes)
    
    # Try to get health status
    health = get_health_status()
    if 'nodes' in health:
        healthy_count = sum(1 for n in health['nodes'].values() if n.get('healthy') is True)
        health_str = f" | 🏥 {healthy_count}H"
    else:
        health_str = ""
    
    # Check if launch service is running
    try:
        result = subprocess.run(
            ['systemctl', 'is-active', 'argo_launch_standard.service'],
            capture_output=True,
            timeout=1.0
        )
        service_status = "🟢" if result.returncode == 0 else "🔴"
    except Exception:
        service_status = "❓"
    
    print(f"🚢 ARGO: [{node_count} nodes] | Service: {service_status}{health_str}", flush=True)


def main():
    parser = argparse.ArgumentParser(description='Argo Status Reporter')
    parser.add_argument('--quick', action='store_true',
                       help='Show quick one-line status')
    
    args = parser.parse_args()
    
    if args.quick:
        print_quick_status()
    else:
        print_full_status()


if __name__ == '__main__':
    main()

