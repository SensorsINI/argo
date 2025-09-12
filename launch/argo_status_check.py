#!/usr/bin/env python3
"""
Argo Status Check - Python implementation of the bash argo_status_check function

This script provides comprehensive status monitoring for the Argo autonomous sailboat system,
including systemd services, ROS2 nodes, system resources, and storage information.

Usage:
    python3 argo_status_check.py [--manual] [--hourly] [--help]

Options:
    --manual    Force full detailed output (equivalent to argo_status_check true)
    --hourly    Run hourly timer check (equivalent to argo_hourly_timer)
    --help      Show this help message
"""

import os
import sys
import time
import subprocess
import argparse
from datetime import datetime
from pathlib import Path

# ANSI color codes
class Colors:
    RESET = '\033[0m'
    BOLD = '\033[1m'
    DIM = '\033[2m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    YELLOW = '\033[33m'
    BLUE = '\033[34m'
    MAGENTA = '\033[35m'
    CYAN = '\033[36m'
    WHITE = '\033[37m'

class ArgoStatusChecker:
    def __init__(self):
        self.home_dir = os.path.expanduser('~')
        self.argo_scripts_dir = os.path.join(self.home_dir, 'argo', 'scripts')
        self.ros_nodes = ["anem.py", "pwm.py", "gps.py", "imu.py", "control.py", "battery_water.py", "foxglove_bridge"]
        
    def run_command(self, cmd, timeout=5):
        """Run a command and return stdout, stderr, returncode"""
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, shell=True)
            return result.stdout.strip(), result.stderr.strip(), result.returncode
        except subprocess.TimeoutExpired:
            return "", "Command timed out", 1
        except Exception as e:
            return "", str(e), 1
    
    def get_service_status(self, service_name):
        """Get systemd service status"""
        stdout, stderr, returncode = self.run_command(f"systemctl is-active {service_name}")
        return stdout if returncode == 0 else "inactive"
    
    def service_exists(self, service_name):
        """Check if systemd service exists"""
        stdout, stderr, returncode = self.run_command(f"systemctl list-unit-files | grep -q '{service_name}'")
        return returncode == 0
    
    def get_service_pid(self, service_name):
        """Get main PID of a systemd service"""
        stdout, stderr, returncode = self.run_command(f"systemctl show {service_name} --property=MainPID --value")
        if returncode == 0 and stdout and stdout != "0":
            return stdout
        return None
    
    def get_process_stats(self, pid):
        """Get CPU and memory stats for a process"""
        stdout, stderr, returncode = self.run_command(f"ps -p {pid} -o pid,pcpu,pmem,cmd --no-headers")
        if returncode == 0 and stdout:
            parts = stdout.split()
            if len(parts) >= 3:
                return {
                    'pid': parts[0],
                    'cpu': float(parts[1].replace('%', '')),
                    'mem': float(parts[2].replace('%', '')),
                    'cmd': ' '.join(parts[3:])
                }
        return None
    
    def find_ros_node_pids(self, node_name):
        """Find PIDs for ROS nodes from argo scripts directory"""
        stdout, stderr, returncode = self.run_command(f"pgrep -f '/{node_name}'")
        if returncode != 0:
            return []
        
        pids = stdout.split()
        valid_pids = []
        
        for pid in pids:
            # Check if the process is from argo scripts directory
            stdout, stderr, returncode = self.run_command(f"ps -p {pid} -o cmd --no-headers")
            if returncode == 0 and self.argo_scripts_dir in stdout:
                valid_pids.append(pid)
        
        return valid_pids
    
    def get_color_for_usage(self, value, thresholds):
        """Get color code based on usage thresholds"""
        if value > thresholds['high']:
            return Colors.RED
        elif value > thresholds['medium']:
            return Colors.YELLOW
        else:
            return Colors.GREEN
    
    def get_system_info(self):
        """Get system load, memory, and storage information"""
        # Load average
        stdout, stderr, returncode = self.run_command("uptime")
        load_avg = "unknown"
        if returncode == 0:
            parts = stdout.split('load average:')
            if len(parts) > 1:
                load_avg = parts[1].split(',')[0].strip()
        
        # Memory usage
        stdout, stderr, returncode = self.run_command("free | grep Mem")
        mem_usage = "unknown"
        if returncode == 0:
            parts = stdout.split()
            if len(parts) >= 3:
                used = int(parts[2])
                total = int(parts[1])
                mem_usage = f"{used/total*100:.1f}"
        
        # Storage usage
        stdout, stderr, returncode = self.run_command("df / | awk 'NR==2 {printf \"%.1f\", $4/1024/1024}'")
        free_gb = "unknown"
        if returncode == 0:
            free_gb = stdout
        
        stdout, stderr, returncode = self.run_command("df / | awk 'NR==2 {printf \"%.1f\", $3/($3+$4)*100}'")
        used_percent = "unknown"
        if returncode == 0:
            used_percent = stdout
        
        return {
            'load_avg': load_avg,
            'mem_usage': mem_usage,
            'free_gb': free_gb,
            'used_percent': used_percent
        }
    
    def check_ros_nodes(self):
        """Check ROS nodes and collect statistics"""
        running_nodes = 0
        total_cpu = 0.0
        total_mem = 0.0
        missing_nodes = []
        node_details = {}
        
        for node in self.ros_nodes:
            pids = self.find_ros_node_pids(node)
            
            if pids:
                running_nodes += 1
                node_cpu = 0.0
                node_mem = 0.0
                
                for pid in pids:
                    stats = self.get_process_stats(pid)
                    if stats:
                        node_cpu += stats['cpu']
                        node_mem += stats['mem']
                
                total_cpu += node_cpu
                total_mem += node_mem
                
                node_details[node] = {
                    'pids': pids,
                    'cpu': node_cpu,
                    'mem': node_mem,
                    'running': True
                }
            else:
                missing_nodes.append(node)
                node_details[node] = {
                    'pids': [],
                    'cpu': 0.0,
                    'mem': 0.0,
                    'running': False
                }
        
        return {
            'running_nodes': running_nodes,
            'total_cpu': total_cpu,
            'total_mem': total_mem,
            'missing_nodes': missing_nodes,
            'node_details': node_details
        }
    
    def print_detailed_status(self, manual_call=False):
        """Print detailed status output"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"{Colors.BOLD}{Colors.YELLOW}🚢 ARGO STATUS CHECK - {timestamp}{Colors.RESET}")
        
        # Get service statuses
        launch_status = self.get_service_status("argo-launch.service")
        record_status = self.get_service_status("argo-record.service")
        launch_exists = self.service_exists("argo-launch.service")
        
        # Check ROS nodes
        ros_info = self.check_ros_nodes()
        
        # System information
        sys_info = self.get_system_info()
        
        # Print systemd services
        print(f"{Colors.BOLD}{Colors.GREEN}📋 SYSTEMD SERVICES:{Colors.RESET}")
        
        # argo-launch service
        if launch_exists:
            if launch_status == "active":
                launch_pid = self.get_service_pid("argo-launch.service")
                if launch_pid:
                    stats = self.get_process_stats(launch_pid)
                    if stats:
                        print(f"  {Colors.GREEN}argo-launch:{Colors.RESET} {stats['pid']} {stats['cpu']:.1f}% {stats['mem']:.1f}% {stats['cmd']}")
                    else:
                        print(f"  {Colors.GREEN}argo-launch:{Colors.RESET} {Colors.YELLOW}ACTIVE{Colors.RESET} (no stats available)")
                else:
                    print(f"  {Colors.GREEN}argo-launch:{Colors.RESET} {Colors.YELLOW}ACTIVE{Colors.RESET} (no main PID)")
            else:
                print(f"  {Colors.RED}argo-launch:{Colors.RESET} {Colors.RED}{launch_status}{Colors.RESET}")
        else:
            print(f"  {Colors.RED}argo-launch:{Colors.RESET} {Colors.RED}SERVICE NOT FOUND{Colors.RESET}")
        
        # argo-record service
        if record_status == "active":
            record_pid = self.get_service_pid("argo-record.service")
            if record_pid:
                stats = self.get_process_stats(record_pid)
                if stats:
                    print(f"  {Colors.GREEN}argo-record:{Colors.RESET} {stats['pid']} {stats['cpu']:.1f}% {stats['mem']:.1f}% {stats['cmd']}")
                else:
                    print(f"  {Colors.GREEN}argo-record:{Colors.RESET} {Colors.YELLOW}ACTIVE{Colors.RESET} (no stats available)")
            else:
                print(f"  {Colors.GREEN}argo-record:{Colors.RESET} {Colors.YELLOW}ACTIVE{Colors.RESET} (no main PID)")
        else:
            print(f"  {Colors.RED}argo-record:{Colors.RESET} {Colors.RED}{record_status}{Colors.RESET}")
        
        # Print ROS nodes
        print(f"{Colors.BOLD}{Colors.GREEN}🤖 ROS NODES:{Colors.RESET}")
        
        for node in self.ros_nodes:
            node_info = ros_info['node_details'][node]
            
            if node_info['running']:
                for pid in node_info['pids']:
                    stats = self.get_process_stats(pid)
                    if stats:
                        cpu_color = self.get_color_for_usage(stats['cpu'], {'high': 50, 'medium': 20})
                        mem_color = self.get_color_for_usage(stats['mem'], {'high': 10, 'medium': 5})
                        print(f"  {Colors.GREEN}{node}:{Colors.RESET} PID:{pid} CPU:{cpu_color}{stats['cpu']:.1f}%{Colors.RESET} MEM:{mem_color}{stats['mem']:.1f}%{Colors.RESET}")
            else:
                print(f"  {Colors.RED}{node}:{Colors.RESET} {Colors.RED}NOT RUNNING{Colors.RESET}")
        
        # Print summary
        print(f"{Colors.BOLD}{Colors.CYAN}📊 SUMMARY:{Colors.RESET}")
        print(f"  Running nodes: {Colors.GREEN}{ros_info['running_nodes']}{Colors.RESET}/{Colors.YELLOW}{len(self.ros_nodes)}{Colors.RESET}")
        print(f"  Total CPU usage: {Colors.GREEN}{ros_info['total_cpu']:.1f}%{Colors.RESET}")
        print(f"  Total memory usage: {Colors.GREEN}{ros_info['total_mem']:.1f}%{Colors.RESET}")
        print(f"  System load: {Colors.GREEN}{sys_info['load_avg']}{Colors.RESET}")
        print(f"  System memory: {Colors.GREEN}{sys_info['mem_usage']}%{Colors.RESET} used")
        print(f"  Storage: {Colors.GREEN}{sys_info['free_gb']}GB{Colors.RESET} free ({Colors.GREEN}{sys_info['used_percent']}%{Colors.RESET} used)")
        
        print("=" * 60)
        
        if launch_status == "active" or record_status == "active":
            print(f"  {Colors.RED}Stop: {Colors.RESET}aq (or: sudo systemctl stop argo-launch.service argo-record.service)")
    
    def print_condensed_status(self):
        """Print condensed single-line status"""
        ros_info = self.check_ros_nodes()
        sys_info = self.get_system_info()
        
        print(f"{Colors.BOLD}{Colors.YELLOW}🚢 ARGO:{Colors.RESET} {Colors.GREEN}{ros_info['running_nodes']}{Colors.RESET}/{Colors.YELLOW}{len(self.ros_nodes)}{Colors.RESET} nodes | "
              f"CPU:{Colors.GREEN}{ros_info['total_cpu']:.1f}%{Colors.RESET} MEM:{Colors.GREEN}{ros_info['total_mem']:.1f}%{Colors.RESET} | "
              f"Load:{Colors.GREEN}{sys_info['load_avg']}{Colors.RESET} SysMem:{Colors.GREEN}{sys_info['mem_usage']}%{Colors.RESET} | "
              f"Storage:{Colors.GREEN}{sys_info['free_gb']}GB{Colors.RESET} free")
    
    def check_hourly_timer(self):
        """Check if hourly timer should run"""
        last_check_file = os.path.join(self.home_dir, ".argo_last_check")
        current_time = int(time.time())
        last_check_time = 0
        
        if os.path.exists(last_check_file):
            try:
                with open(last_check_file, 'r') as f:
                    last_check_time = int(f.read().strip())
            except (ValueError, IOError):
                last_check_time = 0
        
        # Run check if it's been more than 1 hour (3600 seconds)
        time_diff = current_time - last_check_time
        if time_diff >= 3600:
            self.print_condensed_status()
            with open(last_check_file, 'w') as f:
                f.write(str(current_time))
            return True
        return False

def main():
    parser = argparse.ArgumentParser(description='Argo Status Check - Monitor Argo sailboat system status')
    parser.add_argument('--manual', action='store_true', help='Force full detailed output')
    parser.add_argument('--hourly', action='store_true', help='Run hourly timer check')
    
    args = parser.parse_args()
    
    checker = ArgoStatusChecker()
    
    if args.hourly:
        # Run hourly timer check
        checker.check_hourly_timer()
    elif args.manual:
        # Force detailed output
        checker.print_detailed_status(manual_call=True)
    else:
        # Default: show detailed output (same as manual for now)
        checker.print_detailed_status(manual_call=True)

if __name__ == "__main__":
    main()
