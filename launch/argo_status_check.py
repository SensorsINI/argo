#!/usr/bin/env python3
"""
Optimized Argo Status Check - Fast version with performance improvements

Key optimizations:
1. Replace slow systemctl list-unit-files with direct service status check
2. Batch process information gathering
3. Reduce subprocess calls by combining commands
4. Cache service existence checks
"""

import os
import sys
import time
import subprocess
import argparse
import signal
from datetime import datetime
from pathlib import Path

# Handle KeyboardInterrupt gracefully
def signal_handler(signum, frame):
    print("\n🛑 Interrupted by user")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# Import centralized node utilities
from argo_node_utils import ArgoNodeManager

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

class OptimizedArgoStatusChecker:
    def __init__(self):
        self.home_dir = os.path.expanduser('~')
        self.argo_root = os.path.join(self.home_dir, 'argo')
        self.node_manager = ArgoNodeManager(self.argo_root)
        self._service_cache = {}
        
    def run_command(self, cmd, timeout=3):
        """Run a command and return stdout, stderr, returncode"""
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, shell=True)
            return result.stdout.strip(), result.stderr.strip(), result.returncode
        except subprocess.TimeoutExpired:
            return "", "Command timed out", 1
        except Exception as e:
            return "", str(e), 1
    
    def get_service_status_fast(self, service_name):
        """Get systemd service status - optimized version"""
        # Check cache first
        if service_name in self._service_cache:
            return self._service_cache[service_name]
        
        # Use systemctl is-active which is much faster than list-unit-files
        stdout, stderr, returncode = self.run_command(f"systemctl is-active {service_name}")
        
        if returncode == 0:
            status = stdout if stdout else "inactive"
        else:
            # If service doesn't exist, is-active returns non-zero
            # Check if it's because service doesn't exist or is just inactive
            stdout2, stderr2, returncode2 = self.run_command(f"systemctl show {service_name} --property=LoadState --value")
            if returncode2 == 0 and stdout2 == "not-found":
                status = "not-found"
            else:
                status = "inactive"
        
        # Cache the result
        self._service_cache[service_name] = status
        return status
    
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
    
    def get_all_ros_processes(self):
        """Get all ROS processes using centralized node manager"""
        return self.node_manager.get_all_ros_processes()
    
    def get_color_for_usage(self, value, thresholds):
        """Get color code based on usage thresholds"""
        if value > thresholds['high']:
            return Colors.RED
        elif value > thresholds['medium']:
            return Colors.YELLOW
        else:
            return Colors.GREEN
    
    def get_system_info_fast(self):
        """Get system load, memory, and storage information - optimized"""
        # Get all system info in one command to reduce subprocess overhead
        cmd = """
        uptime | sed 's/.*load average: *//' | cut -d',' -f1 | tr -d ' ';
        free | grep Mem | awk '{printf "%.1f", $3/$2*100}';
        df / | awk 'NR==2 {printf "%.1f", $4/1024/1024}';
        df / | awk 'NR==2 {printf "%.1f", $3/($3+$4)*100}'
        """
        
        stdout, stderr, returncode = self.run_command(cmd)
        if returncode == 0:
            lines = stdout.strip().split('\n')
            if len(lines) >= 4:
                return {
                    'load_avg': lines[0].strip(),
                    'mem_usage': lines[1].strip(),
                    'free_gb': lines[2].strip(),
                    'used_percent': lines[3].strip()
                }
        
        # Fallback to individual commands if batch fails
        return self.get_system_info_fallback()
    
    def get_system_info_fallback(self):
        """Fallback system info gathering"""
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
    
    def check_ros_nodes_fast(self):
        """Check ROS nodes and collect statistics using centralized node manager"""
        return self.node_manager.get_node_summary()
    
    def print_detailed_status(self, manual_call=False):
        """Print detailed status output"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f"{Colors.BOLD}{Colors.YELLOW}🚢 ARGO STATUS CHECK - {timestamp}{Colors.RESET}")
        
        # Get service statuses (optimized)
        launch_status = self.get_service_status_fast("argo-launch.service")
        # Recording is now handled via ROS2 service, not systemd
        record_status = "ros2-service"
        
        # Check ROS nodes (optimized)
        ros_info = self.check_ros_nodes_fast()
        
        # System information (optimized)
        sys_info = self.get_system_info_fast()
        
        # Print systemd services
        print(f"{Colors.BOLD}{Colors.GREEN}📋 SYSTEMD SERVICES:{Colors.RESET}")
        
        # argo-launch service
        if launch_status == "not-found":
            print(f"  {Colors.RED}argo-launch:{Colors.RESET} {Colors.RED}SERVICE NOT FOUND{Colors.RESET}")
        elif launch_status == "active":
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
        
        # Recording service (now ROS2-based)
        print(f"  {Colors.CYAN}argo-record:{Colors.RESET} {Colors.CYAN}ros2-service{Colors.RESET}")
        
        # Print ROS nodes
        print(f"{Colors.BOLD}{Colors.GREEN}🤖 ROS NODES:{Colors.RESET}")
        
        for node in self.node_manager.discover_nodes():
            node_info = ros_info['node_status'][node]
            
            if node_info['running']:
                for proc in node_info['processes']:
                    cpu_color = self.get_color_for_usage(proc['cpu'], {'high': 50, 'medium': 20})
                    mem_color = self.get_color_for_usage(proc['mem'], {'high': 10, 'medium': 5})
                    print(f"  {Colors.GREEN}{node}:{Colors.RESET} PID:{proc['pid']} CPU:{cpu_color}{proc['cpu']:.1f}%{Colors.RESET} MEM:{mem_color}{proc['mem']:.1f}%{Colors.RESET}")
            else:
                print(f"  {Colors.RED}{node}:{Colors.RESET} {Colors.RED}NOT RUNNING{Colors.RESET}")
        
        # Print summary
        print(f"{Colors.BOLD}{Colors.CYAN}📊 SUMMARY:{Colors.RESET}")
        print(f"  Running nodes: {Colors.GREEN}{ros_info['running_nodes']}{Colors.RESET}/{Colors.YELLOW}{ros_info['total_nodes']}{Colors.RESET}")
        print(f"  Total CPU usage: {Colors.GREEN}{ros_info['total_cpu']:.1f}%{Colors.RESET}")
        print(f"  Total memory usage: {Colors.GREEN}{ros_info['total_mem']:.1f}%{Colors.RESET}")
        print(f"  System load: {Colors.GREEN}{sys_info['load_avg']}{Colors.RESET}")
        print(f"  System memory: {Colors.GREEN}{sys_info['mem_usage']}%{Colors.RESET} used")
        print(f"  Storage: {Colors.GREEN}{sys_info['free_gb']}GB{Colors.RESET} free ({Colors.GREEN}{sys_info['used_percent']}%{Colors.RESET} used)")
        
        print("=" * 60)
        
        if launch_status == "active":
            print(f"  {Colors.RED}Stop: {Colors.RESET}aq (or: sudo systemctl stop argo-launch.service)")
    
    def print_condensed_status(self):
        """Print condensed single-line status"""
        ros_info = self.check_ros_nodes_fast()
        sys_info = self.get_system_info_fast()
        
        print(f"{Colors.BOLD}{Colors.YELLOW}🚢 ARGO:{Colors.RESET} {Colors.GREEN}{ros_info['running_nodes']}{Colors.RESET}/{Colors.YELLOW}{ros_info['total_nodes']}{Colors.RESET} nodes | "
              f"CPU:{Colors.GREEN}{ros_info['total_cpu']:.1f}%{Colors.RESET} MEM:{Colors.GREEN}{ros_info['total_mem']:.1f}%{Colors.RESET} | "
              f"Load:{Colors.GREEN}{sys_info['load_avg']}{Colors.RESET} SysMem:{Colors.GREEN}{sys_info['mem_usage']}%{Colors.RESET} | "
              f"Storage:{Colors.GREEN}{sys_info['free_gb']}GB{Colors.RESET} free")
    
    def check_quick_timer(self):
        """Check if quick timer should run"""
        last_check_file = os.path.join(self.home_dir, ".argo_last_check")
        current_time = int(time.time())
        last_check_time = 0
        
        if os.path.exists(last_check_file):
            try:
                with open(last_check_file, 'r') as f:
                    last_check_time = int(f.read().strip())
            except (ValueError, IOError):
                last_check_time = 0
        
        # Run check if it's been more than 5 minutes (300 seconds)
        time_diff = current_time - last_check_time
        if time_diff >= 300:
            self.print_condensed_status()
            with open(last_check_file, 'w') as f:
                f.write(str(current_time))
            return True
        else:
            # For automation: exit with code 0 but no output when timer condition not met
            # This allows cron/systemd to know the script ran successfully
            return True

def main():
    parser = argparse.ArgumentParser(
        description='Optimized Argo Status Check - Fast monitoring for Argo sailboat system',
        epilog='''
EXAMPLES:
  %(prog)s                    # Show detailed status output (default)
  %(prog)s --manual          # Force full detailed output (same as default)
  %(prog)s --quick           # Run quick timer check (condensed output if 5+ min since last check)

BEHAVIOR:
  Default mode shows comprehensive status including systemd services, ROS nodes, 
  and system metrics with colored output for easy monitoring.
  
  Quick mode is designed for automated/cron usage - it only displays condensed 
  single-line status if more than 5 minutes have passed since the last check.
  This prevents spam in logs while still providing periodic status updates.
  When timer condition is not met, script exits silently with code 0.
        ''',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('--manual', action='store_true', 
                       help='Force full detailed status output (same as default behavior)')
    parser.add_argument('--quick', action='store_true', 
                       help='Run quick timer check - shows condensed status only if 5+ minutes since last check')
    
    args = parser.parse_args()
    
    checker = OptimizedArgoStatusChecker()
    
    if args.quick:
        # Run quick timer check
        checker.check_quick_timer()
    elif args.manual:
        # Force detailed output
        checker.print_detailed_status(manual_call=True)
    else:
        # Default: show detailed output (same as manual for now)
        checker.print_detailed_status(manual_call=True)

if __name__ == "__main__":
    main()
