#!/usr/bin/env python3
"""
Argo Sailboat CLI GUI
=====================

Interactive command-line interface for monitoring and controlling the Argo autonomous sailboat.
Provides real-time status updates, node management, and recording control with color output.

Features:
- Real-time ROS2 node status monitoring
- Service control (start/stop/restart)
- Recording control (start/stop)
- Color-coded status indicators
- Watch-like screen updates
- Keystroke commands for quick control

Usage:
    sudo python3 launch/argo_gui.py
    sudo ./launch/argo_gui.sh
    python3 launch/argo_gui.py --help

Key Commands:
    s - Start Argo service
    q - Quit Argo service  
    r - Start recording
    c - Stop recording
    R - Restart service
    h - Show help
    Ctrl+C - Exit GUI

Note: Requires sudo privileges to control systemd services.
"""

import os
import sys
import time
import subprocess
import threading
import signal
import argparse
from datetime import datetime
from typing import Dict, List, Optional, Tuple
import json

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

def check_sudo_privileges():
    """Check if running with sudo privileges"""
    return os.geteuid() == 0

def require_sudo():
    """Exit if not running with sudo privileges"""
    if not check_sudo_privileges():
        print(f"{Colors.RED}Error: This GUI requires sudo privileges to control systemd services{Colors.RESET}")
        print(f"{Colors.YELLOW}Please run with: sudo python3 launch/argo_gui.py{Colors.RESET}")
        print(f"{Colors.DIM}Or use: sudo ./launch/argo_gui.sh{Colors.RESET}")
        sys.exit(1)

# Color codes for terminal output
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
    BG_RED = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_YELLOW = '\033[43m'
    BG_BLUE = '\033[44m'

class ArgoGUI:
    def __init__(self):
        self.running = True
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.launch_service = "argo-launch.service"
        self.record_service = "argo-record.service"
        self.refresh_interval = 10.0  # seconds
        self.last_update = 0
        
        # Set up signal handler for immediate Ctrl+C response
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Status tracking
        self.service_status = {}
        self.recording_status = {}
        self.ros_nodes = {}
        self.last_errors = []
        
        # Cache for status output to avoid expensive calls
        self.cached_status_output = ""
        self.status_cache_time = 0
        self.status_cache_interval = 5  # Cache for 5 seconds
        
        # Threading for non-blocking status updates
        self.status_thread = None
        self.status_lock = threading.Lock()
        self.status_updating = False
        
        # Screen dimensions
        self.terminal_width = 80
        self.terminal_height = 24
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Initialize screen
        self.init_screen()
        
    def signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        self.running = False
        self.cleanup_screen()
        print(f"\n{Colors.YELLOW}Argo GUI shutting down...{Colors.RESET}")
        sys.exit(0)
        
    def init_screen(self):
        """Initialize the terminal screen"""
        try:
            # Get terminal size
            import shutil
            size = shutil.get_terminal_size()
            self.terminal_width = size.columns
            self.terminal_height = size.lines
        except:
            pass
            
        # Clear screen and hide cursor
        print('\033[2J\033[H\033[?25l', end='')
        sys.stdout.flush()
        
    def cleanup_screen(self):
        """Restore terminal settings"""
        print('\033[?25h\033[0m', end='')  # Show cursor and reset colors
        sys.stdout.flush()
        
        # Cleanup ROS2 if it was initialized
        if ROS2_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        
    def clear_screen(self):
        """Clear the screen - optimized for raw mode"""
        # Use a more reliable method for raw mode
        print('\033[2J\033[3J\033[1;1H\033[0m', end='')
        sys.stdout.flush()
        time.sleep(0.05)
        
    def move_cursor(self, row: int, col: int):
        """Move cursor to specific position"""
        print(f'\033[{row};{col}H', end='')
        sys.stdout.flush()
        
    def get_service_status(self) -> Dict[str, str]:
        """Get status of Argo services"""
        status = {}
        try:
            # Check launch service
            result = subprocess.run(['systemctl', 'is-active', self.launch_service], 
                                  capture_output=True, text=True, timeout=5)
            status['launch'] = result.stdout.strip()
            
            # Check record service  
            result = subprocess.run(['systemctl', 'is-active', self.record_service], 
                                  capture_output=True, text=True, timeout=5)
            status['record'] = result.stdout.strip()
            
        except Exception as e:
            status['launch'] = 'unknown'
            status['record'] = 'unknown'
            
        return status
        
    def get_argo_status_output(self) -> str:
        """Get the formatted status output from the Python argo_status_check script"""
        try:
            # Call the Python script directly with timeout for responsiveness
            # Determine the correct path to argo_status_check.py
            status_check_path = f'{self.argo_dir}/launch/argo_status_check.py'
            if not os.path.exists(status_check_path):
                status_check_path = f'{self.argo_dir}/argo_status_check.py'
            result = subprocess.run(['python3', status_check_path, '--manual'], 
                                  capture_output=True, text=True, timeout=2)
            if result.returncode == 0:
                # Filter out screen clearing sequences that conflict with GUI
                output = result.stdout
                # Remove common screen clearing sequences
                output = output.replace('\033[2J', '')  # Clear entire screen
                output = output.replace('\033[3J', '')  # Clear scrollback buffer
                output = output.replace('\033[H', '')   # Move cursor to home
                output = output.replace('\033[1;1H', '') # Move cursor to 1,1
                output = output.replace('\033[?25l', '') # Hide cursor
                return output
            else:
                return f"Error getting status: {result.stderr.strip()}"
        except subprocess.TimeoutExpired:
            # Fallback to simple status check
            return self._get_simple_status()
        except Exception as e:
            return f"Error running status check: {str(e)}"
    
    def _get_simple_status(self) -> str:
        """Fallback simple status check when main script times out"""
        try:
            # Check systemd services
            launch_result = subprocess.run(['systemctl', '--no-pager', 'is-active', 'argo-launch.service'], 
                                         capture_output=True, text=True, timeout=5)
            record_result = subprocess.run(['systemctl', '--no-pager', 'is-active', 'argo-record.service'], 
                                         capture_output=True, text=True, timeout=5)
            
            launch_status = "active" if launch_result.returncode == 0 else "inactive"
            record_status = "active" if record_result.returncode == 0 else "inactive"
            
            # Get ROS nodes
            ros_nodes = self._get_ros_nodes_simple()
            
            # Get system info
            cpu_temp = self._get_cpu_temp()
            memory = self._get_memory_usage()
            
            status = f"""🚢 ARGO STATUS CHECK - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
📋 SYSTEMD SERVICES:
  argo-launch: {'🟢 active' if launch_status == 'active' else '🔴 inactive'}
  argo-record: {'🟢 active' if record_status == 'active' else '🔴 inactive'}
🤖 ROS NODES:
{ros_nodes}
📊 SUMMARY:
  Launch service: {launch_status}
  Record service: {record_status}
  CPU Temp: {cpu_temp}
  Memory: {memory}
============================================================"""
            
            return status
        except Exception as e:
            return f"Simple status check failed: {str(e)}"
    
    def _get_cpu_temp(self) -> str:
        """Get CPU temperature"""
        try:
            result = subprocess.run(['cat', '/sys/class/thermal/thermal_zone0/temp'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                temp = int(result.stdout.strip()) / 1000
                return f"{temp:.1f}°C"
        except:
            pass
        return "Unknown"
    
    def _get_memory_usage(self) -> str:
        """Get memory usage"""
        try:
            result = subprocess.run(['free', '-h'], capture_output=True, text=True, timeout=5)
            if result.returncode == 0:
                lines = result.stdout.strip().split('\n')
                if len(lines) > 1:
                    mem_line = lines[1].split()
                    if len(mem_line) >= 3:
                        used = mem_line[2]
                        total = mem_line[1]
                        return f"{used}/{total}"
        except:
            pass
        return "Unknown"
    
    def _get_ros_nodes_simple(self) -> str:
        """Get ROS node status using simple process checks"""
        try:
            # List of expected ROS nodes
            expected_nodes = ['anem.py', 'pwm.py', 'gps.py', 'imu.py', 'control.py', 'battery_water.py', 'foxglove_bridge']
            node_status = []
            
            for node in expected_nodes:
                # Check if process is running - use same pattern as original find_ros_node_pids
                search_pattern = f"/{node}"  # This matches the original: pgrep -f '/{node_name}'
                result = subprocess.run(['pgrep', '-f', search_pattern], 
                                      capture_output=True, text=True, timeout=2)
                if result.returncode == 0:
                    # Get CPU and memory usage
                    try:
                        pids = result.stdout.strip().split('\n')
                        pid = pids[0] if pids else None  # Take first PID if multiple
                        if not pid:
                            node_status.append(f"  {node}: 🟢 RUNNING")
                            continue
                            
                        ps_result = subprocess.run(['ps', '-o', 'pid,pcpu,pmem,cmd', '-p', pid], 
                                                 capture_output=True, text=True, timeout=2)
                        if ps_result.returncode == 0:
                            lines = ps_result.stdout.strip().split('\n')
                            if len(lines) > 1:
                                # Find the line that matches our PID
                                for line in lines[1:]:  # Skip header
                                    parts = line.split(None, 3)  # Split into max 4 parts
                                    if len(parts) >= 4 and parts[0] == pid:
                                        cpu = parts[1] if parts[1] != '-' else '0.0'
                                        mem = parts[2] if parts[2] != '-' else '0.0'
                                        node_status.append(f"  {node}: 🟢 RUNNING (CPU: {cpu}%, MEM: {mem}%)")
                                        break
                                else:
                                    # PID not found in ps output, just show running
                                    node_status.append(f"  {node}: 🟢 RUNNING")
                            else:
                                node_status.append(f"  {node}: 🟢 RUNNING")
                        else:
                            node_status.append(f"  {node}: 🟢 RUNNING")
                    except Exception as e:
                        # If parsing fails, just show running without stats
                        node_status.append(f"  {node}: 🟢 RUNNING")
                else:
                    node_status.append(f"  {node}: 🔴 NOT RUNNING")
            
            return '\n'.join(node_status) if node_status else "  No ROS nodes found"
        except Exception as e:
            return f"  Error checking ROS nodes: {str(e)}"
    
    def get_cached_status_output(self) -> str:
        """Get cached status output or fetch new one if cache is stale"""
        current_time = time.time()
        if (current_time - self.status_cache_time) > self.status_cache_interval or not self.cached_status_output.strip():
            # Cache is stale or empty, fetch new status synchronously
            self.cached_status_output = self.get_argo_status_output()
            self.status_cache_time = current_time
            
        return self.cached_status_output
    
    def _start_background_status_update(self):
        """Start background status update in a separate thread"""
        with self.status_lock:
            if self.status_updating:
                return  # Already updating
            self.status_updating = True
        
        def update_status():
            try:
                new_status = self.get_argo_status_output()
                with self.status_lock:
                    self.cached_status_output = new_status
                    self.status_cache_time = time.time()
                    self.status_updating = False
            except Exception as e:
                with self.status_lock:
                    self.cached_status_output = f"Error updating status: {str(e)}"
                    self.status_cache_time = time.time()
                    self.status_updating = False
        
        self.status_thread = threading.Thread(target=update_status, daemon=True)
        self.status_thread.start()
    
    def force_status_update(self):
        """Force immediate status update by invalidating cache"""
        with self.status_lock:
            self.status_cache_time = 0
            self.status_updating = False
    
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C signal for immediate exit"""
        self.running = False
        print(f"\n{Colors.YELLOW}Received interrupt signal, shutting down...{Colors.RESET}")
        # Don't call sys.exit() here as it can interfere with cleanup
        # Let the main loop handle the exit
        
    def get_ros_nodes(self) -> Dict[str, Dict]:
        """Get ROS2 node information using Python APIs"""
        nodes = {}
        
        if not ROS2_AVAILABLE:
            return nodes
            
        try:
            # Initialize ROS2 if not already initialized
            if not rclpy.ok():
                rclpy.init()
            
            # Create a temporary node to query the graph
            temp_node = Node('argo_gui_temp')
            
            try:
                # Get list of node names
                node_names = temp_node.get_node_names()
                
                for node_name in node_names:
                    if node_name and not node_name.startswith('argo_gui_temp'):
                        try:
                            # Get node info
                            node_info = temp_node.get_node_names_and_namespaces()
                            
                            # Find this specific node
                            node_found = False
                            for name, namespace in node_info:
                                if name == node_name.split('/')[-1]:  # Remove namespace prefix
                                    nodes[node_name] = {
                                        'status': 'active',
                                        'info': f'Namespace: {namespace}'
                                    }
                                    node_found = True
                                    break
                            
                            if not node_found:
                                nodes[node_name] = {
                                    'status': 'active',
                                    'info': 'Node active'
                                }
                                
                        except Exception as e:
                            nodes[node_name] = {
                                'status': 'error',
                                'info': f'Error: {str(e)}'
                            }
                            
            finally:
                temp_node.destroy_node()
                
        except Exception as e:
            # Fallback to subprocess method if ROS2 APIs fail
            try:
                cmd = 'source /opt/ros/humble/setup.bash && ros2 node list'
                result = subprocess.run(['bash', '-i', '-c', cmd], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    node_names = result.stdout.strip().split('\n')
                    for node_name in node_names:
                        if node_name:
                            nodes[node_name] = {
                                'status': 'active',
                                'info': 'Node active'
                            }
            except:
                pass
            
        return nodes
        
    def get_recording_info(self) -> Dict[str, str]:
        """Get recording status and information"""
        info = {}
        try:
            # Check if recording service is active
            result = subprocess.run(['systemctl', 'is-active', self.record_service], 
                                  capture_output=True, text=True, timeout=5)
            info['status'] = result.stdout.strip()
            
            # Get bag files directory info
            bagfiles_dir = "/home/orangepi/bagfiles"
            if os.path.exists(bagfiles_dir):
                # Get latest bag file
                try:
                    result = subprocess.run(['ls', '-t', bagfiles_dir], 
                                          capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        files = result.stdout.strip().split('\n')
                        if files and files[0]:
                            latest_bag = files[0]
                            info['latest_bag'] = latest_bag
                            
                            # Get size
                            bag_path = os.path.join(bagfiles_dir, latest_bag)
                            if os.path.exists(bag_path):
                                result = subprocess.run(['du', '-sh', bag_path], 
                                                      capture_output=True, text=True, timeout=5)
                                if result.returncode == 0:
                                    info['latest_size'] = result.stdout.split()[0]
                except:
                    pass
        except Exception as e:
            info['status'] = 'unknown'
            
        return info
        
    def get_recent_errors(self) -> List[str]:
        """Get recent error messages from journal"""
        errors = []
        try:
            result = subprocess.run([
                'journalctl', '--since', '5 minutes ago', 
                '-u', self.launch_service, '-u', self.record_service,
                '--priority=err', '--no-pager', '-n', '10'
            ], capture_output=True, text=True, timeout=10)
            
            if result.returncode == 0 and result.stdout.strip():
                error_lines = result.stdout.strip().split('\n')
                for line in error_lines[-5:]:  # Last 5 errors
                    if line.strip():
                        # Extract timestamp and message
                        parts = line.split(']', 1)
                        if len(parts) > 1:
                            timestamp = parts[0] + ']'
                            message = parts[1].strip()
                            errors.append(f"{timestamp} {message}")
        except Exception as e:
            pass
            
        return errors
        
    def get_system_info(self) -> Dict[str, str]:
        """Get system information"""
        info = {}
        try:
            # CPU temperature
            try:
                with open('/sys/class/thermal/thermal_zone2/temp', 'r') as f:
                    temp_millicelsius = int(f.read().strip())
                    info['cpu_temp'] = f"{temp_millicelsius / 1000.0:.1f}°C"
            except:
                info['cpu_temp'] = "N/A"
                
            # Memory usage
            try:
                result = subprocess.run(['free', '-h'], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    lines = result.stdout.strip().split('\n')
                    if len(lines) > 1:
                        mem_line = lines[1].split()
                        if len(mem_line) >= 3:
                            info['memory'] = f"{mem_line[2]}/{mem_line[1]}"
            except:
                info['memory'] = "N/A"
                
            # Disk usage
            try:
                result = subprocess.run(['df', '-h', '/'], capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    lines = result.stdout.strip().split('\n')
                    if len(lines) > 1:
                        disk_line = lines[1].split()
                        if len(disk_line) >= 5:
                            info['disk'] = f"{disk_line[4]} used"
            except:
                info['disk'] = "N/A"
                
        except Exception as e:
            pass
            
        return info
        
    def format_status_indicator(self, status: str) -> str:
        """Format status with appropriate color"""
        if status == 'active' or status == 'running':
            return f"{Colors.GREEN}●{Colors.RESET}"
        elif status == 'inactive' or status == 'stopped':
            return f"{Colors.RED}●{Colors.RESET}"
        elif status == 'failed':
            return f"{Colors.RED}✗{Colors.RESET}"
        else:
            return f"{Colors.YELLOW}?{Colors.RESET}"
            
    def format_temperature(self, temp_str: str) -> str:
        """Format temperature with color based on value"""
        try:
            temp_val = float(temp_str.replace('°C', ''))
            if temp_val > 80:
                return f"{Colors.RED}{temp_str}{Colors.RESET}"
            elif temp_val > 70:
                return f"{Colors.YELLOW}{temp_str}{Colors.RESET}"
            else:
                return f"{Colors.GREEN}{temp_str}{Colors.RESET}"
        except:
            return temp_str
            
        
    def update_data(self):
        """Update all data sources"""
        try:
            self.service_status = self.get_service_status()
            self.recording_status = self.get_recording_info()
            self.ros_nodes = self.get_ros_nodes()
            self.system_info = self.get_system_info()
            self.last_errors = self.get_recent_errors()
            self.last_update = time.time()
        except Exception as e:
            pass
            
    def draw_screen(self):
        """Draw the complete screen - optimized for raw mode with manual cursor control"""
        
        # Use manual cursor positioning for raw mode
        row = 1
        
        # Clear screen completely first
        print('\033[2J\033[3J\033[1;1H\033[0m', end='')
        sys.stdout.flush()
        time.sleep(0.1)  # Ensure clear completes
        
        # Argo Status (using cached status output for responsiveness)
        argo_status_output = self.get_cached_status_output()
        if not argo_status_output.strip():
            argo_status_output = "Status check in progress..."
        status_lines = argo_status_output.strip().split('\n')
        
        # Header
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        print(f'\033[{row};1H{Colors.BOLD}{Colors.CYAN}🚢 Argo Sailboat Control Center{Colors.RESET}', end='')
        row += 1
        print(f'\033[{row};1H{Colors.DIM}Last updated: {timestamp}{Colors.RESET}', end='')
        row += 1
        print(f'\033[{row};1H{"=" * self.terminal_width}', end='')
        row += 1
        
        
        for line in status_lines:
            if line.strip():  # Skip empty lines
                print(f'\033[{row};1H{line}', end='')
                row += 1
                if row > self.terminal_height - 5:  # Leave space for commands
                    break
        
        # Add separator line after status
        row += 1
        print(f'\033[{row};1H{"=" * self.terminal_width}', end='')
        row += 1
        
        # System Info
        print(f'\033[{row};1H{Colors.BOLD}System Info:{Colors.RESET}', end='')
        row += 1
        
        if 'cpu_temp' in self.system_info:
            temp_str = self.format_temperature(self.system_info['cpu_temp'])
            print(f'\033[{row};1H  CPU Temp: {temp_str}', end='')
            row += 1
        if 'memory' in self.system_info:
            print(f'\033[{row};1H  Memory:   {self.system_info["memory"]}', end='')
            row += 1
        if 'disk' in self.system_info:
            print(f'\033[{row};1H  Disk:     {self.system_info["disk"]}', end='')
            row += 1
        
        # Errors
        if self.last_errors:
            print(f'\033[{row};1H{Colors.BOLD}{Colors.RED}Recent Errors:{Colors.RESET}', end='')
            row += 1
            for error in self.last_errors[-3:]:  # Show last 3 errors
                print(f'\033[{row};1H  {Colors.RED}•{Colors.RESET} {error}', end='')
                row += 1
        
        # Commands (compact single line)
        print(f'\033[{row};1H{Colors.BOLD}Commands:{Colors.RESET} {Colors.BOLD}{Colors.GREEN}l{Colors.RESET}aunch {Colors.BOLD}{Colors.RED}h{Colors.RESET}alt {Colors.BOLD}{Colors.YELLOW}R{Colors.RESET}estart | {Colors.BOLD}{Colors.GREEN}r{Colors.RESET}ecord {Colors.BOLD}{Colors.RED}c{Colors.RESET}lose | {Colors.BOLD}{Colors.CYAN}?{Colors.RESET}elp {Colors.BOLD}{Colors.MAGENTA}Enter{Colors.RESET}refresh {Colors.BOLD}{Colors.MAGENTA}Ctrl+C{Colors.RESET}exit | {Colors.DIM}Auto-refresh 10s{Colors.RESET}', end='')
        row += 1
        
        # Clear any remaining lines
        for i in range(row + 1, self.terminal_height + 1):
            print(f'\033[{i};1H\033[K', end='')  # Clear line
        
        sys.stdout.flush()
        
    def execute_command(self, cmd: str) -> tuple[bool, str]:
        """Execute a command and return (success_status, message)"""
        try:
            if cmd == 'l':  # Launch service
                result = subprocess.run(['make', '-C', self.argo_dir, 'start'], 
                                      capture_output=True, text=True, timeout=30)
                if result.returncode == 0:
                    return True, "Service launched successfully"
                else:
                    return False, f"Launch failed: {result.stderr.strip() or 'Unknown error'}"
            elif cmd == 'h':  # Halt service
                result = subprocess.run(['make', '-C', self.argo_dir, 'stop'], 
                                      capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    return True, "Service halted successfully"
                else:
                    return False, f"Halt failed: {result.stderr.strip() or 'Unknown error'}"
            elif cmd == 'r':  # Start recording
                result = subprocess.run(['make', '-C', self.argo_dir, 'record'], 
                                      capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    return True, "Recording started successfully"
                else:
                    return False, f"Recording start failed: {result.stderr.strip() or 'Unknown error'}"
            elif cmd == 'c':  # Stop recording
                result = subprocess.run(['make', '-C', self.argo_dir, 'stop-record'], 
                                      capture_output=True, text=True, timeout=10)
                if result.returncode == 0:
                    return True, "Recording stopped successfully"
                else:
                    return False, f"Recording stop failed: {result.stderr.strip() or 'Unknown error'}"
            elif cmd == 'R':  # Restart service
                result = subprocess.run(['make', '-C', self.argo_dir, 'restart'], 
                                      capture_output=True, text=True, timeout=30)
                if result.returncode == 0:
                    return True, "Service restarted successfully"
                else:
                    return False, f"Restart failed: {result.stderr.strip() or 'Unknown error'}"
            elif cmd == '?':  # Show help
                self.show_help()
                return True, "Help displayed"
            else:
                return False, "Unknown command"
        except subprocess.TimeoutExpired:
            return False, "Command timed out"
        except Exception as e:
            return False, f"Error: {str(e)}"
            
    def execute_command_with_progress(self, cmd: str) -> tuple[bool, str]:
        """Execute a command with progress feedback and return (success_status, message)"""
        try:
            if cmd == 'l':  # Launch service
                result = self._execute_with_progress(['make', '-C', self.argo_dir, 'start'], 
                                                   "Launching argo ros nodes", 30)
                # Force status update after command execution
                self.force_status_update()
                return result
            elif cmd == 'h':  # Halt service
                result = self._execute_with_progress(['make', '-C', self.argo_dir, 'stop'], 
                                                   "Halting service", 10)
                # Force status update after command execution
                self.force_status_update()
                return result
            elif cmd == 'r':  # Start recording
                result = self._execute_with_progress(['make', '-C', self.argo_dir, 'record'], 
                                                   "Starting recording", 10)
                # Force status update after command execution
                self.force_status_update()
                return result
            elif cmd == 'c':  # Stop recording
                result = self._execute_with_progress(['make', '-C', self.argo_dir, 'stop-record'], 
                                                   "Stopping recording", 10)
                # Force status update after command execution
                self.force_status_update()
                return result
            elif cmd == 'R':  # Restart service
                result = self._execute_with_progress(['make', '-C', self.argo_dir, 'restart'], 
                                                   "Relaunching argo nodes", 30)
                # Force status update after command execution
                self.force_status_update()
                return result
            elif cmd == '?':  # Show help
                self.show_help()
                return True, "Help displayed"
            else:
                return False, "Unknown command"
        except Exception as e:
            return False, f"Error: {str(e)}"
            
    def _execute_with_progress(self, cmd: list, message: str, timeout: int) -> tuple[bool, str]:
        """Execute a command with progress dots and return (success_status, message)"""
        import threading
        import time
        
        # Start the process
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        
        # Progress feedback thread
        progress_dots = 0
        progress_running = True
        
        def show_progress():
            nonlocal progress_dots
            while progress_running:
                progress_dots += 1
                dots = "." * progress_dots  # Keep adding dots
                print(f'\033[2;1H{Colors.CYAN}{message}{dots}{Colors.RESET}', end='')
                sys.stdout.flush()
                time.sleep(1)
        
        # Start progress thread
        progress_thread = threading.Thread(target=show_progress)
        progress_thread.daemon = True
        progress_thread.start()
        
        try:
            # Wait for process to complete
            stdout, stderr = process.communicate(timeout=timeout)
            progress_running = False
            progress_thread.join(timeout=1)  # Give thread time to finish
            
            if process.returncode == 0:
                return True, f"{message} completed successfully"
            else:
                return False, f"{message} failed: {stderr.strip() or 'Unknown error'}"
        except subprocess.TimeoutExpired:
            progress_running = False
            process.kill()
            progress_thread.join(timeout=1)  # Give thread time to finish
            return False, f"{message} timed out"
        except Exception as e:
            progress_running = False
            progress_thread.join(timeout=1)  # Give thread time to finish
            return False, f"{message} error: {str(e)}"
            
    def show_help(self):
        """Show detailed help information"""
        self.clear_screen()
        print(f"{Colors.BOLD}{Colors.CYAN}Argo Sailboat CLI GUI Help{Colors.RESET}")
        print("=" * 50)
        print()
        print(f"{Colors.BOLD}Prerequisites:{Colors.RESET}")
        print(f"  {Colors.YELLOW}⚠ Requires sudo privileges to control systemd services{Colors.RESET}")
        print(f"  Run with: {Colors.BOLD}sudo python3 launch/argo_gui.py{Colors.RESET}")
        print(f"  Or use: {Colors.BOLD}sudo ./launch/argo_gui.sh{Colors.RESET}")
        print()
        print(f"{Colors.BOLD}Service Control:{Colors.RESET}")
        print(f"  {Colors.BOLD}{Colors.GREEN}l{Colors.RESET} - Launch Argo service (with 30s monitoring)")
        print(f"  {Colors.BOLD}{Colors.RED}h{Colors.RESET} - Halt/Stop Argo service")
        print(f"  {Colors.BOLD}{Colors.YELLOW}R{Colors.RESET} - Restart Argo service")
        print()
        print(f"{Colors.BOLD}Recording Control:{Colors.RESET}")
        print(f"  {Colors.BOLD}{Colors.GREEN}r{Colors.RESET} - Start ROS2 bag recording")
        print(f"  {Colors.BOLD}{Colors.RED}c{Colors.RESET} - Stop ROS2 bag recording")
        print()
        print(f"{Colors.BOLD}Other Commands:{Colors.RESET}")
        print(f"  {Colors.BOLD}{Colors.CYAN}?{Colors.RESET} - Show this help")
        print(f"  {Colors.BOLD}{Colors.MAGENTA}Enter{Colors.RESET} - Refresh screen")
        print(f"  {Colors.BOLD}{Colors.MAGENTA}Ctrl+C{Colors.RESET} - Exit GUI")
        print()
        print(f"{Colors.BOLD}Auto-refresh:{Colors.RESET} Every 10 seconds")
        print()
        print(f"{Colors.BOLD}Status Indicators:{Colors.RESET}")
        print(f"  {Colors.GREEN}●{Colors.RESET} - Active/Running")
        print(f"  {Colors.RED}●{Colors.RESET} - Inactive/Stopped")
        print(f"  {Colors.YELLOW}?{Colors.RESET} - Unknown status")
        print()
        print(f"{Colors.BOLD}Temperature Colors:{Colors.RESET}")
        print(f"  {Colors.GREEN}Green{Colors.RESET} - Normal (<70°C)")
        print(f"  {Colors.YELLOW}Yellow{Colors.RESET} - Warm (70-80°C)")
        print(f"  {Colors.RED}Red{Colors.RESET} - Hot (>80°C)")
        print()
        print("Press any key to return to main screen...")
        
        # Wait for keypress
        try:
            import tty, termios
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except:
            input()  # Fallback for systems without termios
            
    def run(self):
        """Main GUI loop with automatic refresh and single keystroke input"""
        print(f"{Colors.GREEN}Starting Argo CLI GUI...{Colors.RESET}")
        time.sleep(1)
        
        # Initial data update and status load
        self.update_data()
        # Force initial status load
        self.force_status_update()
        self.draw_screen()
        
        # Set up terminal for single keystroke input
        import tty
        import termios
        import select
        
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            # Set terminal to raw mode for single keystroke input
            tty.setraw(sys.stdin.fileno())
            
            while self.running:
                try:
                    # Check for input with very short timeout for responsiveness
                    if select.select([sys.stdin], [], [], 0.05)[0]:
                        key = sys.stdin.read(1)
                        
                        if key == '\x03':  # Ctrl+C
                            print('\033[2J\033[H', end='')  # Clear screen
                            print(f'{Colors.YELLOW}Ctrl+C detected, exiting...{Colors.RESET}')
                            sys.stdout.flush()
                            break
                        elif key in 'lhrcR?':
                            # Show command confirmation after current status output
                            print(f'\033[2;1H{Colors.CYAN}Executing command: {key}{Colors.RESET}', end='')
                            sys.stdout.flush()
                            
                            # Execute command with progress feedback
                            success, message = self.execute_command_with_progress(key)
                            
                            # Show result after command completes
                            if success:
                                print(f'\033[3;1H{Colors.GREEN}✓ Command executed successfully{Colors.RESET}', end='')
                                if message:
                                    print(f'\033[4;1H{Colors.GREEN}{message}{Colors.RESET}', end='')
                            else:
                                print(f'\033[3;1H{Colors.RED}✗ Command failed{Colors.RESET}', end='')
                                if message:
                                    print(f'\033[4;1H{Colors.RED}{message}{Colors.RESET}', end='')
                            
                            time.sleep(1.5)  # Increased delay for better visibility
                            
                            # Now clear screen and redraw with updated status
                            print('\033[2J\033[3J\033[1;1H\033[0m', end='')
                            sys.stdout.flush()
                            time.sleep(0.1)
                            
                            # Update data and redraw screen after command completes
                            self.update_data()
                            self.draw_screen()
                        elif key == '\r' or key == '\n':  # Enter key
                            # Show immediate acknowledgment
                            print('\033[2J\033[3J\033[1;1H\033[0m', end='')
                            sys.stdout.flush()
                            time.sleep(0.1)
                            print(f'\033[1;1H{Colors.CYAN}Refreshing...{Colors.RESET}', end='')
                            sys.stdout.flush()
                            time.sleep(0.5)
                            
                            # Just refresh - ensure clean state
                            self.update_data()
                            self.draw_screen()
                        else:
                            # Unknown key - show brief message
                            print('\033[2J\033[3J\033[1;1H\033[0m', end='')
                            sys.stdout.flush()
                            time.sleep(0.1)
                            print(f'\033[1;1H{Colors.YELLOW}Unknown key: {repr(key)}{Colors.RESET}', end='')
                            print(f'\033[2;1H{Colors.DIM}Valid keys: l, h, r, c, R, ?, Enter{Colors.RESET}', end='')
                            sys.stdout.flush()
                            time.sleep(1)
                            self.draw_screen()
                    
                    # Auto-refresh every 10 seconds (only update data, not screen)
                    current_time = time.time()
                    if current_time - self.last_update >= self.refresh_interval:
                        self.update_data()
                        # Don't redraw screen during auto-refresh to avoid clearing status
                        
                except KeyboardInterrupt:
                    print('\033[2J\033[H', end='')  # Clear screen
                    print(f'{Colors.YELLOW}Ctrl+C detected, exiting...{Colors.RESET}')
                    sys.stdout.flush()
                    break
                except Exception as e:
                    # Continue running even if there's an error
                    time.sleep(0.1)
                    
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
        self.cleanup_screen()
        print(f"{Colors.YELLOW}Argo GUI stopped{Colors.RESET}")

def main():
    parser = argparse.ArgumentParser(
        description='Argo Sailboat CLI GUI - Interactive control interface',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  sudo python3 launch/argo_gui.py     # Start GUI (requires sudo)
  sudo ./launch/argo_gui.sh           # Start GUI using launcher script
  python3 launch/argo_gui.py --help   # Show help

Key Commands:
  s - Start Argo service
  q - Quit Argo service  
  r - Start recording
  c - Stop recording
  R - Restart service
  h - Show help
  Ctrl+C - Exit GUI

Note: This GUI requires sudo privileges to control systemd services.
        """
    )
    
    args = parser.parse_args()
    
    # No directory check needed - script uses absolute paths for all operations
    
    # Check for sudo privileges
    require_sudo()
        
    # Start the GUI
    gui = ArgoGUI()
    gui.run()

if __name__ == '__main__':
    main()
