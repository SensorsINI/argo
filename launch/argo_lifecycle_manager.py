#!/usr/bin/env python3
"""
Argo ROS2 Lifecycle Manager
===========================

A robust lifecycle manager for Argo ROS2 nodes that provides:
- Process monitoring and auto-restart
- Graceful shutdown handling
- Status reporting
- Simple command-line interface

Usage:
    python3 argo_lifecycle_manager.py start    # Start all nodes
    python3 argo_lifecycle_manager.py stop     # Stop all nodes
    python3 argo_lifecycle_manager.py restart  # Restart all nodes
    python3 argo_lifecycle_manager.py status   # Show status
    python3 argo_lifecycle_manager.py monitor  # Monitor mode
"""

import os
import sys
import time
import signal
import subprocess
import threading
import argparse
from datetime import datetime
from typing import Dict, List, Optional

# Import centralized node utilities
from argo_node_utils import ArgoNodeManager
import psutil

class ArgoLifecycleManager:
    def __init__(self):
        self.argo_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.process = None
        self.node_processes = []
        self.monitoring = False
        self.restart_count = 0
        self.max_restarts = 5
        self.restart_delay = 2.0
        self.stabilization_wait = 15.0  # Additional wait time for nodes to stabilize
        self.journal_since = 'today'
        
        # Initialize node manager for discovery
        self.node_manager = ArgoNodeManager(self.argo_dir)
        
        # Discover expected nodes dynamically
        discovered_nodes = self.node_manager.discover_nodes()
        
        # Convert to .py format for process matching and handle special nodes
        self.expected_nodes = []
        self.special_nodes = []
        
        for node in discovered_nodes:
            if node == 'foxglove_bridge':
                self.special_nodes.append(node)  # Special handling needed
            else:
                self.expected_nodes.append(f"{node}.py")  # Regular Python nodes
        
        # Combine all expected nodes for monitoring
        self.all_expected_nodes = self.expected_nodes + self.special_nodes
        
        # Define critical nodes (essential for boat operation)
        self.critical_nodes = ['pwm.py', 'control.py']
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n🛑 Received signal {signum}, shutting down...")
        self.monitoring = False
        self.stop()
        sys.exit(0)
    
    def _get_ros2_processes(self) -> List[psutil.Process]:
        """Get all ROS2 processes related to Argo using node manager"""
        try:
            all_processes = self.node_manager.get_all_ros_processes()
            # Convert to psutil.Process objects
            processes = []
            for node_name, node_processes in all_processes.items():
                for proc_info in node_processes:
                    try:
                        proc = psutil.Process(proc_info['pid'])
                        processes.append(proc)
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        continue
            return processes
        except Exception as e:
            print(f"⚠️  Error getting processes: {e}")
            return []
    
    def _is_launch_running(self) -> bool:
        """Check if any of the expected nodes are still running"""
        if self.process and self.process.poll() is None:
            return True
        
        # Check if any expected nodes are running
        node_status = self._get_node_status()
        running_nodes = [node for node, status in node_status.items() if "RUNNING" in status]
        return len(running_nodes) > 0
    
    def _launch_nodes_directly(self):
        """Launch all expected nodes directly without using ros2 launch"""
        print("🚀 Launching nodes directly...")
        
        # Get the nodes directory
        nodes_dir = os.path.join(self.argo_dir, 'nodes')
        
        # Discover available nodes
        discovered_nodes = self.node_manager.discover_nodes()
        node_scripts = [f"{node}.py" for node in discovered_nodes if node != 'foxglove_bridge']
        
        # Launch each node in a separate process
        self.node_processes = []
        self.node_output_threads = []
        
        # Launch regular Python nodes
        for script in node_scripts:
            script_path = os.path.join(nodes_dir, script)
            if os.path.exists(script_path):
                print(f"✅ Launching {script}...")
                # Launch each node with proper ROS2 environment
                # Use None for stdout/stderr so output goes directly to systemd journal
                cmd = ['bash', '-c', f'source /opt/ros/humble/setup.bash && python3 {script_path}']
                proc = subprocess.Popen(
                    cmd,
                    cwd=self.argo_dir,
                    stdout=None,  # Let output go to systemd journal
                    stderr=None,  # Let errors go to systemd journal
                    universal_newlines=True
                )
                self.node_processes.append(proc)
                print(f"✅ Launched {script} (PID: {proc.pid})")
            else:
                print(f"⚠️  Warning: {script} not found at {script_path}")
        
        # Launch special nodes (like foxglove_bridge)
        for special_node in discovered_nodes:
            if special_node == 'foxglove_bridge':
                print(f"✅ Launching {special_node}...")
                # Launch foxglove_bridge as a ROS2 package
                cmd = ['bash', '-c', f'source /opt/ros/humble/setup.bash && ros2 run foxglove_bridge foxglove_bridge']
                proc = subprocess.Popen(
                    cmd,
                    cwd=self.argo_dir,
                    stdout=None,  # Let output go to systemd journal
                    stderr=None,  # Let errors go to systemd journal
                    universal_newlines=True
                )
                self.node_processes.append(proc)
                print(f"✅ Launched {special_node} (PID: {proc.pid})")
        
        # Set the main process to the first node process for compatibility
        if self.node_processes:
            self.process = self.node_processes[0]
        else:
            print("❌ No nodes were launched")
            self.process = None
    
    def _get_node_status(self) -> Dict[str, str]:
        """Get status of individual nodes"""
        status = {}
        processes = self._get_ros2_processes()
        
        # Check all expected nodes (regular Python nodes + special nodes)
        for node in self.all_expected_nodes:
            node_running = False
            for proc in processes:
                try:
                    cmdline = ' '.join(proc.cmdline() or [])
                    # Handle different node types
                    if node == 'foxglove_bridge':
                        # Special case: foxglove_bridge runs as "ros2 run foxglove_bridge foxglove_bridge"
                        if 'foxglove_bridge' in cmdline and 'ros2 run' in cmdline:
                            node_running = True
                            break
                    else:
                        # Regular Python nodes: look for the .py filename
                        if node in cmdline:
                            node_running = True
                            break
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            status[node] = "🟢 RUNNING" if node_running else "🔴 STOPPED"
        
        return status
    
    def _is_critical_node(self, node: str) -> bool:
        """Check if a node is critical for boat operation"""
        return node in self.critical_nodes
    
    def _should_restart_node(self, node: str, running_nodes: List[str]) -> bool:
        """Determine if a failed node should be restarted"""
        # Don't restart critical nodes if they're running
        if self._is_critical_node(node) and node in running_nodes:
            return False
        
        # Don't restart if we have too few total nodes
        if len(running_nodes) < 3:
            return False
            
        # Restart non-critical nodes if we have enough critical ones
        critical_running = [n for n in self.critical_nodes if n in running_nodes]
        if len(critical_running) == len(self.critical_nodes):
            return True
            
        return False

    def start(self) -> bool:
        """Start the Argo launch process"""
        if self._is_launch_running():
            print("⚠️  Argo launch process is already running")
            return True
        
        print("🚀 Starting Argo ROS2 nodes...")
        try:
            # Start the nodes directly
            self._launch_nodes_directly()
            
            # Wait and check for actual node startup with continuous feedback
            print("⏳ Waiting for nodes to start...")
            
            timeout = 30  # 30 second timeout
            check_interval = .3  # Check every .3 second
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                if not self._is_launch_running():
                    print("❌ Launch process died during startup")
                    
                    # Log that process died (output is now in systemd journal)
                    if self.process and self.process.poll() is not None:
                        print("💀 Process died - check systemd journal for detailed output")
                    
                    return False
                
                # Output is now handled directly by systemd journal
                
                # Check which nodes are running
                node_status = self._get_node_status()
                running_nodes = [node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [node for node, status in node_status.items() if "STOPPED" in status]
                
                # Show progress periodically (journal-friendly)
                elapsed = time.time() - start_time
                if elapsed > 0 and int(elapsed * 10) % 10 == 0:  # Print every second
                    if running_nodes and stopped_nodes:
                        print(f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                        print(f"   Not started: {', '.join(stopped_nodes)}")
                    elif running_nodes:
                        print(f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                    else:
                        print(f"⏳ Waiting for nodes to start... (no nodes detected yet)")
                
                # If all expected nodes are running, wait for stabilization with monitoring
                if len(running_nodes) == len(self.all_expected_nodes):
                    print(f"\n✅ All {len(running_nodes)} nodes detected: {', '.join(running_nodes)}")
                    print(f"⏳ Monitoring nodes during {self.stabilization_wait}s stabilization period...")
                    
                    # Monitor nodes during stabilization period
                    stabilization_start = time.time()
                    stabilization_check_interval = 1.0  # Check every second during stabilization
                    
                    while time.time() - stabilization_start < self.stabilization_wait:
                        # Check for node failures during stabilization
                        current_status = self._get_node_status()
                        current_running = [node for node, status in current_status.items() if "RUNNING" in status]
                        current_stopped = [node for node, status in current_status.items() if "STOPPED" in status]
                        
                        if current_stopped:
                            # Node(s) failed during stabilization
                            print(f"\n⚠️  Node failure detected during stabilization: {', '.join(current_stopped)}")
                            
                            # Get error messages for failed nodes
                            fatal_messages = self._get_fatal_messages_for_nodes()
                            for failed_node in current_stopped:
                                if failed_node in fatal_messages:
                                    print(f"   {failed_node}: {fatal_messages[failed_node]}")
                            
                            # Break out of stabilization to handle the failure
                            break
                        
                        # Show progress during stabilization (journal-friendly)
                        remaining_time = self.stabilization_wait - (time.time() - stabilization_start)
                        if remaining_time > 0 and int(remaining_time) % 2 == 0:  # Only print every 2 seconds
                            print(f"⏳ Stabilizing... {remaining_time:.1f}s remaining ({len(current_running)}/{len(self.all_expected_nodes)} nodes)")
                        
                        time.sleep(stabilization_check_interval)
                    
                    # Final check after stabilization period
                    final_node_status = self._get_node_status()
                    final_running_nodes = [node for node, status in final_node_status.items() if "RUNNING" in status]
                    
                    if len(final_running_nodes) == len(self.all_expected_nodes):
                        print(f"✅ Argo launch process started successfully")
                        print(f"✅ All {len(final_running_nodes)} nodes running and stable: {', '.join(final_running_nodes)}")
                        return True
                    else:
                        failed_nodes = [node for node in self.all_expected_nodes if node not in final_running_nodes]
                        print(f"⚠️  Some nodes failed during stabilization period")
                        print(f"   Still running: {', '.join(final_running_nodes)}")
                        print(f"   Failed nodes: {', '.join(failed_nodes)}")
                        
                        # Check if we have critical nodes running
                        critical_running = [node for node in self.critical_nodes if node in final_running_nodes]
                        
                        if len(critical_running) == len(self.critical_nodes):
                            print(f"✅ Critical nodes operational: {', '.join(critical_running)}")
                            print(f"✅ Argo will continue operating with {len(final_running_nodes)}/{len(self.all_expected_nodes)} nodes")
                            return True
                        elif len(final_running_nodes) >= 3:  # At least 3 nodes running
                            print(f"✅ Sufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(f"✅ Argo will continue operating with available sensors")
                            return True
                        else:
                            print(f"❌ Insufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in final_running_nodes])}")
                            return False
                
                time.sleep(check_interval)
            
            # Timeout reached - show final status
            print(f"\n⚠️  Timeout reached after {timeout}s")
            node_status = self._get_node_status()
            running_nodes = [node for node, status in node_status.items() if "RUNNING" in status]
            stopped_nodes = [node for node, status in node_status.items() if "STOPPED" in status]
            
            if running_nodes:
                failed_nodes = [node for node in self.all_expected_nodes if node not in running_nodes]
                print(f"✅ {len(running_nodes)} nodes running: {', '.join(running_nodes)}")
                if failed_nodes:
                    print(f"⚠️  {len(failed_nodes)} nodes not started: {', '.join(failed_nodes)}")
                
                # Check if we have critical nodes running
                critical_running = [node for node in self.critical_nodes if node in running_nodes]
                
                if len(critical_running) == len(self.critical_nodes):
                    print(f"✅ Critical nodes operational: {', '.join(critical_running)}")
                    print(f"✅ Argo will continue operating with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                    return True
                elif len(running_nodes) >= 3:  # At least 3 nodes running
                    print(f"✅ Sufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                    print(f"✅ Argo will continue operating with available sensors")
                    return True
                else:
                    print(f"❌ Insufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                    print(f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in running_nodes])}")
                    return False
            else:
                print(f"❌ No nodes running after timeout")
                return False
                
        except Exception as e:
            print(f"❌ Error starting Argo: {e}")
            return False
    
    def continuous(self) -> bool:
        """Start Argo and keep it running with fault tolerance"""
        print("🚀 Starting Argo ROS2 nodes in continuous mode...")
        
        # Start the launch process
        if not self.start():
            return False
        
        print("🔄 Starting continuous monitoring...")
        print("   Press Ctrl+C to stop")
        
        try:
            while True:
                time.sleep(10)  # Check every 10 seconds
                
                # Check if launch process is still running
                if not self._is_launch_running():
                    print("❌ Launch process died, restarting...")
                    if not self.start():
                        print("❌ Failed to restart launch process")
                        return False
                
                # Check node status
                node_status = self._get_node_status()
                running_nodes = [node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [node for node, status in node_status.items() if "STOPPED" in status]
                
                if stopped_nodes:
                    print(f"⚠️  {len(stopped_nodes)} nodes stopped: {', '.join(stopped_nodes)}")
                    
                    # Check if we need to restart
                    critical_running = [n for n in self.critical_nodes if n in running_nodes]
                    
                    if len(critical_running) < len(self.critical_nodes):
                        print(f"❌ Critical nodes missing, restarting system...")
                        self.stop()
                        time.sleep(2)
                        if not self.start():
                            print("❌ Failed to restart system")
                            return False
                    elif len(running_nodes) < 3:
                        print(f"⚠️  Too few nodes running ({len(running_nodes)}), but critical nodes OK")
                    else:
                        print(f"✅ System operational with {len(running_nodes)}/{len(self.expected_nodes)} nodes")
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping continuous monitoring...")
            self.stop()
            return True
        except Exception as e:
            print(f"❌ Error in continuous mode: {e}")
            return False
    
    def stop(self) -> bool:
        """Stop the Argo launch process and all related nodes"""
        print("🛑 Stopping Argo ROS2 nodes...")
        
        success = True
        
        # Stop main launch process
        if self.process and self._is_launch_running():
            try:
                self.process.terminate()
                self.process.wait(timeout=5)
                print("✅ Main launch process stopped")
            except subprocess.TimeoutExpired:
                print("⚠️  Force killing launch process...")
                self.process.kill()
                self.process.wait()
            except Exception as e:
                print(f"⚠️  Error stopping launch process: {e}")
                success = False
        
        # Stop individual node processes
        try:
            if hasattr(self, 'node_processes') and self.node_processes:
                print(f"🛑 Stopping {len(self.node_processes)} node processes...")
                for proc in self.node_processes:
                    if proc and proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            print(f"⚡ Force killing process {proc.pid}")
                            proc.kill()
                            proc.wait()
                self.node_processes = []
            
            # Also use pkill as backup to catch any remaining processes
            for node in self.expected_nodes:
                subprocess.run(['pkill', '-f', f'/{node}'], 
                             capture_output=True, timeout=2)
            
            print("✅ Argo processes terminated")
        except Exception as e:
            print(f"⚠️  Error stopping processes: {e}")
            success = False
        
        self.process = None
        print("✅ All Argo processes stopped")
        return success
    
    def restart(self) -> bool:
        """Restart the Argo launch process"""
        print("🔄 Restarting Argo ROS2 nodes...")
        self.stop()
        time.sleep(1)
        return self.start()
    
    def status(self) -> None:
        """Show current status of Argo nodes"""
        print(f"🚢 ARGO STATUS - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 60)
        
        # Check if argo-launch.service is running
        service_running = False
        try:
            result = subprocess.run(['systemctl', 'is-active', 'argo-launch.service'], 
                                  capture_output=True, text=True, timeout=2)
            service_running = result.returncode == 0 and result.stdout.strip() == 'active'
        except Exception:
            service_running = False
        
        # Main launch process status
        if self._is_launch_running():
            print("📋 LAUNCH PROCESS: 🟢 RUNNING")
        else:
            print("📋 LAUNCH PROCESS: 🔴 STOPPED")
        
        if service_running:
            print("📋 SYSTEMD SERVICE: 🟢 RUNNING")
        else:
            print("📋 SYSTEMD SERVICE: 🔴 STOPPED")
        
        # Get FATAL messages for stopped nodes if service is running
        node_fatal_messages = {}
        if service_running:
            node_fatal_messages = self._get_fatal_messages_for_nodes()
        
        # Individual node status
        print("🤖 ROS NODES:")
        node_status = self._get_node_status()
        stopped_nodes = []
        for node, status in node_status.items():
            if "STOPPED" in status and node in node_fatal_messages:
                print(f"  {node}: {status} - {node_fatal_messages[node]}")
            else:
                print(f"  {node}: {status}")
            if "STOPPED" in status:
                stopped_nodes.append(node)
        
        # Show key error messages for stopped nodes
        if stopped_nodes:
            print(f"\n⚠️  KEY ERROR MESSAGES (since {self.journal_since}):")
            try:
                # Get recent error messages from systemd journal
                result = subprocess.run([
                    'journalctl', '-u', 'argo-launch.service', '--since', '5 minutes ago',
                    '--grep', '(FATAL|ERROR|CRITICAL)', '--no-pager'
                ], capture_output=True, text=True, timeout=5)
                
                if result.returncode == 0 and result.stdout:
                    # Parse and show relevant error messages
                    lines = result.stdout.strip().split('\n')
                    recent_errors = []
                    
                    for line in lines:
                        # Check if this line contains errors from any stopped node
                        node_match = False
                        matched_node = None
                        for node in stopped_nodes:
                            node_name = node.replace('.py', '_node')
                            if node_name in line:
                                node_match = True
                                matched_node = node
                                break
                        
                        if node_match:
                            # Extract timestamp and message more robustly
                            if ']: ' in line:
                                parts = line.split(']: ', 1)
                                if len(parts) == 2:
                                    # Get timestamp (first 3 parts: month, day, time)
                                    timestamp_parts = parts[0].split(' ')[0:3]
                                    timestamp = ' '.join(timestamp_parts)
                                    message = parts[1]
                                    recent_errors.append(f"  {matched_node}: {timestamp} - {message}")
                            else:
                                # Fallback if parsing fails
                                recent_errors.append(f"  {matched_node}: {line}")
                    
                    if recent_errors:
                        # Group errors by node to ensure we show at least one error per stopped node
                        errors_by_node = {}
                        for error in recent_errors:
                            # Extract node name from error line
                            node_name = None
                            for node in stopped_nodes:
                                if node in error:
                                    node_name = node
                                    break
                            if node_name:
                                if node_name not in errors_by_node:
                                    errors_by_node[node_name] = []
                                errors_by_node[node_name].append(error)
                        
                        # Show errors, prioritizing FATAL messages and ensuring each stopped node is represented
                        display_errors = []
                        
                        # First, collect all FATAL errors
                        for node_errors in errors_by_node.values():
                            fatal_node_errors = [e for e in node_errors if 'FATAL' in e]
                            if fatal_node_errors:
                                display_errors.append(fatal_node_errors[-1])  # Most recent FATAL per node
                        
                        # Then add other errors if we have space (max 8 total)
                        for node_errors in errors_by_node.values():
                            other_errors = [e for e in node_errors if 'FATAL' not in e]
                            if other_errors and len(display_errors) < 8:
                                display_errors.append(other_errors[-1])  # Most recent other error per node
                        
                        # Sort by timestamp (newest first) and limit to 8 errors
                        display_errors = display_errors[-8:]
                        
                        for error in display_errors:
                            print(error)
                    else:
                        print("  No specific node errors found in recent logs")
                else:
                    print("  Unable to retrieve error messages from systemd journal")
            except Exception as e:
                print(f"  Error retrieving messages: {e}")
        
        # If nothing is running, provide I2C bus health info to help diagnostics
        try:
            launch_stopped = not self._is_launch_running()
            all_nodes_stopped = all("STOPPED" in s for s in node_status.values()) if node_status else True
            if launch_stopped or all_nodes_stopped:
                print("\n🔌 I2C BUS HEALTH (bus 0):")
                self._print_i2c_health(bus=0)
        except Exception as e:
            print(f"⚠️  I2C health check failed: {e}")
        
        # System info
        try:
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            # add free disk space
            disk = psutil.disk_usage("/")
            free_disk = disk.free / (1024**3)
            print(f"📊 SYSTEM: CPU {cpu_percent:.1f}% | Memory {memory.percent:.1f}% | Free Disk {free_disk:.1f}GB ({disk.percent:.1f}% used)  ")
        except:
            print("📊 SYSTEM: Unable to get system info")
        
        print("=" * 60)
    
    def monitor(self) -> None:
        """Monitor mode - watch for failures and auto-restart"""
        print("👁️  Starting Argo monitor mode...")
        print("Press Ctrl+C to stop monitoring")
        
        self.monitoring = True
        last_status_check = 0
        status_interval = 10  # Check status every 10 seconds
        
        try:
            while self.monitoring:
                current_time = time.time()
                
                # Check if we need to restart
                if not self._is_launch_running() and self.restart_count < self.max_restarts:
                    print(f"⚠️  Launch process died, restarting... (attempt {self.restart_count + 1}/{self.max_restarts})")
                    if self.start():
                        self.restart_count += 1
                        print(f"✅ Restarted successfully (restart count: {self.restart_count})")
                    else:
                        print("❌ Restart failed")
                        time.sleep(self.restart_delay)
                
                # Periodic status check
                if current_time - last_status_check >= status_interval:
                    self.status()
                    last_status_check = current_time
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n🛑 Monitor mode stopped by user")
        finally:
            self.monitoring = False

    def _get_i2c_addresses(self, bus: int = 0) -> List[int]:
        """Run i2cdetect and parse detected device addresses on the given bus."""
        try:
            proc = subprocess.run(
                ["i2cdetect", "-y", str(bus)],
                capture_output=True,
                text=True,
                timeout=1,
                check=True,
            )
            output = proc.stdout
        except FileNotFoundError:
            raise RuntimeError("i2cdetect not found. Install i2c-tools or ensure it's in PATH.")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"i2cdetect failed (exit {e.returncode}). stderr: {e.stderr}")
        except subprocess.TimeoutExpired:
            raise TimeoutError("i2cdetect timed out after 1s")

        detected: List[int] = []
        for line in output.splitlines():
            line = line.strip()
            if not line:
                continue
            # lines with rows look like: "10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --"
            if ":" not in line:
                continue
            try:
                _, cells_str = line.split(":", 1)
            except ValueError:
                continue
            cells = [c.strip() for c in cells_str.strip().split()]
            for cell in cells:
                # Valid device entries are two-hex-digit addresses; '--' is empty; 'UU' is in-use by driver
                if cell == "--" or cell == "UU":
                    continue
                # Accept hex like '34', '69'
                if len(cell) == 2:
                    try:
                        addr = int(cell, 16)
                        detected.append(addr)
                    except ValueError:
                        continue
        return sorted(set(detected))
    
    def _get_fatal_messages_for_nodes(self) -> Dict[str, str]:
        """Get the most recent FATAL message for each node from systemd journal"""
        fatal_messages = {}
        
        try:
            # Get recent FATAL messages from systemd journal for argo-launch.service
            # Look back further to catch initial startup failures
            result = subprocess.run([
                'journalctl', '-u', 'argo-launch.service', '--since', self.journal_since,
                '--grep', 'FATAL', '--no-pager', '-o', 'short-precise'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0 and result.stdout:
                lines = result.stdout.strip().split('\n')
                
                # Parse lines to extract node-specific FATAL messages
                # Process in reverse order to get the most recent message for each node
                for line in reversed(lines):
                    if 'FATAL' in line:
                        # Try to identify which node this FATAL message belongs to
                        for node in self.expected_nodes:
                            node_name = node.replace('.py', '')
                            # Look for node name patterns in the log line
                            if (f'{node_name}_node' in line or 
                                f'/{node_name}' in line or 
                                f'{node_name}.py' in line or
                                f'[{node_name}]' in line or
                                f'anemometer' in line.lower() and node_name == 'anem'):
                                
                                # Skip if we already have a message for this node (most recent)
                                if node in fatal_messages:
                                    continue
                                
                                # Extract the FATAL message part
                                if 'FATAL:' in line:
                                    # Find the FATAL message and extract a concise version
                                    fatal_part = line.split('FATAL:', 1)
                                    if len(fatal_part) > 1:
                                        message = fatal_part[1].strip()
                                        # Clean up the message - take first meaningful sentence
                                        if '.' in message:
                                            message = message.split('.')[0].strip()
                                        # Limit message length for display
                                        if len(message) > 50:
                                            message = message[:47] + "..."
                                        
                                        # Store the most recent FATAL for this node
                                        fatal_messages[node] = f"FATAL: {message}"
                                elif 'fatal(' in line.lower():
                                    # Handle ROS2 logger format: .fatal("message")
                                    import re
                                    match = re.search(r'fatal\("([^"]*)"', line, re.IGNORECASE)
                                    if match:
                                        message = match.group(1)
                                        if len(message) > 50:
                                            message = message[:47] + "..."
                                        fatal_messages[node] = f"FATAL: {message}"
                                break
                
        except Exception as e:
            # Silently fail - this is just for enhanced display
            pass
            
        return fatal_messages

    def _print_i2c_health(self, bus: int = 0) -> None:
        """Print a summary of I2C device presence vs expected sensors."""
        try:
            detected = self._get_i2c_addresses(bus)
        except TimeoutError as e:
            print(f"  I2C bus {bus} is malfunctioning ({e})")
            return
        except Exception as e:
            print(f"  I2C scan failed: {e}")
            return
        hex_list = ", ".join([f"0x{a:02x}" for a in detected]) if detected else "<none>"
        print(f"  Detected addresses: {hex_list}")

        expected_map = {
            "anem": [0x21, 0x22, 0x23],
            "battery_water": [0x34, 0x44],  # 0x34: ADC, 0x44: humidity
            "imu": [0x69],
        }

        for sensor, addrs in expected_map.items():
            present = [a for a in addrs if a in detected]
            missing = [a for a in addrs if a not in detected]
            if present and not missing:
                print(f"  {sensor}: 🟢 present ({', '.join([f'0x{x:02x}' for x in present])})")
            elif present and missing:
                print(
                    f"  {sensor}: 🟡 partially present (found {', '.join([f'0x{x:02x}' for x in present])}; missing {', '.join([f'0x{x:02x}' for x in missing])})"
                )
            else:
                print(f"  {sensor}: 🔴 not present (expected {', '.join([f'0x{x:02x}' for x in addrs])})")

def main():
    parser = argparse.ArgumentParser(description='Argo ROS2 Lifecycle Manager')
    parser.add_argument('command', choices=['start', 'stop', 'restart', 'status', 'monitor', 'continuous'],
                       help='Command to execute')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug output')
    
    args = parser.parse_args()
    
    manager = ArgoLifecycleManager()
    if args.debug:
        print("🔧 DEBUG: Debug mode enabled")
    
    if args.command == 'start':
        success = manager.start()
        if not success and args.debug:
            print("🔧 DEBUG: Start failed, showing detailed status...")
            manager.status()
        sys.exit(0 if success else 1)
    elif args.command == 'stop':
        success = manager.stop()
        sys.exit(0 if success else 1)
    elif args.command == 'restart':
        success = manager.restart()
        sys.exit(0 if success else 1)
    elif args.command == 'status':
        manager.status()
    elif args.command == 'monitor':
        manager.monitor()
    elif args.command == 'continuous':
        success = manager.continuous()
        sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
