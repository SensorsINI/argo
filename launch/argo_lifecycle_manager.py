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
    python3 argo_lifecycle_manager.py run      # Start all nodes and keep running (for systemd)
    python3 argo_lifecycle_manager.py stop     # Stop all nodes
    python3 argo_lifecycle_manager.py restart  # Restart all nodes
    python3 argo_lifecycle_manager.py status   # Show status
    python3 argo_lifecycle_manager.py monitor  # Monitor mode (watch for failures)
"""

import os
import sys
import time
import signal
import subprocess
import threading
import argparse
from datetime import datetime
from typing import Dict, List, Optional, Any
import json

# Import centralized node utilities
from argo_node_utils import ArgoNodeManager
import psutil

# ROS2 imports for service client
try:
    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

# Add config loading for remote simulation
try:
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(SCRIPT_DIR, '..', 'scripts'))
    from load_config import load_config
    REMOTE_CONFIG = load_config()
except ImportError:
    REMOTE_CONFIG = None


class ArgoLifecycleManager:
    def __init__(self):
        self.argo_dir = os.path.dirname(
            os.path.dirname(os.path.abspath(__file__)))
        self.process = None
        self.node_processes = []
        # Removed restart logic - nodes should not be restarted automatically
        # Failures should be preserved for debugging
        self.stabilization_wait = 15.0  # Additional wait time for nodes to stabilize
        self.journal_since = 'today'
        self.remote_simulator_proc = None
        self.remote_tunnel_proc = None

        # Initialize ROS2 for service client if available
        self.ros2_node = None
        self.battery_service_client = None
        if ROS2_AVAILABLE:
            try:
                if not rclpy.ok():
                    rclpy.init()
                self.ros2_node = Node('argo_lifecycle_manager')
                self.battery_service_client = self.ros2_node.create_client(
                    Trigger, '/battery_status')
            except Exception as e:
                print(
                    f"Warning: Could not initialize ROS2 service client: {e}")
                self.ros2_node = None
                self.battery_service_client = None

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
                self.expected_nodes.append(
                    f"{node}.py")  # Regular Python nodes

        # Combine all expected nodes for monitoring
        self.all_expected_nodes = self.expected_nodes + self.special_nodes

        # Define critical nodes (essential for boat operation)
        self.critical_nodes = ['pwm.py', 'control.py']

        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(
            f"\n🛑 argo_lifecycle_manager: Received signal {signum}, shutting down...")
        self._cleanup_ros2()
        self.stop()
        sys.exit(0)

    def _cleanup_ros2(self):
        """Clean up ROS2 resources"""
        if self.ros2_node:
            try:
                self.ros2_node.destroy_node()
            except Exception:
                pass
            self.ros2_node = None
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

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
        running_nodes = [node for node,
                         status in node_status.items() if "RUNNING" in status]
        return len(running_nodes) > 0

    def _launch_nodes_directly(self):
        """Launch all expected nodes directly without using ros2 launch"""
        print("🚀 Launching nodes directly...")

        # Get the nodes directory
        nodes_dir = os.path.join(self.argo_dir, 'nodes')

        # Use expected_nodes if defined (e.g., in simulation mode), otherwise discover all nodes
        if hasattr(self, 'expected_nodes') and self.expected_nodes:
            node_scripts = self.expected_nodes
            print(f"Launching specific nodes: {', '.join(node_scripts)}")
        else:
            # Discover available nodes, excluding simulation-only nodes for normal launches
            discovered_nodes = self.node_manager.discover_nodes(
                exclude_simulation_only=True)
            node_scripts = [
                f"{node}.py" for node in discovered_nodes if node != 'foxglove_bridge']

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
                cmd_str = f'source /opt/ros/humble/setup.bash && python3 {script_path}'
                if script == 'argo_unified_simulator_bridge.py' and hasattr(self, 'simulation_mode'):
                    cmd_str += f' --mode {self.simulation_mode}'
                cmd = ['bash', '-c', cmd_str]
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
        # In simulation mode, launch special nodes defined in self.special_nodes
        # In normal mode, discover and launch all special nodes
        special_nodes_to_launch = []
        if hasattr(self, 'special_nodes') and self.special_nodes:
            # Simulation mode: use explicitly defined special nodes
            special_nodes_to_launch = self.special_nodes
        elif not (hasattr(self, 'expected_nodes') and self.expected_nodes):
            # Normal mode: discover all special nodes
            discovered_nodes = self.node_manager.discover_nodes()
            special_nodes_to_launch = [
                node for node in discovered_nodes if node == 'foxglove_bridge']

        for special_node in special_nodes_to_launch:
            if special_node == 'foxglove_bridge':
                print(f"✅ Launching {special_node}...")
                # Launch foxglove_bridge as a ROS2 package
                cmd = [
                    'bash', '-c', f'source /opt/ros/humble/setup.bash && ros2 run foxglove_bridge foxglove_bridge']
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

    # Removed _should_restart_node method - nodes are not restarted automatically
    # Node failures are preserved for debugging purposes

    def continuous(self) -> bool:
        """Start Argo and keep it running with fault tolerance"""
        print("🚀 Starting Argo ROS2 nodes...")

        if self._is_launch_running():
            print("⚠️  Argo launch process is already running")
            return True

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
                        print(
                            "💀 Process died - check systemd journal for detailed output")

                    return False

                # Check which nodes are running
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]

                # Show progress periodically (journal-friendly)
                elapsed = time.time() - start_time
                if elapsed > 0 and int(elapsed * 10) % 10 == 0:  # Print every second
                    if running_nodes and stopped_nodes:
                        print(
                            f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                        print(f"   Not started: {', '.join(stopped_nodes)}")
                    elif running_nodes:
                        print(
                            f"⏳ Waiting for nodes to start... {len(running_nodes)}/{len(self.all_expected_nodes)} running: {', '.join(running_nodes)}")
                    else:
                        print(
                            f"⏳ Waiting for nodes to start... (no nodes detected yet)")

                # If all expected nodes are running, wait for stabilization with monitoring
                if len(running_nodes) == len(self.all_expected_nodes):
                    print(
                        f"\n✅ All {len(running_nodes)} nodes detected: {', '.join(running_nodes)}")
                    print(
                        f"⏳ Monitoring nodes during {self.stabilization_wait}s stabilization period...")

                    # Monitor nodes during stabilization period
                    stabilization_start = time.time()
                    stabilization_check_interval = 1.0  # Check every second during stabilization

                    while time.time() - stabilization_start < self.stabilization_wait:
                        # Check for node failures during stabilization
                        current_status = self._get_node_status()
                        current_running = [
                            node for node, status in current_status.items() if "RUNNING" in status]
                        current_stopped = [
                            node for node, status in current_status.items() if "STOPPED" in status]

                        if current_stopped:
                            # Node(s) failed during stabilization
                            print(
                                f"\n⚠️  Node failure detected during stabilization: {', '.join(current_stopped)}")

                            # Get error messages for failed nodes
                            fatal_messages = self._get_fatal_messages_for_nodes()
                            for failed_node in current_stopped:
                                if failed_node in fatal_messages:
                                    print(
                                        f"   {failed_node}: {fatal_messages[failed_node]}")

                            # Break out of stabilization to handle the failure
                            break

                        # Show progress during stabilization (journal-friendly)
                        remaining_time = self.stabilization_wait - \
                            (time.time() - stabilization_start)
                        # Only print every 2 seconds
                        if remaining_time > 0 and int(remaining_time) % 2 == 0:
                            print(
                                f"⏳ Stabilizing... {remaining_time:.1f}s remaining ({len(current_running)}/{len(self.all_expected_nodes)} nodes)")

                        time.sleep(stabilization_check_interval)

                    # Final check after stabilization period
                    final_node_status = self._get_node_status()
                    final_running_nodes = [
                        node for node, status in final_node_status.items() if "RUNNING" in status]

                    if len(final_running_nodes) == len(self.all_expected_nodes):
                        print(f"✅ Argo launch process started successfully")
                        print(
                            f"✅ All {len(final_running_nodes)} nodes running and stable: {', '.join(final_running_nodes)}")
                        break  # Continue to monitoring phase
                    else:
                        failed_nodes = [
                            node for node in self.all_expected_nodes if node not in final_running_nodes]
                        print(f"⚠️  Some nodes failed during stabilization period")
                        print(
                            f"   Still running: {', '.join(final_running_nodes)}")
                        print(f"   Failed nodes: {', '.join(failed_nodes)}")

                        # Check if we have critical nodes running
                        critical_running = [
                            node for node in self.critical_nodes if node in final_running_nodes]

                        if len(critical_running) == len(self.critical_nodes):
                            print(
                                f"✅ Critical nodes operational: {', '.join(critical_running)}")
                            print(
                                f"✅ Argo will continue operating with {len(final_running_nodes)}/{len(self.all_expected_nodes)} nodes")
                            break  # Continue to monitoring phase
                        elif len(final_running_nodes) >= 3:  # At least 3 nodes running
                            print(
                                f"✅ Sufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(
                                f"✅ Argo will continue operating with available sensors")
                            break  # Continue to monitoring phase
                        else:
                            print(
                                f"❌ Insufficient nodes running ({len(final_running_nodes)}/{len(self.all_expected_nodes)})")
                            print(
                                f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in final_running_nodes])}")
                            return False

                time.sleep(check_interval)

            # Check if we timed out
            if time.time() - start_time >= timeout:
                print(f"\n⚠️  Timeout reached after {timeout}s")
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]

                if running_nodes:
                    failed_nodes = [
                        node for node in self.all_expected_nodes if node not in running_nodes]
                    print(
                        f"✅ {len(running_nodes)} nodes running: {', '.join(running_nodes)}")
                    if failed_nodes:
                        print(
                            f"⚠️  {len(failed_nodes)} nodes not started: {', '.join(failed_nodes)}")

                    # Check if we have critical nodes running
                    critical_running = [
                        node for node in self.critical_nodes if node in running_nodes]

                    if len(critical_running) == len(self.critical_nodes):
                        print(
                            f"✅ Critical nodes operational: {', '.join(critical_running)}")
                        print(
                            f"✅ Argo will continue operating with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                    elif len(running_nodes) >= 3:  # At least 3 nodes running
                        print(
                            f"✅ Sufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                        print(
                            f"✅ Argo will continue operating with available sensors")
                    else:
                        print(
                            f"❌ Insufficient nodes running ({len(running_nodes)}/{len(self.all_expected_nodes)})")
                        print(
                            f"❌ Critical nodes missing: {', '.join([n for n in self.critical_nodes if n not in running_nodes])}")
                        return False
                else:
                    print(f"❌ No nodes running after timeout")
                    return False

        except Exception as e:
            print(f"❌ Error starting Argo: {e}")
            return False

        print("🔄 Starting continuous monitoring...")
        print("   Press Ctrl+C to stop")
        print("   NOTE: Node failures will be logged but NOT restarted for debugging")

        try:
            while True:
                time.sleep(30)  # Check every 30 seconds (less frequent)

                # Check node status
                node_status = self._get_node_status()
                running_nodes = [
                    node for node, status in node_status.items() if "RUNNING" in status]
                stopped_nodes = [
                    node for node, status in node_status.items() if "STOPPED" in status]

                if stopped_nodes:
                    # Log stopped nodes but do NOT restart them
                    print(
                        f"⚠️  {len(stopped_nodes)} nodes stopped: {', '.join(stopped_nodes)}")

                    # Check if critical nodes are still running
                    critical_running = [
                        n for n in self.critical_nodes if n in running_nodes]
                    critical_stopped = [
                        n for n in self.critical_nodes if n in stopped_nodes]

                    if critical_stopped:
                        print(
                            f"❌ CRITICAL NODES STOPPED: {', '.join(critical_stopped)}")
                        print(
                            f"   System will continue with remaining nodes for debugging")
                        print(f"   Check systemd journal for error details")
                    else:
                        print(
                            f"✅ Critical nodes operational: {', '.join(critical_running)}")

                    # Show system status
                    if len(running_nodes) >= 3:
                        print(
                            f"✅ System operational with {len(running_nodes)}/{len(self.all_expected_nodes)} nodes")
                    else:
                        print(
                            f"⚠️  Low node count: {len(running_nodes)}/{len(self.all_expected_nodes)} nodes running")

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
                print(
                    f"🛑 Stopping {len(self.node_processes)} node processes...")
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

            # Also kill special nodes (like foxglove_bridge)
            if hasattr(self, 'special_nodes') and self.special_nodes:
                for special_node in self.special_nodes:
                    if special_node == 'foxglove_bridge':
                        subprocess.run(['pkill', '-f', 'foxglove_bridge'],
                                       capture_output=True, timeout=2)

            print("✅ Argo processes terminated")
        except Exception as e:
            print(f"⚠️  Error stopping processes: {e}")
            success = False

        self.process = None
        # Stop remote processes if they were started
        self._stop_remote_processes()
        print("✅ All Argo processes stopped")
        return success

    def restart(self) -> bool:
        """Restart the Argo launch process"""
        print("🔄 Restarting Argo ROS2 nodes...")
        self.stop()
        time.sleep(1)
        return self.start()

    def simulate_local(self) -> bool:
        """Launch Argo in local simulation mode."""
        return self._simulate(mode='local')

    def simulate_remote(self) -> bool:
        """Launch Argo in remote simulation mode."""
        print("INFO: Remote simulation requires manual setup on the remote machine")
        print("      and running 'scripts/remote_simulator_tunnel.sh' on this machine.")
        # Start remote processes
        if not self._start_remote_tunnel():
            return False
        if not self._start_remote_simulator():
            self._stop_remote_processes()
            return False
        return self._simulate(mode='remote')

    def _simulate(self, mode: str) -> bool:
        """
        Launch Argo in simulation mode.

        In simulation mode, only essential nodes are launched:
        - argo_unified_simulator_bridge.py (provides simulated sensor data)
        - controller.py (autonomous navigation)
        - battery_water.py (hardware monitoring)
        - temp_monitor.py (hardware monitoring)
        - foxglove_bridge (provides visualization via Foxglove Studio)

        Hardware nodes that conflict with simulator are excluded:
        - gps.py (conflicts with simulator GPS topics)
        - imu.py (conflicts with simulator compass topics)
        - anem.py (conflicts with simulator wind topics)
        - rudder_sail_radio.py (conflicts with simulator control)
        """
        self.simulation_mode = mode
        print(f"🚢 Starting Argo in SIMULATION mode ({mode.upper()})...")
        if mode == 'local':
            print("Local simulation runs the simulator on this machine.")
        else:
            print("Remote simulation connects to a simulator on another machine.")

        print("Simulation mode excludes conflicting hardware nodes:")
        print("  - gps.py (GPS data provided by simulator)")
        print("  - imu.py (compass data provided by simulator)")
        print("  - anem.py (wind data provided by simulator)")
        print("  - rudder_sail_radio.py (control handled by simulator)")
        print("Simulation mode includes visualization:")
        print("  - foxglove_bridge (Foxglove Studio at ws://localhost:9090)")

        # Check if simulator bridge exists
        simulator_bridge_path = os.path.join(
            self.argo_dir, "nodes", "argo_unified_simulator_bridge.py")
        if not os.path.exists(simulator_bridge_path):
            print(f"❌ Simulator bridge not found: {simulator_bridge_path}")
            return False

        # Define simulation mode node scripts (exclude conflicting hardware nodes)
        self.expected_nodes = [
            # Provides simulated sensor data + keyboard control
            "argo_unified_simulator_bridge.py",
            "controller.py",                     # Autonomous navigation
            "battery_water.py",                  # Hardware monitoring
            "temp_monitor.py"                    # Hardware monitoring
        ]

        # Add foxglove_bridge to special nodes for simulation mode
        self.special_nodes = ["foxglove_bridge"]

        # Critical nodes for simulation (simulator bridge is critical)
        self.critical_nodes = ["argo_unified_simulator_bridge.py"]

        print(f"Expected simulation nodes: {', '.join(self.expected_nodes)}")
        print(f"Special simulation nodes: {', '.join(self.special_nodes)}")
        print(f"Critical simulation nodes: {', '.join(self.critical_nodes)}")

        # Launch simulation nodes
        self._launch_nodes_directly()

        # Wait for nodes to start
        print("⏳ Waiting for simulation nodes to start...")
        start_time = time.time()
        timeout = 30
        check_interval = 0.3

        # Calculate total expected nodes (regular + special)
        total_expected_nodes = len(
            self.expected_nodes) + len(self.special_nodes)

        while time.time() - start_time < timeout:
            node_status = self._get_node_status()
            running_nodes = [
                node for node, status in node_status.items() if "RUNNING" in status]

            # Progress reporting (journal-friendly)
            elapsed = time.time() - start_time
            if int(elapsed * 10) % 10 == 0:
                print(
                    f"⏳ Waiting for simulation nodes... {len(running_nodes)}/{total_expected_nodes} running")

            if len(running_nodes) == total_expected_nodes:
                break  # All nodes detected, proceed to stabilization

            time.sleep(check_interval)

        # Check if all expected nodes are running
        if len(running_nodes) != total_expected_nodes:
            print(
                f"❌ Only {len(running_nodes)}/{total_expected_nodes} simulation nodes started")
            print(f"Running: {running_nodes}")
            print(f"Expected: {self.expected_nodes + self.special_nodes}")
            return False

        # Monitor during stabilization period
        print("⏳ Monitoring simulation nodes during stabilization...")
        stabilization_start = time.time()
        self.stabilization_wait = 15.0  # 15 seconds stabilization for simulation

        while time.time() - stabilization_start < self.stabilization_wait:
            current_status = self._get_node_status()
            current_stopped = [
                node for node, status in current_status.items() if "STOPPED" in status]

            if current_stopped:
                print(
                    f"\n⚠️  Simulation node failure detected during stabilization: {', '.join(current_stopped)}")
                # Get and display error messages
                fatal_messages = self._get_fatal_messages_for_nodes()
                for failed_node in current_stopped:
                    if failed_node in fatal_messages:
                        print(
                            f"   {failed_node}: {fatal_messages[failed_node]}")
                break

            time.sleep(1.0)  # Check every second

        # Final status check
        final_node_status = self._get_node_status()
        final_running_nodes = [
            node for node, status in final_node_status.items() if "RUNNING" in status]

        # Determine success based on critical nodes and minimum thresholds
        critical_running = [
            node for node in self.critical_nodes if node in final_running_nodes]
        if len(critical_running) == len(self.critical_nodes):
            print("✅ All critical simulation nodes running")
            success = True
        elif len(final_running_nodes) >= 2:  # At least simulator + one other
            print(
                f"✅ Sufficient simulation nodes running ({len(final_running_nodes)}/2+)")
            success = True
        else:
            print(
                f"❌ Insufficient simulation nodes running ({len(final_running_nodes)}/2+)")
            success = False

        if success:
            print("🎉 Argo simulation mode started successfully!")
            print("Simulated sensor data available on:")
            print("  - /pose (compass heading)")
            print("  - /gps_cog, /gps_sog, /gps_velocity (GPS navigation)")
            print("  - /anem_speed_angle_temp (wind data)")
            print("  - /rudder_sail_radio (integrated keyboard control)")
            print("Control commands sent to simulator via /rudder_sail_servo")
            print(
                "Keyboard control: Use arrow keys in curses display to control rudder and sail")
            print("Visualization available via Foxglove Studio at ws://localhost:9090")
            print("\n🔄 Simulation running... Press Ctrl+C to stop and clean up all nodes")

            # Start continuous monitoring with proper cleanup
            try:
                while True:
                    time.sleep(30)  # Check every 30 seconds

                    # Check node status
                    node_status = self._get_node_status()
                    running_nodes = [
                        node for node, status in node_status.items() if "RUNNING" in status]
                    stopped_nodes = [
                        node for node, status in node_status.items() if "STOPPED" in status]

                    if stopped_nodes:
                        print(
                            f"⚠️  {len(stopped_nodes)} simulation nodes stopped: {', '.join(stopped_nodes)}")

                        # Check if critical nodes are still running
                        critical_running = [
                            n for n in self.critical_nodes if n in running_nodes]
                        critical_stopped = [
                            n for n in self.critical_nodes if n in stopped_nodes]

                        if critical_stopped:
                            print(
                                f"❌ CRITICAL SIMULATION NODES STOPPED: {', '.join(critical_stopped)}")
                            print(
                                f"   Simulation will continue with remaining nodes")
                        else:
                            print(
                                f"✅ Critical simulation nodes operational: {', '.join(critical_running)}")

                        # Show system status
                        if len(running_nodes) >= 2:
                            print(
                                f"✅ Simulation operational with {len(running_nodes)}/{len(self.expected_nodes + self.special_nodes)} nodes")
                        else:
                            print(
                                f"⚠️  Low node count: {len(running_nodes)}/{len(self.expected_nodes + self.special_nodes)} nodes running")

            except KeyboardInterrupt:
                print("\n🛑 Stopping simulation and cleaning up all nodes...")
                self.stop()
                print("✅ All simulation nodes terminated")
                return True
            except Exception as e:
                print(f"❌ Error in simulation mode: {e}")
                self.stop()
                return False
        else:
            print("❌ Argo simulation mode failed to start")

        return success

    def status(self) -> None:
        """Show current status of Argo nodes"""
        # Show checking message and clear it
        print("🔍 Checking Argo status...", end='', flush=True)
        time.sleep(0.1)  # Brief pause to show the message
        print("\r" + " " * 50 + "\r", end='', flush=True)  # Clear the line

        print(
            f"🚢 ARGO STATUS - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("=" * 60)

        # Check if argo_power_control.service is running
        power_control_running = False
        try:
            result = subprocess.run(['systemctl', 'is-active', 'argo_power_control.service'],
                                    capture_output=True, text=True, timeout=2)
            power_control_running = result.returncode == 0 and result.stdout.strip() == 'active'
        except Exception:
            power_control_running = False

        if power_control_running:
            print("⚡ POWER CONTROL: 🟢 RUNNING")
        else:
            print("⚡ POWER CONTROL: 🔴 STOPPED")

        # Check if argo-launch.service is running
        service_running = False
        try:
            result = subprocess.run(['systemctl', 'is-active', 'argo-launch.service'],
                                    capture_output=True, text=True, timeout=2)
            service_running = result.returncode == 0 and result.stdout.strip() == 'active'
        except Exception:
            service_running = False

        if service_running:
            print("📋 LAUNCH SERVICE: 🟢 RUNNING")
        else:
            print("📋 LAUNCH SERVICE: 🔴 STOPPED")

        # Get FATAL messages for stopped nodes if service is running
        node_fatal_messages = {}
        if service_running:
            node_fatal_messages = self._get_fatal_messages_for_nodes()

        # Individual node status
        node_status = self._get_node_status()
        running_count = sum(
            1 for status in node_status.values() if "RUNNING" in status)
        total_count = len(node_status)
        print(f"🤖 ROS NODES: [{running_count}/{total_count}]")
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
            # print with no newline so that if there are no errors, the next line is not indented
            # use green good symbol if no errors
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
                                    recent_errors.append(
                                        f"  {matched_node}: {timestamp} - {message}")
                            else:
                                # Fallback if parsing fails
                                recent_errors.append(
                                    f"  {matched_node}: {line}")

                    if recent_errors:
                        # new line before actual errors
                        print("⚠️  Errors found in systemd journal:")
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
                            fatal_node_errors = [
                                e for e in node_errors if 'FATAL' in e]
                            if fatal_node_errors:
                                # Most recent FATAL per node
                                display_errors.append(fatal_node_errors[-1])

                        # Then add other errors if we have space (max 8 total)
                        for node_errors in errors_by_node.values():
                            other_errors = [
                                e for e in node_errors if 'FATAL' not in e]
                            if other_errors and len(display_errors) < 8:
                                # Most recent other error per node
                                display_errors.append(other_errors[-1])

                        # Sort by timestamp (newest first) and limit to 8 errors
                        display_errors = display_errors[-8:]

                        for error in display_errors:
                            print(error)
                    else:
                        print("  No specific node errors found in recent logs")
                else:
                    print(
                        f"🟢  No systemd journal errors found since {self.journal_since}")
            except Exception as e:
                print(f"  Error retrieving messages: {e}")

        # If nothing is running, provide I2C bus health info to help diagnostics
        try:
            launch_stopped = not self._is_launch_running()
            all_nodes_stopped = all(
                "STOPPED" in s for s in node_status.values()) if node_status else True
            if launch_stopped or all_nodes_stopped:
                print("\n🔌 I2C BUS HEALTH (bus 0):")
                self._print_i2c_health(bus=0)
        except Exception as e:
            print(f"⚠️  I2C health check failed: {e}")

        # System info
        try:
            # CPU percentage (this waits 1 second by design)
            cpu_percent = psutil.cpu_percent(interval=1)

            # Memory and disk
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage("/")
            free_disk = disk.free / (1024**3)

            # CPU temperature from thermal logs
            cpu_temp = None
            try:
                # Try to find the most recent thermal log file
                import glob
                thermal_logs = sorted(
                    glob.glob('/var/log.hdd/persistent/thermal-*.log'), reverse=True)
                for thermal_log in thermal_logs:
                    if os.path.exists(thermal_log) and os.path.getsize(thermal_log) > 0:
                        with open(thermal_log, 'r') as f:
                            # Read last line
                            lines = f.readlines()
                            if lines:
                                last_line = lines[-1].strip()
                                # Parse: "2025-10-01 06:30:23: GPU:60°C VE:57°C CPU:58°C DDR:58°C"
                                if 'CPU:' in last_line:
                                    cpu_part = last_line.split(
                                        'CPU:')[1].split()[0]
                                    cpu_temp = cpu_part.replace('°C', '')
                                    break
            except Exception:
                pass

            # Fallback to reading thermal zone directly if log method failed
            if cpu_temp is None:
                try:
                    with open('/sys/class/thermal/thermal_zone2/temp', 'r') as f:
                        temp_millicelsius = int(f.read().strip())
                        cpu_temp = str(temp_millicelsius // 1000)
                except Exception:
                    cpu_temp = None

            # Get node status (needed for battery check)
            node_status = self._get_node_status()

            # Battery and alerts
            battery_summary, critical_alerts = None, None
            if "battery_water.py" in node_status and "RUNNING" in node_status["battery_water.py"]:
                battery_summary, critical_alerts = self._get_battery_water_status_alerts()

            # Build system info line with optional battery info
            system_info = f"📊 SYSTEM: CPU {cpu_percent:.1f}% | Mem. {memory.percent:.1f}% | Free Disk {free_disk:.1f}GB ({disk.percent:.1f}% used) | CPU Temp. {cpu_temp}°C"
            if battery_summary:
                system_info += f" | Batt. {battery_summary}"
            print(system_info)

            # Display critical alerts if any
            if critical_alerts:
                print(f"⚠️  CRITICAL ALERTS: {critical_alerts}")

        except Exception as e:
            print(f"📊 SYSTEM: Unable to get system info - {e}")
            import traceback
            traceback.print_exc()

        # Update timestamp file to prevent quick timer from running on next terminal startup
        # This ensures that manual status checks are treated the same as quick timer checks
        try:
            home_dir = os.path.expanduser('~')
            last_check_file = os.path.join(home_dir, ".argo_last_check")
            current_time = int(time.time())
            with open(last_check_file, 'w') as f:
                f.write(str(current_time))
        except Exception:
            # Silently fail - this is just for preventing redundant quick timer checks
            pass

        print("=" * 60)

    def _start_remote_tunnel(self):
        """Start SSH tunnel for remote simulation"""
        if not REMOTE_CONFIG:
            print("❌ Remote config not found")
            return False

        host = REMOTE_CONFIG['remote']['host']
        user = REMOTE_CONFIG['remote']['user']
        local_port = REMOTE_CONFIG['network']['local_port']
        remote_port = REMOTE_CONFIG['network']['remote_port']

        print(f"🔗 Creating SSH tunnel to {user}@{host}...")
        cmd = [
            'ssh', '-N', '-L', f'{local_port}:localhost:{remote_port}',
            f'{user}@{host}'
        ]
        self.remote_tunnel_proc = subprocess.Popen(cmd)
        time.sleep(2)  # Wait for tunnel to establish

        if self.remote_tunnel_proc.poll() is None:
            print(
                f"✅ SSH tunnel established (PID: {self.remote_tunnel_proc.pid})")
            return True
        else:
            print("❌ SSH tunnel failed to start")
            return False

    def _start_remote_simulator(self):
        """Start the simulator on the remote machine"""
        if not REMOTE_CONFIG:
            print("❌ Remote config not found")
            return False

        host = REMOTE_CONFIG['remote']['host']
        user = REMOTE_CONFIG['remote']['user']
        argo_dir = REMOTE_CONFIG['remote']['argo_dir']
        ros_domain_id = REMOTE_CONFIG['ros2']['domain_id']

        print(f"🚀 Launching remote simulator on {user}@{host}...")
        remote_cmd = (
            f"cd {argo_dir} && "
            f"source /opt/ros/humble/setup.bash && "
            f"export ROS_DOMAIN_ID={ros_domain_id} && "
            f"python3 nodes/argo_unified_simulator_bridge.py --mode local"
        )
        cmd = ['ssh', f'{user}@{host}', remote_cmd]

        self.remote_simulator_proc = subprocess.Popen(cmd)
        if self.remote_simulator_proc.poll() is None:
            print(
                f"✅ Remote simulator launched (PID: {self.remote_simulator_proc.pid})")
            return True
        else:
            print("❌ Failed to launch remote simulator")
            return False

    def _stop_remote_processes(self):
        """Stop remote simulator and SSH tunnel"""
        if self.remote_simulator_proc:
            print("🛑 Stopping remote simulator...")
            # Need to kill the process on the remote machine
            if REMOTE_CONFIG:
                host = REMOTE_CONFIG['remote']['host']
                user = REMOTE_CONFIG['remote']['user']
                kill_cmd = "pkill -f 'argo_unified_simulator_bridge.py'"
                subprocess.run(['ssh', f'{user}@{host}', kill_cmd])
            self.remote_simulator_proc.terminate()
            self.remote_simulator_proc.wait(timeout=5)
            self.remote_simulator_proc = None

        if self.remote_tunnel_proc:
            print("🛑 Stopping SSH tunnel...")
            self.remote_tunnel_proc.terminate()
            self.remote_tunnel_proc.wait(timeout=5)
            self.remote_tunnel_proc = None

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
            raise RuntimeError(
                "i2cdetect not found. Install i2c-tools or ensure it's in PATH.")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(
                f"i2cdetect failed (exit {e.returncode}). stderr: {e.stderr}")
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
                                            message = message.split(
                                                '.')[0].strip()
                                        # Limit message length for display
                                        if len(message) > 50:
                                            message = message[:47] + "..."

                                        # Store the most recent FATAL for this node
                                        fatal_messages[node] = f"FATAL: {message}"
                                elif 'fatal(' in line.lower():
                                    # Handle ROS2 logger format: .fatal("message")
                                    import re
                                    match = re.search(
                                        r'fatal\("([^"]*)"', line, re.IGNORECASE)
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

    def _get_battery_water_status_alerts(self) -> tuple[Optional[str], Optional[str]]:
        """Get battery info and alerts using the battery Trigger service client"""
        try:
            # Use ROS2 service client if available, otherwise fallback to subprocess
            if self.battery_service_client and ROS2_AVAILABLE:
                battery_data = self._call_battery_service_client()
            else:
                battery_data = self._call_battery_service_subprocess()

            if battery_data:
                # Extract battery summary and alerts
                battery_summary = battery_data.get('battery_summary')
                critical_alerts = battery_data.get('critical_alerts')
                return battery_summary, critical_alerts
            else:
                return None, None

        except Exception as e:
            print(f"    Error getting battery and alerts: {e}")
            return None, None

    def _call_battery_service_client(self) -> Optional[Dict[str, Any]]:
        """Call battery service using ROS2 service client"""
        try:
            if not self.battery_service_client.service_is_ready():
                return None

            # Create request
            request = Trigger.Request()

            # Call service with timeout
            future = self.battery_service_client.call_async(request)
            rclpy.spin_until_future_complete(
                self.ros2_node, future, timeout_sec=3.0)

            if future.done():
                response = future.result()
                if response.success:
                    # Parse JSON from response message
                    return json.loads(response.message)
                else:
                    return None
            else:
                return None

        except Exception as e:
            return None

    def _call_battery_service_subprocess(self) -> Optional[Dict[str, Any]]:
        """Fallback: Call battery service using subprocess"""
        try:
            result = subprocess.run([
                'ros2', 'service', 'call', '/battery_status', 'std_srvs/srv/Trigger'
            ], capture_output=True, text=True, timeout=5)

            if result.returncode == 0 and result.stdout.strip():
                return self._parse_trigger_response_subprocess(result.stdout)
            else:
                return None
        except subprocess.TimeoutExpired:
            return None
        except Exception as e:
            return None

    def _parse_trigger_response_subprocess(self, output: str) -> Optional[Dict[str, Any]]:
        """Parse ROS2 Trigger service response from subprocess output"""
        try:
            # Look for the message field in the Trigger response
            lines = output.strip().split('\n')
            message_content = None

            for line in lines:
                line = line.strip()
                if line.startswith('message:'):
                    # Extract the JSON content from the message field
                    message_part = line.split(':', 1)[1].strip()
                    # Remove quotes if present
                    if message_part.startswith('"') and message_part.endswith('"'):
                        message_part = message_part[1:-1]
                    # Unescape newlines and quotes
                    message_part = message_part.replace(
                        '\\n', '\n').replace('\\"', '"')
                    message_content = message_part
                    break

            if message_content:
                # Parse the JSON content
                return json.loads(message_content)
            else:
                return None

        except json.JSONDecodeError:
            return None
        except Exception:
            return None

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
        hex_list = ", ".join(
            [f"0x{a:02x}" for a in detected]) if detected else "<none>"
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
                print(
                    f"  {sensor}: 🟢 present ({', '.join([f'0x{x:02x}' for x in present])})")
            elif present and missing:
                print(
                    f"  {sensor}: 🟡 partially present (found {', '.join([f'0x{x:02x}' for x in present])}; missing {', '.join([f'0x{x:02x}' for x in missing])})"
                )
            else:
                print(
                    f"  {sensor}: 🔴 not present (expected {', '.join([f'0x{x:02x}' for x in addrs])})")


def main():
    parser = argparse.ArgumentParser(description='Argo ROS2 Lifecycle Manager')
    parser.add_argument('command', choices=['run', 'stop', 'restart', 'status', 'simulate_local', 'simulate_remote'],
                        help='Command to execute')
    parser.add_argument('--debug', action='store_true',
                        help='Enable debug output')

    args = parser.parse_args()

    manager = ArgoLifecycleManager()
    if args.debug:
        print("🔧 DEBUG: Debug mode enabled")

    try:
        if args.command == 'run':
            success = manager.continuous()
            sys.exit(0 if success else 1)
        elif args.command == 'stop':
            success = manager.stop()
            sys.exit(0 if success else 1)
        elif args.command == 'restart':
            success = manager.restart()
            sys.exit(0 if success else 1)
        elif args.command == 'status':
            manager.status()
        elif args.command == 'simulate_local':
            success = manager.simulate_local()
            sys.exit(0 if success else 1)
        elif args.command == 'simulate_remote':
            success = manager.simulate_remote()
            sys.exit(0 if success else 1)
    finally:
        # Ensure ROS2 cleanup
        manager._cleanup_ros2()


if __name__ == '__main__':
    main()
