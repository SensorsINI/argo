#!/usr/bin/env python3
"""
Argo Keyboard Control Node
==========================

Standalone ROS2 node for keyboard control of Argo simulator.
Publishes control commands to /rudder_sail_radio topic for simulator control.

This node can be run in a separate terminal while simulation runs via 'asim'.

Features:
- Arrow key control (←→ rudder, ↑↓ sail)
- Visual display of current control positions
- Real-time status updates
- Process-level pause (SIGSTOP/SIGCONT) to freeze simulation and preserve markers
- Clean shutdown with terminal restoration

Usage:
    python3 nodes/argo_keyboard_control.py

Keyboard Controls:
    ←  : Rudder left (decrease)
    →  : Rudder right (increase)
    ↑  : Sail out (increase)
    ↓  : Sail in (decrease)
    c  : Center both controls
    w  : Rotate wind +10°
    e  : Rotate wind -10°
    SPACE : Toggle simulation pause (SIGSTOP/SIGCONT - keeps markers visible!)
    r  : Reset simulation
    q  : Quit

Simulation Pause (SPACE):
    Uses SIGSTOP to literally freeze all simulation processes, keeping them alive
    so visualization markers remain visible in Foxglove/RViz for inspection.
    Press SPACE again to resume with SIGCONT.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, ParameterEvent
import curses
import time
import queue
import signal
import sys
import psutil
import os

class KeyboardControlNode(Node):
    """ROS2 node for keyboard control of Argo simulator."""
    
    def __init__(self):
        super().__init__('argo_keyboard_control')
        
        # Publisher for control commands
        self.pub_rudder_sail_radio = self.create_publisher(Vector3, '/rudder_sail_radio', 10)
        self.pub_simulation_paused = self.create_publisher(Bool, '/simulation_paused', 10)
        self.wind_param_name = 'simulation.wind.wind_direction'
        self.wind_param_target = '/argo_unified_simulator_bridge'
        self.wind_get_client = self.create_client(GetParameters, f'{self.wind_param_target}/get_parameters')
        self.wind_set_client = self.create_client(SetParameters, f'{self.wind_param_target}/set_parameters')
        self.wind_direction_deg = None
        self.create_subscription(ParameterEvent, '/parameter_events', self.parameter_event_callback, 10)
        
        # Service client for simulation reset
        self.reset_service_client = self.create_client(Trigger, '/simulator/reset')
        
        # Control state
        self.rudder_position = 0.0  # -1.0 to +1.0
        self.sail_position = 0.0    # -1.0 to +1.0
        self.step_size = 1.0 / 8.0  # 8 steps for full scale (0.125)
        self.running = True
        self.simulation_paused = False
        self.use_process_pause = True  # Use SIGSTOP/SIGCONT instead of topic
        
        # Keyboard activity tracking - only publish when keys are actively pressed
        self.last_keyboard_activity = 0.0  # Timestamp of last keyboard activity
        self.keyboard_timeout = 0.5  # Stop publishing 0.5 seconds after last key press
        
        # Track simulation process PIDs for pause/resume
        self.simulation_pids = []
        
        # Status from simulator (for display)
        self.simulator_heading = 0.0
        self.simulator_speed = 0.0
        self.simulator_mode = "UNKNOWN"
        
        # Subscribe to status topics for display
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Vector3, '/gps_velocity', self.velocity_callback, 10)
        self.create_subscription(Bool, '/human_controlled', self.control_mode_callback, 10)
        
        # Setup curses
        self.stdscr = curses.initscr()
        self._setup_curses()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGWINCH, self._signal_handler)
        
        # Control loop timer (publishes only when keyboard is active)
        self.control_timer = self.create_timer(0.1, self.publish_control)
        
        # Display update timer
        self.display_timer = self.create_timer(0.2, self.update_display)
        
        # Keyboard input timer
        self.keyboard_timer = self.create_timer(0.05, self.handle_keyboard_input)
        
        self.publish_simulation_paused()
        self._initialize_wind_direction()

        self.get_logger().info('Keyboard control node ready')
        self.get_logger().info('Controls: ←→ Rudder | ↑↓ Sail | C=Center | SPACE=Pause | W/E=Wind ±10° | R=Reset | Q=Quit')
    
    def _setup_curses(self):
        """Setup curses for terminal control."""
        try:
            curses.curs_set(0)  # Hide cursor
            curses.noecho()      # Don't echo keystrokes
            curses.cbreak()      # Immediate key input
            self.stdscr.keypad(True)  # Enable keypad
            self.stdscr.nodelay(True)  # Non-blocking input
            
            # Initialize colors if supported
            if curses.has_colors():
                curses.start_color()
                curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
                curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
                curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)
                curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)
                self.colors_available = True
            else:
                self.colors_available = False
        except curses.error:
            self.colors_available = False
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals."""
        if signum == signal.SIGWINCH:
            # Terminal resize - handled in update_display
            pass
        else:
            self.running = False
            self.cleanup()
            if rclpy.ok():
                rclpy.shutdown()
    
    def cleanup(self):
        """Restore terminal to normal state and resume simulation if paused."""
        try:
            if hasattr(self, 'stdscr') and self.stdscr:
                self.stdscr.keypad(False)
                self.stdscr.nodelay(False)
            
            curses.nocbreak()
            curses.echo()
            curses.curs_set(1)
            curses.endwin()
        except:
            pass
        
        # Ensure simulation resumes when exiting
        if self.simulation_paused:
            try:
                if self.use_process_pause and self.simulation_pids:
                    # Resume frozen processes
                    self._resume_simulation_processes()
                    self.get_logger().info('Resumed simulation processes on exit')
                else:
                    # Resume via topic
                    self.simulation_paused = False
                    self.publish_simulation_paused()
            except Exception as e:
                # Try to resume processes even if logging fails
                if self.simulation_pids:
                    for pid in self.simulation_pids:
                        try:
                            os.kill(pid, signal.SIGCONT)
                        except:
                            pass
    def parameter_event_callback(self, event: ParameterEvent):
        """Handle parameter events to track wind direction updates."""
        if event.node != self.wind_param_target:
            return
        for param in list(event.changed_parameters) + list(event.new_parameters):
            if param.name != self.wind_param_name:
                continue
            if param.value.type == ParameterType.PARAMETER_DOUBLE:
                self.wind_direction_deg = param.value.double_value % 360.0
                self.get_logger().info(f"Wind direction parameter event: {self.wind_direction_deg:.1f}°")
            return

    def _initialize_wind_direction(self):
        """Fetch current wind direction parameter from simulator (best effort)."""
        attempts = 0
        while attempts < 5 and not self.wind_get_client.wait_for_service(timeout_sec=1.0):
            attempts += 1
            self.get_logger().info("Waiting for simulator wind parameter service...")
        if attempts == 5:
            self.get_logger().warn("Could not contact wind parameter service; waiting for parameter events")
            return
        request = GetParameters.Request()
        request.names = [self.wind_param_name]
        future = self.wind_get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            response = future.result()
            if response and response.values:
                value = response.values[0]
                if value.type == ParameterType.PARAMETER_DOUBLE:
                    self.wind_direction_deg = value.double_value % 360.0
                    self.get_logger().info(f"Initial wind direction: {self.wind_direction_deg:.1f}°")
                    return
        self.get_logger().warn("Failed to fetch initial wind direction; waiting for parameter events")

    def pose_callback(self, msg):
        """Receive heading/pose from simulator."""
        heading_math = float(msg.z) % 360.0
        # Convert mathematical (counter-clockwise, 0° = East) heading back to compass convention (clockwise from North)
        self.simulator_heading = (450.0 - heading_math) % 360.0
    
    def velocity_callback(self, msg):
        """Receive velocity from simulator."""
        self.simulator_speed = msg.z  # Speed in knots
    
    def control_mode_callback(self, msg):
        """Receive control mode status."""
        self.simulator_mode = "HUMAN" if msg.data else "ROBOT"
    
    def handle_keyboard_input(self):
        """Process keyboard input."""
        if not self.running:
            return
        
        try:
            key = self.stdscr.getch()
            if key == -1:
                return  # No key pressed
            
            # Handle arrow keys
            if key == curses.KEY_LEFT:
                self.adjust_rudder(-1)  # Rudder left
            elif key == curses.KEY_RIGHT:
                self.adjust_rudder(1)   # Rudder right
            elif key == curses.KEY_UP:
                self.adjust_sail(1)      # Sail out
            elif key == curses.KEY_DOWN:
                self.adjust_sail(-1)     # Sail in
            elif key == ord('c') or key == ord('C'):
                self.center_controls()
            elif key == ord('w') or key == ord('W'):
                self.adjust_wind_direction(10.0)
            elif key == ord('e') or key == ord('E'):
                self.adjust_wind_direction(-10.0)
            elif key == ord(' '):
                self.toggle_simulation_pause()
            elif key == ord('r') or key == ord('R'):
                self.reset_simulation()
            elif key == ord('q') or key == ord('Q'):
                self.running = False
                self.cleanup()
                if rclpy.ok():
                    rclpy.shutdown()
            elif key == 3:  # Ctrl+C
                self.running = False
                self.cleanup()
                if rclpy.ok():
                    rclpy.shutdown()
        except:
            pass
    
    def adjust_rudder(self, direction):
        """Adjust rudder position by one step."""
        self.rudder_position += direction * self.step_size
        self.rudder_position = max(-1.0, min(1.0, self.rudder_position))
        self.last_keyboard_activity = time.time()  # Mark keyboard activity
        # Publish immediately when key is pressed
        self.publish_control()
    
    def adjust_sail(self, direction):
        """Adjust sail position by one step."""
        self.sail_position += direction * self.step_size
        self.sail_position = max(-1.0, min(1.0, self.sail_position))
        self.last_keyboard_activity = time.time()  # Mark keyboard activity
        # Publish immediately when key is pressed
        self.publish_control()
    
    def center_controls(self):
        """Center both rudder and sail."""
        self.rudder_position = 0.0
        self.sail_position = 0.0
        self.last_keyboard_activity = time.time()  # Mark keyboard activity
        # Publish immediately when key is pressed
        self.publish_control()
    
    def _find_simulation_processes(self):
        """Find all simulation-related processes (simulator bridge, visualization, etc.)."""
        sim_processes = []
        my_pid = os.getpid()
        
        # Processes to pause (simulation nodes, but not keyboard control)
        target_processes = [
            'argo_unified_simulator_bridge.py',
            'argo_boat_visualization.py',
            'argo_transform_publisher.py',
            'controller.py',
            # Add other simulation nodes as needed
        ]
        
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    # Skip ourselves
                    if proc.pid == my_pid:
                        continue
                    
                    cmdline = proc.cmdline()
                    if not cmdline:
                        continue
                    
                    # Check if this is one of our target processes
                    for target in target_processes:
                        if any(target in arg for arg in cmdline):
                            sim_processes.append(proc.pid)
                            break
                            
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
        except Exception as e:
            self.get_logger().warn(f"Error finding simulation processes: {e}")
        
        return sim_processes
    
    def _pause_simulation_processes(self):
        """Pause simulation by sending SIGSTOP to all simulation processes."""
        self.simulation_pids = self._find_simulation_processes()
        
        if not self.simulation_pids:
            self.get_logger().warn("No simulation processes found to pause")
            return False
        
        paused_count = 0
        for pid in self.simulation_pids:
            try:
                os.kill(pid, signal.SIGSTOP)
                paused_count += 1
            except (ProcessLookupError, PermissionError) as e:
                self.get_logger().warn(f"Could not pause process {pid}: {e}")
        
        self.get_logger().info(f"Paused {paused_count}/{len(self.simulation_pids)} simulation processes")
        return paused_count > 0
    
    def _resume_simulation_processes(self):
        """Resume simulation by sending SIGCONT to all paused processes."""
        if not self.simulation_pids:
            self.get_logger().warn("No simulation processes to resume")
            return False
        
        resumed_count = 0
        for pid in self.simulation_pids:
            try:
                os.kill(pid, signal.SIGCONT)
                resumed_count += 1
            except (ProcessLookupError, PermissionError) as e:
                self.get_logger().warn(f"Could not resume process {pid}: {e}")
        
        self.get_logger().info(f"Resumed {resumed_count}/{len(self.simulation_pids)} simulation processes")
        self.simulation_pids = []  # Clear list after resume
        return resumed_count > 0
    
    def toggle_simulation_pause(self):
        """Toggle simulation pause state using process signals or topic."""
        self.simulation_paused = not self.simulation_paused
        
        if self.use_process_pause:
            # Use SIGSTOP/SIGCONT to literally freeze the simulation
            if self.simulation_paused:
                success = self._pause_simulation_processes()
                if success:
                    self.get_logger().info('🔴 Simulation FROZEN (SIGSTOP) - markers will persist')
                else:
                    self.get_logger().warn('⚠️  Could not pause simulation processes')
                    self.simulation_paused = False  # Revert if failed
            else:
                success = self._resume_simulation_processes()
                if success:
                    self.get_logger().info('🟢 Simulation RESUMED (SIGCONT)')
                else:
                    self.get_logger().warn('⚠️  Could not resume simulation processes')
        else:
            # Fallback: use topic-based pause (old behavior)
            self.publish_simulation_paused()
            state = "PAUSED" if self.simulation_paused else "RUNNING"
            self.get_logger().info(f'Simulation {state} (topic-based)')
    
    def publish_simulation_paused(self):
        """Publish current simulation pause state."""
        msg = Bool()
        msg.data = self.simulation_paused
        self.pub_simulation_paused.publish(msg)

    def adjust_wind_direction(self, delta_deg: float):
        """Adjust simulator wind direction parameter."""
        if self.wind_direction_deg is None:
            self.get_logger().warn("Wind direction unknown; awaiting parameter event before adjusting")
            return
        new_value = (self.wind_direction_deg + delta_deg) % 360.0

        if not self.wind_set_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Wind direction service unavailable")
            return

        param = Parameter()
        param.name = self.wind_param_name
        value = ParameterValue()
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = new_value
        param.value = value

        request = SetParameters.Request()
        request.parameters.append(param)
        future = self.wind_set_client.call_async(request)
        if rclpy.spin_until_future_complete(self, future, timeout_sec=2.0) is None or not future.done():
            self.get_logger().warn("Failed to set wind direction parameter (timeout)")
            return
        result = future.result()
        if result is None or not result.results or not result.results[0].successful:
            self.get_logger().warn("Failed to set wind direction parameter (service error)")
            return

        self.wind_direction_deg = new_value
        self.get_logger().info(f"Wind direction set to {self.wind_direction_deg:.1f}°")
    
    def reset_simulation(self):
        """Reset simulation to initial state."""
        if not self.reset_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Reset service not available (simulator bridge may not be running)")
            return
        
        request = Trigger.Request()
        future = self.reset_service_client.call_async(request)
        
        # Wait for the response with timeout
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f"✅ {response.message}")
                else:
                    self.get_logger().warn(f"⚠️  Reset service returned: {response.message}")
            except Exception as e:
                self.get_logger().error(f"❌ Error calling reset service: {e}")
        else:
            self.get_logger().warn("⚠️  Reset service call timed out")
    
    def publish_control(self):
        """Publish current control commands to /rudder_sail_radio only when keyboard is active."""
        if not self.running:
            return
        
        current_time = time.time()
        time_since_activity = current_time - self.last_keyboard_activity
        
        # Only publish if keyboard was active recently (within timeout period)
        # This prevents continuous publishing from interfering with controller
        if time_since_activity <= self.keyboard_timeout:
            control_msg = Vector3()
            control_msg.x = self.rudder_position  # Rudder: -1=left, +1=right
            control_msg.y = self.sail_position    # Sail: -1=in, +1=out
            control_msg.z = 0.0                   # Reserved
            
            self.pub_rudder_sail_radio.publish(control_msg)
        # If timeout expired, don't publish (controller takes over)
    
    def create_control_bar(self, value, width=20, control="generic"):
        """Create ASCII bar visualization for control position."""
        normalized = (value + 1.0) / 2.0  # -1..+1 -> 0..1
        filled_length = int(normalized * width)
        bar = "[" + "█" * filled_length + "░" * (width - filled_length) + "]"
        
        if control == "sail":
            if value < -0.1:
                direction = "IN"
            elif value > 0.1:
                direction = "OUT"
            else:
                direction = "CENTER"
        else:
            if value < -0.1:
                direction = "← LEFT"
            elif value > 0.1:
                direction = "RIGHT →"
            else:
                direction = "CENTER"
        
        return f"{bar} {direction}"
    
    def update_display(self):
        """Update the curses display."""
        if not self.running:
            return
        
        try:
            self.stdscr.clear()
            self.stdscr.border()
            
            height, width = self.stdscr.getmaxyx()
            
            # Title
            title = "🚢 ARGO KEYBOARD CONTROL"
            self.stdscr.addstr(1, (width - len(title)) // 2, title)
            
            # Empty line
            self.stdscr.addstr(2, 1, " " * (width - 2))
            
            # Rudder visualization
            rudder_bar = self.create_control_bar(self.rudder_position, 16, control="rudder")
            self.stdscr.addstr(3, 2, f"Rudder: {rudder_bar} ({self.rudder_position:+.3f})")
            
            # Sail visualization
            sail_bar = self.create_control_bar(self.sail_position, 16, control="sail")
            self.stdscr.addstr(4, 2, f"Sail:   {sail_bar} ({self.sail_position:+.3f})")
            
            # Empty line
            self.stdscr.addstr(5, 1, " " * (width - 2))
            
            # Status info
            status_line = f"Mode: {self.simulator_mode} | Heading: {self.simulator_heading:.1f}° | Speed: {self.simulator_speed:.1f}kt"
            if len(status_line) > width - 4:
                status_line = status_line[:width - 7] + "..."
            self.stdscr.addstr(6, 2, status_line)
            
            wind_value = self.wind_direction_deg if self.wind_direction_deg is not None else float('nan')
            wind_line = f"Wind Dir: {wind_value:.1f}° (absolute, compass, from)"
            if len(wind_line) > width - 4:
                wind_line = wind_line[:width - 7] + "..."
            self.stdscr.addstr(7, 2, wind_line)
            
            if self.simulation_paused:
                pause_mode = "SIGSTOP" if self.use_process_pause else "topic"
                pause_line = f"Simulation: FROZEN ({pause_mode}) - markers persist in Foxglove"
            else:
                pause_line = f"Simulation: RUNNING"
            if len(pause_line) > width - 4:
                pause_line = pause_line[:width - 7] + "..."
            self.stdscr.addstr(8, 2, pause_line)
            
            # Controls
            controls_line = "Controls: ←→ Rudder | ↑↓ Sail | C=Center | SPACE=Pause | W/E=Wind ±10° | R=Reset | Q=Quit"
            if len(controls_line) > width - 4:
                controls_line = controls_line[:width - 7] + "..."
            self.stdscr.addstr(9, 2, controls_line)
            
            # Topic info
            topic_line = f"Publishing to: /rudder_sail_radio"
            self.stdscr.addstr(10, 2, topic_line)
            
            self.stdscr.refresh()
        except:
            pass
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        self.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
