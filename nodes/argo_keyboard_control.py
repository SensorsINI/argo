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
from std_msgs.msg import Bool, Float64
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
        
        # Batch successive W/E keypresses and send a single wind update after inactivity
        self.WIND_BATCH_TIMEOUT_S = 2.0  # seconds without new W/E before flush
        
        # Publisher for control commands
        self.pub_rudder_sail_radio = self.create_publisher(Vector3, '/rudder_sail_radio', 10)
        self.pub_simulation_paused = self.create_publisher(Bool, '/simulation_paused', 10)
        # Fast wind-set topic (lower latency than parameter service)
        self.pub_wind_direction_set = self.create_publisher(Float64, '/simulation/wind/wind_direction_set', 10)
        self.wind_param_name = 'simulation.wind.wind_direction'
        self.wind_param_target = '/argo_unified_simulator_bridge'
        self.wind_get_client = self.create_client(GetParameters, f'{self.wind_param_target}/get_parameters')
        self.wind_set_client = self.create_client(SetParameters, f'{self.wind_param_target}/set_parameters')
        self.wind_direction_deg = None
        self.create_subscription(ParameterEvent, '/parameter_events', self.parameter_event_callback, 10)
        # Wind batching state
        self._pending_wind_delta = 0.0
        self._last_wind_keypress_time = 0.0
        
        # Service client for simulation reset
        self.reset_service_client = self.create_client(Trigger, '/simulator/reset')
        
        # Controller switching
        self.controller_param_target = '/controller_node'
        self.controller_get_client = self.create_client(GetParameters, f'{self.controller_param_target}/get_parameters')
        self.controller_set_client = self.create_client(SetParameters, f'{self.controller_param_target}/set_parameters')
        self.default_controller_type = None  # Will be read from current parameter
        self.current_controller_type = None  # Track current controller
        self.is_rth_mode = False  # Track if we're in RTH mode
        
        # Controller pause service client
        from std_srvs.srv import SetBool
        self.controller_pause_client = self.create_client(SetBool, '/controller_node/pause')
        self.controller_paused = False  # Track current pause state
        
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
        
        # Diagnostic message storage (preserve last message)
        self.last_diagnostic_message = ""
        self.last_diagnostic_time = 0.0
        self.diagnostic_messages = []  # Store multiple diagnostic messages (with timestamps)
        self.max_diagnostic_messages = 3  # Keep last 3 messages visible
        
        # Subscribe to status topics for display
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Vector3, '/gps_velocity', self.velocity_callback, 10)
        self.create_subscription(Bool, '/human_controlled', self.control_mode_callback, 10)
        self.create_subscription(Bool, '/controller_pause_state', self.controller_pause_state_callback, 10)
        
        # Log initialization messages BEFORE curses takes over terminal
        self.get_logger().info('Keyboard control node ready')
        self.get_logger().info('Controls: ←→ Rudder | ↑↓ Sail | C=Center | SPACE=Pause | W/E=Wind ±10° | R=Reset | H=Toggle RTH | M=Toggle Manual | ENTER=Refresh | Q=Quit')
        
        # Setup curses (after logging to avoid polluting display)
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
        # Wind flush timer
        self.wind_flush_timer = self.create_timer(0.05, self._flush_wind_adjustment)
        
        self.publish_simulation_paused()
        self._initialize_wind_direction()
        self._initialize_controller_type()
    
    def _setup_curses(self):
        """Setup curses for terminal control."""
        try:
            curses.curs_set(0)  # Hide cursor
            curses.noecho()      # Don't echo keystrokes
            curses.cbreak()      # Immediate key input
            self.stdscr.keypad(True)  # Enable keypad
            self.stdscr.nodelay(True)  # Non-blocking input
            # Optimize refresh to reduce tearing
            self.stdscr.timeout(0)  # Non-blocking for input
            # Use leaveok to optimize cursor movement (reduces flicker)
            try:
                self.stdscr.leaveok(True)  # Don't move cursor after refresh
            except:
                pass  # Some terminals don't support this
            
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
            # Terminal resize - trigger display update
            try:
                # Force display refresh on resize
                self.stdscr.clear()
                self.update_display()
            except:
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
        """Handle parameter events to track wind direction and controller type updates."""
        # Handle wind direction updates
        if event.node == self.wind_param_target:
            for param in list(event.changed_parameters) + list(event.new_parameters):
                if param.name != self.wind_param_name:
                    continue
                if param.value.type == ParameterType.PARAMETER_DOUBLE:
                    self.wind_direction_deg = param.value.double_value % 360.0
                    # Use diagnostic message system instead of logger to avoid polluting curses display
                    self._add_diagnostic_message(f"Wind direction parameter event: {self.wind_direction_deg:.1f}°")
                return
        
        # Handle controller type updates
        if event.node == self.controller_param_target:
            for param in list(event.changed_parameters) + list(event.new_parameters):
                if param.name != 'controller_type':
                    continue
                if param.value.type == ParameterType.PARAMETER_STRING:
                    new_controller = param.value.string_value.strip().lower()
                    self.current_controller_type = new_controller
                    was_rth = self.is_rth_mode
                    self.is_rth_mode = (new_controller == 'return_to_home')
                    # If switching away from RTH to a non-RTH controller, update default
                    if was_rth and not self.is_rth_mode:
                        self.default_controller_type = new_controller
                    # If we don't have a default yet and this is not RTH, save it
                    elif not self.is_rth_mode and not self.default_controller_type:
                        self.default_controller_type = new_controller
                    # Use diagnostic message system instead of logger to avoid polluting curses display
                    self._add_diagnostic_message(f"Controller type parameter event: {new_controller} (RTH mode: {self.is_rth_mode}, default: {self.default_controller_type})")
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
    
    def _initialize_controller_type(self):
        """Fetch current controller type parameter from controller node (best effort)."""
        attempts = 0
        while attempts < 5 and not self.controller_get_client.wait_for_service(timeout_sec=1.0):
            attempts += 1
            self.get_logger().info("Waiting for controller parameter service...")
        if attempts == 5:
            self.get_logger().warn("Could not contact controller parameter service")
            return
        
        request = GetParameters.Request()
        request.names = ['controller_type']
        future = self.controller_get_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            response = future.result()
            if response and response.values:
                value = response.values[0]
                if value.type == ParameterType.PARAMETER_STRING:
                    initial_controller = value.string_value.strip().lower()
                    self.current_controller_type = initial_controller
                    self.is_rth_mode = (initial_controller == 'return_to_home')
                    # Save as default only if it's not RTH (RTH is temporary, default is persistent)
                    if not self.is_rth_mode:
                        self.default_controller_type = initial_controller
                    else:
                        # If starting in RTH mode, we need to fetch the default from argo.yaml
                        # For now, default to 'crosser' (common default)
                        self.default_controller_type = 'crosser'
                    self.get_logger().info(f"Initial controller type: {self.current_controller_type} (default: {self.default_controller_type}, RTH mode: {self.is_rth_mode})")
                    return
        self.get_logger().warn("Failed to fetch initial controller type; defaulting to 'crosser'")
        self.default_controller_type = 'crosser'  # Default from argo.yaml
        self.current_controller_type = self.default_controller_type
        self.is_rth_mode = False

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
    
    def controller_pause_state_callback(self, msg):
        """Receive controller pause state."""
        self.controller_paused = bool(msg.data)
    
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
            elif key == ord('h') or key == ord('H'):
                self.toggle_rth_controller()
            elif key == ord('m') or key == ord('M'):
                self.toggle_controller_pause()
            elif key == ord('\n') or key == ord('\r') or key == curses.KEY_ENTER:
                self.clear_and_refresh_display()
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
        """Accumulate wind direction adjustments; flush later as one publish."""
        if self.wind_direction_deg is None:
            self.get_logger().warn("Wind direction unknown; awaiting parameter event before adjusting")
            return
        self._pending_wind_delta += float(delta_deg)
        self._last_wind_keypress_time = time.time()
        # Update preview immediately for UI
        self.wind_direction_deg = (self.wind_direction_deg + float(delta_deg)) % 360.0
    
    def _flush_wind_adjustment(self):
        """Publish a single wind direction update if inactivity window elapsed."""
        if self._pending_wind_delta == 0.0:
            return
        if (time.time() - self._last_wind_keypress_time) < self.WIND_BATCH_TIMEOUT_S:
            return
        if self.wind_direction_deg is None:
            self._pending_wind_delta = 0.0
            return
        try:
            msg = Float64()
            msg.data = float(self.wind_direction_deg) % 360.0
            self.pub_wind_direction_set.publish(msg)
            self.get_logger().info(f"Wind direction batch set to {msg.data:.1f}° (Δ={self._pending_wind_delta:+.1f}°)")
        finally:
            self._pending_wind_delta = 0.0
    
    def reset_simulation(self):
        """Reset simulation to initial state."""
        if not self.reset_service_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Reset service not available (simulator bridge may not be running)")
            return
        
        request = Trigger.Request()
        future = self.reset_service_client.call_async(request)
        
        # Wait for the response with timeout (increased to allow for reset processing time)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    message = f"✅ {response.message}"
                    self.get_logger().info(message)
                    self._add_diagnostic_message(message)
                else:
                    message = f"⚠️  Reset service returned: {response.message}"
                    self.get_logger().warn(message)
                    self._add_diagnostic_message(message)
            except Exception as e:
                message = f"❌ Error calling reset service: {e}"
                self.get_logger().error(message)
                self._add_diagnostic_message(message)
        else:
            # Check if future completed after timeout (service might have succeeded)
            # Give it a moment to check if it completed
            import time as time_module
            time_module.sleep(0.1)  # Brief pause to check if response arrived
            if future.done():
                try:
                    response = future.result()
                    if response.success:
                        message = f"✅ Reset completed (slow response): {response.message}"
                        self.get_logger().info(message)
                        self._add_diagnostic_message(message)
                    else:
                        message = f"⚠️  Reset service returned: {response.message}"
                        self.get_logger().warn(message)
                        self._add_diagnostic_message(message)
                except:
                    message = "⚠️  Reset service call timed out (check if reset succeeded)"
                    self.get_logger().warn(message)
                    self._add_diagnostic_message(message)
            else:
                message = "⚠️  Reset service call timed out (check if reset succeeded)"
                self.get_logger().warn(message)
                self._add_diagnostic_message(message)
    
    def toggle_rth_controller(self):
        """Toggle between default controller (from argo.yaml) and RTH controller."""
        if not self.controller_set_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Controller parameter service not available")
            return
        
        # Determine target controller
        if self.is_rth_mode:
            # Switch back to default controller
            target_controller = self.default_controller_type if self.default_controller_type else 'crosser'
            self.is_rth_mode = False
        else:
            # Switch to RTH controller
            target_controller = 'return_to_home'
            self.is_rth_mode = True
        
        # Create parameter set request
        request = SetParameters.Request()
        param = Parameter()
        param.name = 'controller_type'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = target_controller
        request.parameters = [param]
        
        # Call service
        future = self.controller_set_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            try:
                response = future.result()
                if response and response.results and response.results[0].successful:
                    self.current_controller_type = target_controller
                    mode = "RTH" if self.is_rth_mode else "DEFAULT"
                    message = f"✅ Controller switched to {target_controller} ({mode} mode)"
                    self.get_logger().info(message)
                    # Store diagnostic message for display (with newline)
                    self._add_diagnostic_message(message)
                else:
                    reason = response.results[0].reason if response and response.results else "Unknown error"
                    message = f"⚠️  Failed to switch controller: {reason}"
                    self.get_logger().warn(message)
                    self._add_diagnostic_message(message)
            except Exception as e:
                message = f"❌ Error switching controller: {e}"
                self.get_logger().error(message)
                self._add_diagnostic_message(message)
        else:
            message = "⚠️  Controller switch service call timed out"
            self.get_logger().warn(message)
            self._add_diagnostic_message(message)
    
    def toggle_controller_pause(self):
        """Toggle controller pause state (manual/human control)."""
        if not self.controller_pause_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Controller pause service not available")
            return
        
        # Toggle pause state
        new_pause_state = not self.controller_paused
        
        # Create request
        request = SetBool.Request()
        request.data = new_pause_state
        
        # Call service
        future = self.controller_pause_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    self.controller_paused = new_pause_state
                    mode = "PAUSED (Manual)" if new_pause_state else "UNPAUSED (Autonomous)"
                    message = f"✅ Controller {mode}"
                    self.get_logger().info(message)
                    self._add_diagnostic_message(message)
                else:
                    message = f"⚠️  Failed to toggle controller pause: {response.message}"
                    self.get_logger().warn(message)
                    self._add_diagnostic_message(message)
            except Exception as e:
                message = f"❌ Error toggling controller pause: {e}"
                self.get_logger().error(message)
                self._add_diagnostic_message(message)
        else:
            message = "⚠️  Controller pause service call timed out"
            self.get_logger().warn(message)
            self._add_diagnostic_message(message)
    
    def clear_and_refresh_display(self):
        """Clear and refresh the display to fix corruption from logging messages."""
        try:
            # Force a complete screen clear and refresh
            self.stdscr.clear()
            # Reset terminal size tracking to force full redraw
            if hasattr(self, '_last_term_size'):
                del self._last_term_size
            # Immediately update display
            self.update_display()
            self._add_diagnostic_message("Display cleared and refreshed")
        except Exception as e:
            # If clearing fails, at least try to refresh
            try:
                self.stdscr.refresh()
            except:
                pass
    
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
    
    def _add_diagnostic_message(self, message):
        """Add a diagnostic message to the display queue."""
        # Strip any trailing newlines to prevent display issues
        message = message.rstrip('\n\r')
        current_time = time.time()
        self.diagnostic_messages.append((message, current_time))
        # Keep only recent messages
        if len(self.diagnostic_messages) > self.max_diagnostic_messages * 2:
            # Keep more in memory, but only display last N
            self.diagnostic_messages = self.diagnostic_messages[-self.max_diagnostic_messages * 2:]
        # Also update legacy fields for compatibility
        self.last_diagnostic_message = message
        self.last_diagnostic_time = current_time
    
    def _split_message(self, message, max_width):
        """Split a message into multiple lines that fit within max_width."""
        if len(message) <= max_width:
            return [message]
        
        lines = []
        words = message.split()
        current_line = ""
        
        for word in words:
            # Check if adding this word would exceed width
            test_line = current_line + (" " if current_line else "") + word
            if len(test_line) <= max_width:
                current_line = test_line
            else:
                # Current line is full, start new line
                if current_line:
                    lines.append(current_line)
                # If single word is too long, truncate it
                if len(word) > max_width:
                    current_line = word[:max_width-3] + "..."
                else:
                    current_line = word
        
        if current_line:
            lines.append(current_line)
        
        return lines if lines else [message[:max_width-3] + "..."]
    
    def get_terminal_size(self):
        """Get terminal size, handling resizing dynamically."""
        try:
            import shutil
            return shutil.get_terminal_size()
        except:
            # Fallback: try to get from curses
            try:
                height, width = self.stdscr.getmaxyx()
                return (width, height)
            except:
                return (80, 24)
    
    def update_display(self):
        """Update the curses display."""
        if not self.running:
            return
        
        try:
            # Get current terminal size (handles dynamic resizing)
            term_width, term_height = self.get_terminal_size()
            
            # Use erase() instead of clear() to reduce tearing (faster, less flicker)
            # Only clear if terminal size changed
            if not hasattr(self, '_last_term_size') or self._last_term_size != (term_width, term_height):
                self.stdscr.clear()
                self._last_term_size = (term_width, term_height)
            else:
                self.stdscr.erase()  # Faster than clear(), reduces tearing
            
            self.stdscr.border()
            
            # Use actual terminal dimensions
            height, width = term_height, term_width
            
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
            
            # Status info - split into multiple lines to prevent clipping
            line_num = 6
            mode_line = f"Mode: {self.simulator_mode}"
            heading_line = f"Heading: {self.simulator_heading:.1f}°"
            speed_line = f"Speed: {self.simulator_speed:.1f}kt"
            # Fit on one line if possible, otherwise split
            status_combined = f"{mode_line} | {heading_line} | {speed_line}"
            if len(status_combined) <= width - 4:
                self.stdscr.addstr(line_num, 2, status_combined)
                line_num += 1
            else:
                # Split into multiple lines
                self.stdscr.addstr(line_num, 2, mode_line)
                line_num += 1
                self.stdscr.addstr(line_num, 2, f"{heading_line} | {speed_line}")
                line_num += 1
            
            wind_value = self.wind_direction_deg if self.wind_direction_deg is not None else float('nan')
            wind_line = f"Wind Dir: {wind_value:.1f}° (absolute, compass, from)"
            if len(wind_line) > width - 4:
                # Split wind line
                wind_line1 = f"Wind Dir: {wind_value:.1f}°"
                wind_line2 = "(absolute, compass, from)"
                self.stdscr.addstr(line_num, 2, wind_line1)
                line_num += 1
                if len(wind_line2) <= width - 4:
                    self.stdscr.addstr(line_num, 2, wind_line2)
                    line_num += 1
            else:
                self.stdscr.addstr(line_num, 2, wind_line)
                line_num += 1
            
            # Simulation status
            if self.simulation_paused:
                pause_mode = "SIGSTOP" if self.use_process_pause else "topic"
                pause_line1 = f"Simulation: FROZEN ({pause_mode})"
                pause_line2 = "markers persist in Foxglove"
                self.stdscr.addstr(line_num, 2, pause_line1)
                line_num += 1
                if len(pause_line2) <= width - 4:
                    self.stdscr.addstr(line_num, 2, pause_line2)
                    line_num += 1
            else:
                pause_line = f"Simulation: RUNNING"
                self.stdscr.addstr(line_num, 2, pause_line)
                line_num += 1
            
            # Controller status
            controller_name = self.current_controller_type.upper() if self.current_controller_type else 'UNKNOWN'
            pause_status = " [PAUSED]" if self.controller_paused else ""
            controller_status = f"Controller: {controller_name}{pause_status}"
            if self.is_rth_mode:
                controller_status += " (RTH MODE)"
            else:
                controller_status += " (DEFAULT)"
            if len(controller_status) > width - 4:
                # Split controller status
                self.stdscr.addstr(line_num, 2, f"Controller: {controller_name}")
                line_num += 1
                mode_text = "(RTH MODE)" if self.is_rth_mode else "(DEFAULT)"
                self.stdscr.addstr(line_num, 2, mode_text)
                line_num += 1
            else:
                self.stdscr.addstr(line_num, 2, controller_status)
                line_num += 1
            
            # Controls - split into multiple lines
            line_num += 1  # Empty line before controls
            self.stdscr.addstr(line_num, 2, "Controls:")
            line_num += 1
            controls_line1 = "  ←→ Rudder | ↑↓ Sail | C=Center"
            controls_line2 = "  SPACE=Sim Pause | W/E=Wind ±10°"
            controls_line3 = "  R=Reset | H=Toggle RTH | M=Toggle Manual"
            controls_line4 = "  ENTER=Refresh | Q=Quit"
            self.stdscr.addstr(line_num, 2, controls_line1[:width-4])
            line_num += 1
            self.stdscr.addstr(line_num, 2, controls_line2[:width-4])
            line_num += 1
            self.stdscr.addstr(line_num, 2, controls_line3[:width-4])
            line_num += 1
            self.stdscr.addstr(line_num, 2, controls_line4[:width-4])
            line_num += 1
            
            # Topic info
            topic_line = f"Publishing to: /rudder_sail_radio"
            self.stdscr.addstr(line_num, 2, topic_line[:width-4])
            line_num += 1
            
            # Diagnostic messages (preserve last few messages with newlines)
            current_time = time.time()
            # Clean up old messages (older than 10 seconds)
            self.diagnostic_messages = [
                (msg, msg_time) for msg, msg_time in self.diagnostic_messages
                if (current_time - msg_time) < 10.0
            ]
            
            # Display diagnostic messages below main output (always reserve space)
            # Clear any old message lines first to prevent overwriting
            messages_start_line = line_num + 1
            for clear_line in range(messages_start_line, height - 1):
                try:
                    # Clear line to prevent old message remnants
                    self.stdscr.addstr(clear_line, 2, " " * (width - 4))
                except:
                    pass
            
            # Display diagnostic messages at bottom (after clearing old content)
            if self.diagnostic_messages and height > 15:
                # Start diagnostics a few lines from bottom to ensure visibility
                diag_start_line = max(line_num + 1, height - 8)  # Reserve bottom 8 lines
                diag_line_num = diag_start_line
                
                # Add separator if we have messages
                if diag_line_num < height - 2:
                    try:
                        self.stdscr.addstr(diag_line_num, 2, "─" * min(40, width - 4))
                        diag_line_num += 1
                    except:
                        pass
                
                for msg, msg_time in self.diagnostic_messages[-self.max_diagnostic_messages:]:
                    # Split long messages across multiple lines
                    msg_lines = self._split_message(msg, width - 4)
                    for msg_line in msg_lines:
                        if diag_line_num < height - 2:  # Leave room for border
                            try:
                                self.stdscr.addstr(diag_line_num, 2, msg_line)
                                diag_line_num += 1
                            except:
                                break
                        else:
                            break
                    if diag_line_num >= height - 2:
                        break
            
            self.stdscr.refresh()
        except curses.error as e:
            # Handle terminal resize gracefully
            if "addstr" in str(e).lower() or "waddstr" in str(e).lower():
                # Terminal was resized - try to recover
                try:
                    self.stdscr.clear()
                    self.stdscr.refresh()
                except:
                    pass
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
