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
    t  : Select controller (popup)
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
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float64, String
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
import math

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
        # Simulator publishes true wind here; parameters alone often never update this node’s UI.
        self._sim_true_wind_deg = None
        self.create_subscription(Float64, '/simulator/true_wind_direction', self._true_wind_direction_callback, 10)
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
        
        self._last_non_human_controller = None  # For M: toggle human vs last autonomous type
        self._controller_runtime_name = ''

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
        self.create_subscription(String, '/controller_state', self._controller_state_name_callback, 10)
        
        # Log and block on ROS/sim until ready while TTY is still cooked. Curses (cbreak/noecho)
        # breaks Rcutils line-oriented logging; keep fullscreen mode only after this phase.
        self.get_logger().info('Keyboard control node ready')
        self.get_logger().info('Controls: ←→ Rudder | ↑↓ Sail | C=Center | T=Select Controller | SPACE=Sim Pause | W/E=Wind ±10° | R=Reset | H=Toggle RTH | M=Human/Auton | Q=Quit Sim | X=Quit Control | ENTER=Refresh')
        
        self._curses_active = False
        self._curses_ever_initialized = False
        self.stdscr = None
        
        # Setup signal handlers before potentially long init waits
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
        
        self.get_logger().info('Simulator/controller handshake done; starting fullscreen UI')
        self.stdscr = curses.initscr()
        self._setup_curses()
        self._curses_ever_initialized = True
        self._curses_active = True
    
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
                if getattr(self, "_curses_active", False) and getattr(self, "stdscr", None):
                    self.stdscr.clear()
                    self.update_display()
            except Exception:
                pass
        else:
            self.running = False
            self.cleanup()
            if rclpy.ok():
                rclpy.shutdown()
    
    def cleanup(self):
        """Restore terminal to normal state and resume simulation if paused."""
        self._curses_active = False
        if getattr(self, "_curses_ever_initialized", False):
            try:
                if getattr(self, "stdscr", None):
                    try:
                        self.stdscr.keypad(False)
                        self.stdscr.nodelay(False)
                    except Exception:
                        pass
                curses.nocbreak()
                curses.echo()
                curses.curs_set(1)
                curses.endwin()
            except Exception:
                pass
            self._curses_ever_initialized = False
        self.stdscr = None
        
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

    def _notify(self, message: str, level: str = "info"):
        """Show feedback at bottom while curses is active; otherwise log normally."""
        message = (message or "").rstrip("\n\r")
        if getattr(self, "_curses_active", False):
            self._add_diagnostic_message(message)
            return
        try:
            if level == "warn":
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)
            else:
                self.get_logger().info(message)
        except Exception:
            pass
    def parameter_event_callback(self, event: ParameterEvent):
        """Handle parameter events to track wind direction and controller type updates."""
        def _norm_node_name(name: str) -> str:
            return (name or "").strip().strip('/')

        event_node_norm = _norm_node_name(getattr(event, 'node', ''))
        wind_target_norm = _norm_node_name(self.wind_param_target)
        controller_target_norm = _norm_node_name(self.controller_param_target)

        # Handle wind direction updates.
        # Some ParameterEvent publishers don't match the exact node string we expect (leading '/', namespace),
        # so we key primarily on parameter name and secondarily on node match.
        for param in list(event.changed_parameters) + list(event.new_parameters):
            if param.name == self.wind_param_name and param.value.type == ParameterType.PARAMETER_DOUBLE:
                self.wind_direction_deg = param.value.double_value % 360.0
                return
        if wind_target_norm and event_node_norm and event_node_norm.endswith(wind_target_norm):
            for param in list(event.changed_parameters) + list(event.new_parameters):
                if param.name != self.wind_param_name:
                    continue
                if param.value.type == ParameterType.PARAMETER_DOUBLE:
                    self.wind_direction_deg = param.value.double_value % 360.0
                return
        
        # Handle controller type updates
        for param in list(event.changed_parameters) + list(event.new_parameters):
            if param.name != 'controller_type':
                continue
            if param.value.type == ParameterType.PARAMETER_STRING:
                new_controller = param.value.string_value.strip().lower()
                if controller_target_norm and event_node_norm and not event_node_norm.endswith(controller_target_norm):
                    # Ignore controller_type updates from other nodes (rare, but possible)
                    continue
                self.current_controller_type = new_controller
                if new_controller != 'human':
                    self._last_non_human_controller = new_controller
                was_rth = self.is_rth_mode
                self.is_rth_mode = (new_controller == 'return_to_home')
                # If switching away from RTH to a non-RTH controller, update default
                if was_rth and not self.is_rth_mode:
                    self.default_controller_type = new_controller
                # If we don't have a default yet and this is not RTH, save it
                elif not self.is_rth_mode and not self.default_controller_type:
                    self.default_controller_type = new_controller
                self._add_diagnostic_message(
                    f"Controller type parameter event: {new_controller} (RTH mode: {self.is_rth_mode}, default: {self.default_controller_type})"
                )
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
        
        # Wait for result with manual timeout
        start_time = time.time()
        timeout = 2.0
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.05)
        
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
        
        # Wait for result with manual timeout
        start_time = time.time()
        timeout = 2.0
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.05)
        
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
                    if initial_controller != 'human':
                        self._last_non_human_controller = initial_controller
                    return
        self.get_logger().warn("Failed to fetch initial controller type; defaulting to 'crosser'")
        self.default_controller_type = 'crosser'  # Default from argo.yaml
        self.current_controller_type = self.default_controller_type
        self.is_rth_mode = False
    
    def _controller_state_name_callback(self, msg: String):
        """Runtime controller class name (e.g. HumanController) for status display."""
        self._controller_runtime_name = (msg.data or "").strip()

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
    
    def _true_wind_direction_callback(self, msg: Float64):
        """True wind direction from simulator (degrees, compass from N)."""
        try:
            v = float(msg.data)
            if math.isnan(v):
                return
            self._sim_true_wind_deg = v % 360.0
        except (TypeError, ValueError):
            pass

    def _effective_wind_deg(self):
        """Wind for display / W/E: prefer parameter-backed value, else sim topic."""
        w = self.wind_direction_deg
        if w is not None and not (isinstance(w, float) and math.isnan(w)):
            return float(w) % 360.0
        if self._sim_true_wind_deg is not None:
            return self._sim_true_wind_deg
        return None

    @staticmethod
    def _trunc_to_width(text: str, max_cells: int) -> str:
        if max_cells < 1:
            return ""
        if len(text) <= max_cells:
            return text
        if max_cells <= 3:
            return text[:max_cells]
        return text[: max_cells - 3] + "..."

    def handle_keyboard_input(self):
        """Process keyboard input."""
        if not self.running:
            return
        if not getattr(self, "_curses_active", False) or not getattr(self, "stdscr", None):
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
            elif key == ord('t') or key == ord('T'):
                self.select_controller_popup()
            elif key == ord('m') or key == ord('M'):
                self.toggle_controller_human_mode()
            elif key == ord('q') or key == ord('Q'):
                self.quit_simulation()
            elif key == ord('\n') or key == ord('\r') or key == curses.KEY_ENTER:
                self.clear_and_refresh_display()
            elif key == ord('x') or key == ord('X'):
                # Quit keyboard control (exit this node)
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
            'sailing_area_publisher.py',
            # ros2 run foxglove_bridge foxglove_bridge (no .py in argv)
            'foxglove_bridge',
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
            self._notify(f"⚠️  Error finding simulation processes: {e}", level="warn")
        
        return sim_processes
    
    def _pause_simulation_processes(self):
        """Pause simulation by sending SIGSTOP to all simulation processes."""
        self.simulation_pids = self._find_simulation_processes()
        
        if not self.simulation_pids:
            self._notify("⚠️  No simulation processes found to pause", level="warn")
            return False
        
        paused_count = 0
        for pid in self.simulation_pids:
            try:
                os.kill(pid, signal.SIGSTOP)
                paused_count += 1
            except (ProcessLookupError, PermissionError) as e:
                self._notify(f"⚠️  Could not pause process {pid}: {e}", level="warn")
        
        self._notify(f"Paused {paused_count}/{len(self.simulation_pids)} simulation processes")
        return paused_count > 0
    
    def _resume_simulation_processes(self):
        """Resume simulation by sending SIGCONT to all paused processes."""
        if not self.simulation_pids:
            self._notify("⚠️  No simulation processes to resume", level="warn")
            return False
        
        resumed_count = 0
        for pid in self.simulation_pids:
            try:
                os.kill(pid, signal.SIGCONT)
                resumed_count += 1
            except (ProcessLookupError, PermissionError) as e:
                self._notify(f"⚠️  Could not resume process {pid}: {e}", level="warn")
        
        self._notify(f"Resumed {resumed_count}/{len(self.simulation_pids)} simulation processes")
        self.simulation_pids = []  # Clear list after resume
        return resumed_count > 0
    
    def _quit_simulation_processes(self):
        """Quit simulation by sending SIGTERM to all simulation processes."""
        sim_pids = self._find_simulation_processes()
        
        if not sim_pids:
            self._notify("⚠️  No simulation processes found to quit", level="warn")
            return False
        
        terminated_count = 0
        for pid in sim_pids:
            try:
                # If SPACE froze the sim (SIGSTOP), SIGTERM is deferred until the process
                # receives SIGCONT. Resume first so quit always takes effect.
                try:
                    os.kill(pid, signal.SIGCONT)
                except (ProcessLookupError, PermissionError):
                    pass
                os.kill(pid, signal.SIGTERM)
                terminated_count += 1
            except (ProcessLookupError, PermissionError) as e:
                self._notify(f"⚠️  Could not terminate process {pid}: {e}", level="warn")
        
        self._notify(f"✅ Quit simulation: SIGTERM sent to {terminated_count} processes")
        return terminated_count > 0
    
    def toggle_simulation_pause(self):
        """Toggle simulation pause state using process signals or topic."""
        self.simulation_paused = not self.simulation_paused
        
        if self.use_process_pause:
            # Use SIGSTOP/SIGCONT to literally freeze the simulation
            if self.simulation_paused:
                success = self._pause_simulation_processes()
                if success:
                    self._notify('🔴 Simulation FROZEN (SIGSTOP) - markers will persist')
                else:
                    self._notify('⚠️  Could not pause simulation processes', level="warn")
                    self.simulation_paused = False  # Revert if failed
            else:
                success = self._resume_simulation_processes()
                if success:
                    self._notify('🟢 Simulation RESUMED (SIGCONT)')
                else:
                    self._notify('⚠️  Could not resume simulation processes', level="warn")
        else:
            # Fallback: use topic-based pause (old behavior)
            self.publish_simulation_paused()
            state = "PAUSED" if self.simulation_paused else "RUNNING"
            self._notify(f'Simulation {state} (topic-based)')
    
    def publish_simulation_paused(self):
        """Publish current simulation pause state."""
        msg = Bool()
        msg.data = self.simulation_paused
        self.pub_simulation_paused.publish(msg)

    def adjust_wind_direction(self, delta_deg: float):
        """Accumulate wind direction adjustments; flush later as one publish."""
        if self.wind_direction_deg is None:
            if self._sim_true_wind_deg is not None:
                self.wind_direction_deg = self._sim_true_wind_deg % 360.0
            else:
                self._notify("⚠️  Wind direction unknown; awaiting sim or parameter before adjusting", level="warn")
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
            self._notify(f"✅ Wind direction set to {msg.data:.1f}° (Δ={self._pending_wind_delta:+.1f}°)")
        finally:
            self._pending_wind_delta = 0.0
    
    def reset_simulation(self):
        """Reset simulation to initial state (fire-and-forget)."""
        # Fire-and-forget: just send the request, don't wait for response
        request = Trigger.Request()
        try:
            self.reset_service_client.call_async(request)
            message = "✅ Reset request sent"
            self._notify(message)
        except Exception as e:
            message = f"⚠️  Error sending reset request: {e}"
            self._notify(message, level="warn")
    
    def toggle_rth_controller(self):
        """Toggle between default controller (from argo.yaml) and RTH controller."""
        if not self.controller_set_client.wait_for_service(timeout_sec=2.0):
            self._notify("⚠️  Controller parameter service not available", level="warn")
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
        
        # Wait for result with manual timeout (avoid spin_until_future_complete in timer callback)
        start_time = time.time()
        timeout = 3.0
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.05)  # Small sleep to avoid busy-waiting
        
        if future.done():
            try:
                response = future.result()
                if response and response.results and response.results[0].successful:
                    self.current_controller_type = target_controller
                    mode = "RTH" if self.is_rth_mode else "DEFAULT"
                    message = f"✅ Controller switched to {target_controller} ({mode} mode)"
                    self._notify(message)
                else:
                    reason = response.results[0].reason if response and response.results else "Unknown error"
                    message = f"⚠️  Failed to switch controller: {reason}"
                    self._notify(message, level="warn")
            except Exception as e:
                message = f"❌ Error switching controller: {e}"
                self._notify(message, level="error")
        else:
            message = "⚠️  Controller switch service call timed out"
            self._notify(message, level="warn")

    def _available_controller_types(self):
        """Return deduplicated known controller types, sorted alphabetically (stable for 1–9 keys)."""
        names = set()
        for raw in (self.default_controller_type, self.current_controller_type):
            t = (raw or "").strip().lower()
            if t:
                names.add(t)
        for raw in self._controller_known_types:
            t = (raw or "").strip().lower()
            if t:
                names.add(t)
        # If we ended up with only RTH (shouldn't happen), add a safe fallback
        if names == {'return_to_home'}:
            names.add('crosser')
        return sorted(names, key=str.lower)

    def _set_controller_type(self, target_controller: str):
        """Set controller_type parameter on controller node (best effort)."""
        target_controller = (target_controller or "").strip().lower()
        if not target_controller:
            return
        if not self.controller_set_client.wait_for_service(timeout_sec=2.0):
            message = "⚠️  Controller parameter service not available"
            self._notify(message, level="warn")
            return

        request = SetParameters.Request()
        param = Parameter()
        param.name = 'controller_type'
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = target_controller
        request.parameters = [param]

        future = self.controller_set_client.call_async(request)

        start_time = time.time()
        timeout = 3.0
        while not future.done() and (time.time() - start_time) < timeout:
            time.sleep(0.05)

        if not future.done():
            message = "⚠️  Controller switch service call timed out"
            self._notify(message, level="warn")
            return

        try:
            response = future.result()
            if response and response.results and response.results[0].successful:
                self.current_controller_type = target_controller
                self.is_rth_mode = (target_controller == 'return_to_home')
                if not self.is_rth_mode:
                    self.default_controller_type = target_controller
                message = f"✅ Controller switched to {target_controller}"
                self._notify(message)
            else:
                reason = response.results[0].reason if response and response.results else "Unknown error"
                message = f"⚠️  Failed to switch controller: {reason}"
                self._notify(message, level="warn")
        except Exception as e:
            message = f"❌ Error switching controller: {e}"
            self._notify(message, level="error")

    def select_controller_popup(self):
        """Open a popup to select controller type via numeric keys."""
        if self._controller_popup_open:
            return
        if not getattr(self, "_curses_active", False) or not getattr(self, "stdscr", None):
            return
        self._controller_popup_open = True

        try:
            options = self._available_controller_types()
            if not options:
                self._notify("⚠️  No controllers available", level="warn")
                return

            term_width, term_height = self.get_terminal_size()
            height, width = term_height, term_width

            # Size popup to content but cap within terminal
            popup_h = min(max(8, len(options) + 6), max(8, height - 4))
            popup_w = min(max(44, max(len(o) for o in options) + 16), max(44, width - 4))

            top = max(1, (height - popup_h) // 2)
            left = max(1, (width - popup_w) // 2)

            win = curses.newwin(popup_h, popup_w, top, left)
            win.keypad(True)

            # Temporarily make input blocking so selection feels snappy
            prev_nodelay = self.stdscr.nodelay(False)
            try:
                self.stdscr.nodelay(False)
            except Exception:
                prev_nodelay = None

            selected = None
            while selected is None and self.running:
                win.erase()
                win.border()

                title = "Select controller (press 1-9, ESC to cancel)"
                if len(title) > popup_w - 4:
                    title = "Select controller"
                win.addstr(1, 2, title[: popup_w - 4])

                current = (self.current_controller_type or "unknown").strip().lower()
                default = (self.default_controller_type or "unknown").strip().lower()
                meta = f"Current: {current} | Default: {default}"
                win.addstr(2, 2, meta[: popup_w - 4])

                win.addstr(3, 2, "─" * min(popup_w - 4, 40))

                max_items = min(9, popup_h - 6)
                visible = options[:max_items]
                for idx, opt in enumerate(visible, start=1):
                    marker = "→" if opt == current else " "
                    line = f"{marker} {idx}) {opt}"
                    win.addstr(4 + (idx - 1), 2, line[: popup_w - 4])

                if len(options) > len(visible):
                    more = f"... ({len(options) - len(visible)} more)"
                    win.addstr(popup_h - 2, 2, more[: popup_w - 4])

                win.refresh()

                ch = win.getch()
                if ch in (27, ord('q'), ord('Q')):  # ESC or q
                    break
                if ord('1') <= ch <= ord('9'):
                    idx = ch - ord('1')
                    if idx < len(visible):
                        selected = visible[idx]
                        break

            if selected:
                self._set_controller_type(selected)
        except Exception as e:
            self._notify(f"❌ Controller popup error: {e}", level="error")
        finally:
            try:
                # Restore nodelay mode for normal operation
                self.stdscr.nodelay(True)
            except Exception:
                pass
            self._controller_popup_open = False
    
    def toggle_controller_human_mode(self):
        """Toggle controller_type between human and last non-human type (replaces pause)."""
        cur = (self.current_controller_type or '').strip().lower()
        if cur == 'human':
            restore = self._last_non_human_controller or self.default_controller_type or 'crosser'
            if restore == 'human':
                restore = 'crosser'
            self._set_controller_type(restore)
        else:
            self._last_non_human_controller = cur
            self._set_controller_type('human')
    
    def quit_simulation(self):
        """Quit simulation by sending SIGTERM to all simulation processes."""
        success = self._quit_simulation_processes()
        if not success:
            message = "⚠️  No simulation processes found to quit"
            self._notify(message, level="warn")
        else:
            # Clear pause bookkeeping so UI matches (SIGSTOP targets may overlap sim_pids)
            self.simulation_pids = []
            self.simulation_paused = False
    
    def clear_and_refresh_display(self):
        """Clear and refresh the display to fix corruption from logging messages."""
        if not getattr(self, "_curses_active", False) or not getattr(self, "stdscr", None):
            return
        try:
            # Force a complete screen clear and refresh
            self.stdscr.clear()
            # Reset terminal size tracking to force full redraw
            if hasattr(self, '_last_term_size'):
                del self._last_term_size
            # Immediately update display
            self.update_display()
            self._notify("Display cleared and refreshed")
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
        """Get terminal size. Prefer curses window size so layout matches the actual pad."""
        try:
            if getattr(self, "_curses_active", False) and getattr(self, "stdscr", None):
                rows, cols = self.stdscr.getmaxyx()
                return (cols, rows)
        except Exception:
            pass
        try:
            import shutil
            return shutil.get_terminal_size()
        except Exception:
            return (80, 24)
    
    def update_display(self):
        """Update the curses display."""
        if not self.running:
            return
        if not getattr(self, "_curses_active", False) or not getattr(self, "stdscr", None):
            return
        
        try:
            # Match curses pad size (shutil can disagree and cause overflow/garbled rows).
            term_width, term_height = self.get_terminal_size()
            
            if not hasattr(self, '_last_term_size') or self._last_term_size != (term_width, term_height):
                self.stdscr.clear()
                self._last_term_size = (term_width, term_height)
            else:
                self.stdscr.erase()
            
            self.stdscr.border()
            
            height, width = term_height, term_width
            max_text = max(0, width - 4)
            last_content_row = height - 2
            
            # Title: ASCII only — wide glyphs (emoji/arrows) break len() vs terminal columns.
            title = "ARGO KEYBOARD CONTROL"
            title_x = max(1, (width - len(title)) // 2)
            if title_x + len(title) > width - 1:
                title = self._trunc_to_width(title, max(1, width - 2))
                title_x = 1
            try:
                self.stdscr.addstr(1, title_x, title)
            except curses.error:
                pass
            
            try:
                self.stdscr.addstr(2, 1, " " * (width - 2))
            except curses.error:
                pass
            
            rudder_bar = self.create_control_bar(self.rudder_position, 16, control="rudder")
            try:
                self.stdscr.addstr(3, 2, self._trunc_to_width(
                    f"Rudder: {rudder_bar} ({self.rudder_position:+.3f})", max_text))
            except curses.error:
                pass
            
            sail_bar = self.create_control_bar(self.sail_position, 16, control="sail")
            try:
                self.stdscr.addstr(4, 2, self._trunc_to_width(
                    f"Sail:   {sail_bar} ({self.sail_position:+.3f})", max_text))
            except curses.error:
                pass
            
            try:
                self.stdscr.addstr(5, 1, " " * (width - 2))
            except curses.error:
                pass
            
            line_num = 6

            def put_row(text: str) -> None:
                nonlocal line_num
                if line_num > last_content_row:
                    return
                s = self._trunc_to_width(text, max_text)
                try:
                    self.stdscr.addstr(line_num, 2, s)
                except curses.error:
                    pass
                line_num += 1
            
            mode_line = f"Mode: {self.simulator_mode}"
            heading_line = f"Heading: {self.simulator_heading:.1f}°"
            speed_line = f"Speed: {self.simulator_speed:.1f}kt"
            status_combined = f"{mode_line} | {heading_line} | {speed_line}"
            if len(status_combined) <= max_text:
                put_row(status_combined)
            else:
                put_row(mode_line)
                put_row(self._trunc_to_width(f"{heading_line} | {speed_line}", max_text))
            
            eff_wind = self._effective_wind_deg()
            # One line only — avoids duplicate "(absolute...)" wrap and stray columns.
            if eff_wind is None:
                put_row(self._trunc_to_width(
                    "Wind: ---  (true deg from N, /simulator/true_wind_direction)", max_text))
            else:
                put_row(self._trunc_to_width(
                    f"Wind: {eff_wind:.1f}°  (true from N, /simulator/true_wind_direction)", max_text))
            
            if self.simulation_paused:
                pause_mode = "SIGSTOP" if self.use_process_pause else "topic"
                put_row(f"Simulation: FROZEN ({pause_mode})")
                put_row(self._trunc_to_width("markers persist in Foxglove", max_text))
            else:
                put_row("Simulation: RUNNING")
            
            controller_name = self.current_controller_type.upper() if self.current_controller_type else 'UNKNOWN'
            human_mode = (
                (self.current_controller_type or '').strip().lower() == 'human'
                or getattr(self, '_controller_runtime_name', '') == 'HumanController'
            )
            pause_status = ' [HUMAN]' if human_mode else ''
            rth = self.is_rth_mode
            controller_status = f"Controller: {controller_name}{pause_status}"
            if rth:
                controller_status += " (RTH MODE)"
            else:
                controller_status += " (DEFAULT)"
            if len(controller_status) <= max_text:
                put_row(controller_status)
            else:
                put_row(self._trunc_to_width(f"Controller: {controller_name}{pause_status}", max_text))
                put_row("(RTH MODE)" if rth else "(DEFAULT)")
            
            line_num += 1
            if line_num <= last_content_row:
                try:
                    self.stdscr.addstr(line_num, 2, self._trunc_to_width("Controls:", max_text))
                except curses.error:
                    pass
                line_num += 1
            
            for ctl in (
                "  Left/Right Rudder | Up/Down Sail | C=Center",
                "  SPACE=Sim Pause | W/E=Wind +/-10 deg",
                "  T=Select Controller | R=Reset | H=Toggle RTH",
                "  M=Toggle Manual | Q=Quit Sim | X=Quit Control | ENTER=Refresh",
            ):
                put_row(self._trunc_to_width(ctl, max_text))
            
            put_row(self._trunc_to_width("Publishing to: /rudder_sail_radio", max_text))
            
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
                        self.stdscr.addstr(diag_line_num, 2, "-" * min(40, max_text))
                        diag_line_num += 1
                    except Exception:
                        pass
                
                for msg, msg_time in self.diagnostic_messages[-self.max_diagnostic_messages:]:
                    # Split long messages across multiple lines
                    msg_lines = self._split_message(msg, width - 4)
                    for msg_line in msg_lines:
                        if diag_line_num < height - 2:  # Leave room for border
                            try:
                                self.stdscr.addstr(
                                    diag_line_num, 2, self._trunc_to_width(msg_line, max_text))
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
    executor = None
    try:
        node = KeyboardControlNode()
        # Use MultiThreadedExecutor to allow nested service calls
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        # Spin with proper KeyboardInterrupt handling
        try:
            executor.spin()
        except KeyboardInterrupt:
            # Ctrl+C pressed - ensure cleanup happens
            pass
    except KeyboardInterrupt:
        # Ctrl+C during initialization
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # Always cleanup properly
        print("\nShutting down keyboard control...")
        if node:
            # Trigger cleanup explicitly (restore terminal, resume simulation if paused)
            try:
                node.cleanup()
            except Exception as e:
                print(f"Error during cleanup: {e}")
        if executor:
            try:
                executor.shutdown()
            except:
                pass
        if node:
            try:
                node.destroy_node()
            except:
                pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass
        print("Keyboard control stopped.")

if __name__ == '__main__':
    main()
