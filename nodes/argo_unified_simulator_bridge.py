#!/usr/bin/env python3
"""
Argo Unified Simulator Bridge - Local and Remote Simulation Support
Connects Argo control system with sailboat simulator (local or remote)

This unified bridge can operate in two modes:
1. LOCAL mode: Runs the simulator directly on this machine
2. REMOTE mode: Connects to a remote simulator via ROS2 topics

Usage:
  python3 argo_unified_simulator_bridge.py --mode local    # Local simulation
  python3 argo_unified_simulator_bridge.py --mode remote   # Remote simulation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String, UInt8
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix, NavSatStatus
import numpy as np
import time
import math
import threading
import argparse
import sys
import os
import curses
import queue
import signal

# Try to import sailboat-playground for local simulation
try:
    # Set headless mode for pyglet (no display required)
    os.environ['PYGLET_HEADLESS'] = '1'
    
    # Add simulator submodule to Python path
    simulator_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'simulator', 'sailboat-playground')
    if simulator_path not in sys.path:
        sys.path.insert(0, simulator_path)
    
    # Import sailboat-playground modules
    import sailboat_playground
    available_attrs = [x for x in dir(sailboat_playground) if not x.startswith('_')]
    print(f"INFO: sailboat-playground module available, contents: {available_attrs}")
    
    # Try to import the engine module for simulation
    try:
        from sailboat_playground import engine
        print("INFO: sailboat-playground engine module imported successfully")
        SIMULATOR_AVAILABLE = True
    except ImportError as e:
        print(f"INFO: sailboat-playground engine not available: {e}")
        SIMULATOR_AVAILABLE = False
        
except ImportError:
    print("WARNING: sailboat-playground not available, using mock simulator")
    SIMULATOR_AVAILABLE = False
except Exception as e:
    print(f"WARNING: sailboat-playground failed to initialize: {e}")
    SIMULATOR_AVAILABLE = False

class CursesDisplayManager:
    """Manages curses-based display with control window on top and log window below."""
    
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.log_queue = queue.Queue()
        self.control_data = {
            'rudder': 0.0,
            'sail': 0.0,
            'heading': 0.0,
            'speed': 0.0,
            'wind_speed': 0.0,
            'wind_direction': 0.0,
            'mode': 'HUMAN',
            'last_cmd': 0.0
        }
        self._cleanup_called = False
        
        # Setup curses
        try:
            curses.curs_set(0)  # Hide cursor
            curses.noecho()     # Don't echo keystrokes
            curses.cbreak()     # Immediate key input
            stdscr.keypad(True)  # Enable keypad
            stdscr.nodelay(True)  # Enable non-blocking input for continuous keyboard handling
            
            # Initialize colors if supported
            if curses.has_colors():
                curses.start_color()
                # Define color pairs
                curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)      # Error
                curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)   # Warning
                curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK)    # Info
                curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)     # Debug
                curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_BLUE)     # Control title
                self.colors_available = True
            else:
                self.colors_available = False
        except curses.error:
            # If curses setup fails, we'll handle it gracefully
            self.colors_available = False
        
        # Get screen dimensions
        self.height, self.width = stdscr.getmaxyx()
        
        # Calculate window sizes (ensure minimum height)
        self.control_height = 12  # Increased for better spacing (was 10, now 12)
        self.log_height = max(10, self.height - self.control_height - 2)  # Leave 2 lines buffer
        
        # Create windows
        self.control_win = stdscr.subwin(self.control_height, self.width, 0, 0)
        self.log_win = stdscr.subwin(self.log_height, self.width, self.control_height, 0)
        
        # Enable scrolling for log window
        self.log_win.scrollok(True)
        self.log_win.idlok(True)
        
        # Log buffer
        self.log_lines = []
        self.max_log_lines = self.log_height - 2
        
        # Setup signal handlers for clean exit
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGWINCH, self._signal_handler)  # Handle terminal resize
        
        # Keyboard control state
        self.keyboard_control_enabled = True
        self.rudder_position = 0.0  # -1.0 to +1.0
        self.sail_position = 0.0    # -1.0 to +1.0
        self.step_size = 1.0 / 8.0  # 8 steps for full scale (0.125)
        
        self.running = True
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully."""
        if signum == signal.SIGWINCH:
            # Handle terminal resize
            self._handle_resize()
        else:
            # Handle shutdown signals
            self.running = False
            self.cleanup()
            # Signal ROS2 to shutdown gracefully
            if rclpy.ok():
                rclpy.shutdown()
    
    def cleanup(self):
        """Restore terminal to default state."""
        if self._cleanup_called:
            return  # Prevent multiple cleanup calls
        self._cleanup_called = True
        
        try:
            # Restore curses window settings
            if hasattr(self, 'stdscr') and self.stdscr:
                self.stdscr.keypad(False)
                self.stdscr.nodelay(False)
            
            # Restore terminal to normal mode (with error handling)
            try:
                curses.nocbreak()      # Disable cbreak mode
            except curses.error:
                pass
            try:
                curses.echo()          # Re-enable echo
            except curses.error:
                pass
            try:
                curses.curs_set(1)     # Show cursor
            except curses.error:
                pass
            try:
                curses.noraw()         # Disable raw mode (if it was enabled)
            except curses.error:
                pass
            try:
                curses.endwin()        # End curses session
            except curses.error:
                pass
            
        except Exception as e:
            # If curses cleanup fails, try basic terminal restoration
            try:
                import termios
                import tty
                # This is a fallback - restore to cooked mode
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
            except:
                pass
    
    def __del__(self):
        """Destructor to ensure cleanup is called."""
        if not self._cleanup_called:
            self.cleanup()
    
    def _handle_resize(self):
        """Handle terminal resize event."""
        try:
            # Get new screen dimensions
            new_height, new_width = self.stdscr.getmaxyx()
            
            # Check if dimensions actually changed
            if new_height == self.height and new_width == self.width:
                return
            
            # Update dimensions
            self.height = new_height
            self.width = new_width
            
            # Recalculate window sizes
            self.control_height = 12  # Keep control window height fixed
            self.log_height = max(10, self.height - self.control_height - 2)
            
            # Recreate windows with new dimensions
            self.control_win = self.stdscr.subwin(self.control_height, self.width, 0, 0)
            self.log_win = self.stdscr.subwin(self.log_height, self.width, self.control_height, 0)
            
            # Re-enable scrolling for log window
            self.log_win.scrollok(True)
            self.log_win.idlok(True)
            
            # Update max log lines
            self.max_log_lines = self.log_height - 2
            
            # Clear and redraw
            self.stdscr.clear()
            self.update_display()
            
        except Exception as e:
            # If resize fails, just continue with current dimensions
            pass
    
    def update_control_data(self, **kwargs):
        """Update control display data."""
        self.control_data.update(kwargs)
    
    def log_message(self, message, level="info"):
        """Add message to log queue with level."""
        self.log_queue.put((message, level))
    
    def get_log_icon(self, level):
        """Get colored icon for log level."""
        if not self.colors_available:
            # Fallback to text icons
            icons = {
                "error": "❌",
                "warning": "⚠️",
                "info": "ℹ️",
                "debug": "🔍",
                "success": "✅"
            }
            return icons.get(level, "ℹ️")
        
        # Colored icons
        icons = {
            "error": ("❌", curses.color_pair(1)),
            "warning": ("⚠️", curses.color_pair(2)),
            "info": ("ℹ️", curses.color_pair(3)),
            "debug": ("🔍", curses.color_pair(4)),
            "success": ("✅", curses.color_pair(3))
        }
        return icons.get(level, ("ℹ️", curses.color_pair(3)))
    
    def create_control_bar(self, value, label, width=20):
        """Create ASCII bar visualization for control position."""
        # Normalize value to 0-1 range for bar calculation
        normalized = (value + 1.0) / 2.0  # -1..+1 -> 0..1
        
        # Create bar with specified width
        filled_length = int(normalized * width)
        
        # Create bar string
        bar = "[" + "█" * filled_length + "░" * (width - filled_length) + "]"
        
        # Add direction indicators
        if value < -0.1:
            direction = "← LEFT"
        elif value > 0.1:
            direction = "RIGHT →"
        else:
            direction = "CENTER"
        
        return f"{bar} {direction}"
    
    def draw_control_window(self):
        """Draw the control window with rudder/sail status."""
        try:
            self.control_win.clear()
            self.control_win.border()
        except curses.error:
            # Window might be invalid due to resize, skip this update
            return
        
        # Title (with proper spacing)
        title = "🚢 ARGO SIMULATOR CONTROL"
        self.control_win.addstr(1, (self.width - len(title)) // 2, title)
        
        # Empty line for spacing
        self.control_win.addstr(2, 1, " " * (self.width - 2))
        
        # Rudder visualization
        rudder_bar = self.create_control_bar(self.control_data['rudder'], "RUDDER", 16)
        self.control_win.addstr(3, 2, f"Rudder: {rudder_bar} ({self.control_data['rudder']:+.3f})")
        
        # Sail visualization  
        sail_bar = self.create_control_bar(self.control_data['sail'], "SAIL", 16)
        self.control_win.addstr(4, 2, f"Sail:   {sail_bar} ({self.control_data['sail']:+.3f})")
        
        # Empty line for spacing
        self.control_win.addstr(5, 1, " " * (self.width - 2))
        
        # Status info
        status_line = f"Mode: {self.control_data['mode']} | Heading: {self.control_data['heading']:.1f}° | Speed: {self.control_data['speed']:.1f}m/s | Wind: {self.control_data['wind_speed']:.1f}m/s @ {self.control_data['wind_direction']:.0f}°"
        if len(status_line) > self.width - 4:
            status_line = status_line[:self.width - 7] + "..."
        self.control_win.addstr(6, 2, status_line)
        
        # Keyboard controls
        controls_line = "Controls: ←→ Rudder | ↑↓ Sail | C=Center | Q=Quit"
        if len(controls_line) > self.width - 4:
            controls_line = controls_line[:self.width - 7] + "..."
        self.control_win.addstr(7, 2, controls_line)
        
        # Empty line for spacing
        self.control_win.addstr(8, 1, " " * (self.width - 2))
        
        self.control_win.refresh()
    
    def draw_log_window(self):
        """Draw the log window with recent messages."""
        try:
            self.log_win.clear()
            self.log_win.border()
        except curses.error:
            # Window might be invalid due to resize, skip this update
            return
        
        # Title
        title = "SIMULATION LOG"
        self.log_win.addstr(0, (self.width - len(title)) // 2, title)
        
        # Display log lines
        start_line = max(0, len(self.log_lines) - self.max_log_lines)
        for i, line_data in enumerate(self.log_lines[start_line:]):
            if i < self.max_log_lines:
                if isinstance(line_data, tuple):
                    line, color = line_data
                else:
                    line, color = line_data, curses.color_pair(0)
                
                # Truncate line if too long
                display_line = line[:self.width - 4]
                try:
                    if self.colors_available and color != curses.color_pair(0):
                        self.log_win.addstr(i + 1, 2, display_line, color)
                    else:
                        self.log_win.addstr(i + 1, 2, display_line)
                except curses.error:
                    # Handle line too long or other curses errors
                    pass
        
        self.log_win.refresh()
    
    def process_log_queue(self):
        """Process messages from log queue."""
        while not self.log_queue.empty():
            try:
                message_data = self.log_queue.get_nowait()
                if isinstance(message_data, tuple):
                    message, level = message_data
                else:
                    message, level = message_data, "info"
                
                # Add timestamp and icon
                timestamp = time.strftime('%H:%M:%S')
                icon_data = self.get_log_icon(level)
                if isinstance(icon_data, tuple):
                    icon, color = icon_data
                else:
                    icon, color = icon_data, curses.color_pair(0)
                formatted_message = f"[{timestamp}] {icon} {message}"
                
                self.log_lines.append((formatted_message, color))
                
                # Keep only recent log lines
                if len(self.log_lines) > self.max_log_lines * 2:
                    self.log_lines = self.log_lines[-self.max_log_lines:]
            except queue.Empty:
                break
    
    def update_display(self):
        """Update the entire display."""
        if not self.running:
            return
        
        try:
            # Check for terminal resize
            new_height, new_width = self.stdscr.getmaxyx()
            if new_height != self.height or new_width != self.width:
                self._handle_resize()
                return  # Skip this update cycle, let resize handle redraw
            
            # Handle keyboard input
            self.handle_keyboard_input()
            
            # Process log messages
            self.process_log_queue()
            
            # Draw windows
            self.draw_control_window()
            self.draw_log_window()
            
            # Refresh main screen
            self.stdscr.refresh()
            
        except curses.error as e:
            # Handle terminal resize or other curses errors
            # Try to recover by checking for resize
            try:
                new_height, new_width = self.stdscr.getmaxyx()
                if new_height != self.height or new_width != self.width:
                    self._handle_resize()
            except:
                pass
    
    def get_key(self):
        """Get a key press (non-blocking)."""
        try:
            # Set nodelay mode for non-blocking input
            self.stdscr.nodelay(True)
            key = self.stdscr.getch()
            # Keep nodelay mode for continuous input
            return key
        except:
            return -1
    
    def handle_keyboard_input(self):
        """Handle keyboard input for control."""
        if not self.keyboard_control_enabled:
            return
        
        key = self.get_key()
        if key == -1:
            return
        
        # Debug: Log key presses (uncomment for debugging)
        # try:
        #     key_name = curses.keyname(key).decode('utf-8') if key > 0 else 'unknown'
        #     self.log_message(f"Key: {key} ({key_name})", "debug")
        # except:
        #     self.log_message(f"Key: {key}", "debug")
        
        # Handle arrow keys
        if key == curses.KEY_UP:
            self.adjust_sail(1)  # Sail out
        elif key == curses.KEY_DOWN:
            self.adjust_sail(-1)  # Sail in
        elif key == curses.KEY_RIGHT:
            self.adjust_rudder(1)  # Rudder right
        elif key == curses.KEY_LEFT:
            self.adjust_rudder(-1)  # Rudder left
        elif key == ord('c') or key == ord('C'):
            self.center_controls()
        elif key == ord('q') or key == ord('Q'):
            self.running = False
        elif key == 3:  # Ctrl+C
            self.running = False
    
    def adjust_rudder(self, direction):
        """Adjust rudder position by one step."""
        self.control_data['rudder'] += direction * self.step_size
        self.control_data['rudder'] = max(-1.0, min(1.0, self.control_data['rudder']))
        self.log_message(f"Rudder adjusted to {self.control_data['rudder']:+.3f}", "info")
    
    def adjust_sail(self, direction):
        """Adjust sail position by one step."""
        self.control_data['sail'] += direction * self.step_size
        self.control_data['sail'] = max(-1.0, min(1.0, self.control_data['sail']))
        self.log_message(f"Sail adjusted to {self.control_data['sail']:+.3f}", "info")
    
    def center_controls(self):
        """Center both rudder and sail."""
        self.control_data['rudder'] = 0.0
        self.control_data['sail'] = 0.0
        self.log_message("Controls centered")

class MockSailboatSimulator:
    """Mock simulator for testing when sailboat-playground is not available."""
    
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.boat_heading = 0.0  # degrees
        self.boat_speed = 0.0    # m/s
        self.rudder_angle = 0.0  # -1 to +1
        self.sail_angle = 0.0    # -1 to +1
        
        # Wind conditions
        self.wind_speed = 8.0    # m/s
        self.wind_direction = 45.0  # degrees (where wind comes from)
        
        # Physics parameters
        self.dt = 0.1  # time step
        self.max_turn_rate = 30.0  # degrees per second
        self.max_speed = 5.0  # m/s
        
    def set_control(self, rudder, sail):
        """Set rudder and sail positions (-1 to +1)."""
        self.rudder_angle = np.clip(rudder, -1.0, 1.0)
        self.sail_angle = np.clip(sail, -1.0, 1.0)
    
    def step(self):
        """Update simulation by one time step."""
        # Simple boat physics simulation
        
        # Calculate apparent wind angle relative to boat
        wind_boat_angle = (self.wind_direction - self.boat_heading) % 360
        if wind_boat_angle > 180:
            wind_boat_angle -= 360
            
        # Speed based on wind angle and sail setting
        # Best speed when wind is 90-120 degrees off bow
        wind_efficiency = max(0.1, abs(math.sin(math.radians(wind_boat_angle))))
        sail_efficiency = 1.0 - abs(self.sail_angle * 0.3)  # Better when sail is pulled in
        target_speed = self.wind_speed * 0.3 * wind_efficiency * sail_efficiency
        target_speed = min(target_speed, self.max_speed)
        
        # Smooth speed changes
        speed_diff = target_speed - self.boat_speed
        self.boat_speed += np.clip(speed_diff * 2.0 * self.dt, -1.0, 1.0)
        self.boat_speed = max(0.0, self.boat_speed)
        
        # Turning based on rudder
        if self.boat_speed > 0.5:  # Need some speed to turn
            turn_rate = self.rudder_angle * self.max_turn_rate * (self.boat_speed / self.max_speed)
            self.boat_heading += turn_rate * self.dt
            self.boat_heading = self.boat_heading % 360
        
        # Update position
        self.boat_x += self.boat_speed * math.cos(math.radians(self.boat_heading)) * self.dt
        self.boat_y += self.boat_speed * math.sin(math.radians(self.boat_heading)) * self.dt
        
        return {
            'x': self.boat_x,
            'y': self.boat_y,
            'heading': self.boat_heading,
            'speed': self.boat_speed,
            'wind_speed': self.wind_speed,
            'wind_direction': wind_boat_angle,  # Relative to boat
            'rudder': self.rudder_angle,
            'sail': self.sail_angle
        }

class ArgoUnifiedSimulatorBridge(Node):
    """Unified bridge for local and remote sailboat simulation."""
    
    def __init__(self, mode='local', use_curses=True):
        super().__init__('argo_unified_simulator_bridge')
        self.mode = mode
        self.use_curses = use_curses
        self.display_manager = None
        
        if self.use_curses:
            # Initialize curses display
            self.stdscr = curses.initscr()
            self.display_manager = CursesDisplayManager(self.stdscr)
            self.display_manager.log_message(f'Argo Unified Simulator Bridge starting in {mode.upper()} mode...')
            
            # Override the logger to send messages to curses display
            self._original_logger = self.get_logger()
            self._setup_curses_logger()
        else:
            self.get_logger().info(f'Argo Unified Simulator Bridge starting in {mode.upper()} mode...')
        
        # Initialize simulator based on mode
        if mode == 'local':
            self._init_local_simulator()
        else:  # remote mode
            self._init_remote_simulator()
        
        # Common state
        self.last_control_time = time.time()
        self.simulation_running = True
        self.human_controlled = True  # Start in human control
        self.mock_human_input = True
        self.human_input_time = 0.0
        
        # --- GPS Base Location (for NavSatFix) ---
        self.base_latitude = 47.3769  # Zurich, Switzerland
        self.base_longitude = 8.5417
        
        # Remote mode specific state
        if mode == 'remote':
            self.last_remote_data_time = time.time()
            self.remote_connected = False
            self.control_mode = True
        
        # --- Publishers (Simulator → Argo) ---
        # IMU/Compass data
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        
        # Keyboard control (if curses enabled)
        if self.use_curses:
            self.pub_rudder_sail_radio = self.create_publisher(Vector3, '/rudder_sail_radio', 10)
        
        # GPS data
        self.pub_gps_cog = self.create_publisher(Float64, '/gps_cog', 10)
        self.pub_gps_sog = self.create_publisher(Float64, '/gps_sog', 10)
        self.pub_gps_velocity = self.create_publisher(Vector3, '/gps_velocity', 10)
        self.pub_gps_satellites = self.create_publisher(UInt8, '/gps_num_satellites', 10)
        self.pub_gps_fix = self.create_publisher(NavSatFix, '/fix', 10)
        self.pub_gps_data = self.create_publisher(String, '/gps_data', 10)

        # Wind data
        self.pub_wind = self.create_publisher(Vector3, '/anem_speed_angle_temp', 10)
        
        # Simulated radio input (for testing human control)
        self.pub_radio = self.create_publisher(Vector3, '/rudder_sail_radio', 10)
        
        # --- Subscribers (Argo → Simulator) ---
        # Control commands from Argo
        self.create_subscription(Vector3, '/rudder_sail_servo', self.control_callback, 10)
        
        # Monitor control authority
        self.create_subscription(Bool, '/human_controlled', self.human_control_callback, 10)
        
        # --- Simulation Parameters ---
        self.simulation_rate = 10.0  # Hz
        
        # --- Timers ---
        if mode == 'local':
            self.sim_timer = self.create_timer(1.0/self.simulation_rate, self.local_simulation_step)
        else:  # remote mode
            self.status_timer = self.create_timer(5.0, self.check_remote_connection)
            self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        # Keyboard control timer (if curses enabled)
        if self.use_curses:
            self.keyboard_timer = self.create_timer(0.1, self.publish_keyboard_control)
        
        # Control arbitration publishers (simulate rudder_sail_radio.py functionality)
        self.pub_human_controlled = self.create_publisher(Bool, '/human_controlled', 10)
        self.pub_control_authority = self.create_publisher(Vector3, '/control_authority', 10)
        
        # Control arbitration state
        self.human_controlled = True  # Start in human control for safety
        self.last_human_activity = time.time()
        self.last_auto_update = 0.0
        self.control_arbitration_timer = self.create_timer(0.1, self.publish_control_arbitration)
        
        # Subscribe to autonomous commands (from controller.py)
        self.create_subscription(Vector3, '/rudder_sail_cmd', self.auto_control_callback, 10)
        
        self.get_logger().info(f'Unified simulator bridge ready ({mode} mode)')
        self.get_logger().info('Publishing simulated sensor data to Argo topics')
        
        # Test colored logging if curses is enabled
        if self.use_curses and self.display_manager:
            self.display_manager.log_message("Simulator bridge initialized successfully", "success")
            self.display_manager.log_message("Keyboard control enabled - use arrow keys", "info")
            self.display_manager.log_message("Press 'c' to center controls, 'q' to quit", "info")
        
        # Initial state
        self.boat_state = None
    
    def _setup_curses_logger(self):
        """Setup logger to send messages to curses display."""
        class CursesLogger:
            def __init__(self, display_manager, original_logger):
                self.display_manager = display_manager
                self.original_logger = original_logger
            
            def info(self, msg):
                self.display_manager.log_message(f"INFO: {msg}")
                self.original_logger.info(msg)
            
            def warn(self, msg):
                self.display_manager.log_message(f"WARN: {msg}")
                self.original_logger.warn(msg)
            
            def error(self, msg):
                self.display_manager.log_message(f"ERROR: {msg}")
                self.original_logger.error(msg)
            
            def debug(self, msg):
                self.display_manager.log_message(f"DEBUG: {msg}")
                self.original_logger.debug(msg)
        
        # Replace the logger
        self.get_logger = lambda: CursesLogger(self.display_manager, self._original_logger)
    
    def publish_keyboard_control(self):
        """Publish keyboard control commands."""
        if not self.use_curses or not self.display_manager:
            return
        
        # Get current control positions from display manager
        rudder = self.display_manager.control_data['rudder']
        sail = self.display_manager.control_data['sail']
        
        # Check for human activity (keyboard input changes)
        if hasattr(self, 'last_rudder') and hasattr(self, 'last_sail'):
            rudder_change = abs(rudder - self.last_rudder)
            sail_change = abs(sail - self.last_sail)
            
            # If there's significant change, update human activity
            if rudder_change > 0.01 or sail_change > 0.01:  # Small threshold for keyboard input
                self.last_human_activity = time.time()
                self.human_controlled = True
        
        # Store current values for next comparison
        self.last_rudder = rudder
        self.last_sail = sail
        
        # Create control message in same format as hardware radio
        control_msg = Vector3()
        control_msg.x = rudder  # Rudder: -1=left, +1=right
        control_msg.y = sail    # Sail: -1=in, +1=out
        control_msg.z = 0.0     # Reserved
        
        self.pub_rudder_sail_radio.publish(control_msg)
    
    def publish_control_arbitration(self):
        """Publish control arbitration status (simulate rudder_sail_radio.py functionality)."""
        current_time = time.time()
        
        # Check for recent human activity (within 2 seconds)
        time_since_human_activity = current_time - self.last_human_activity
        time_since_auto_update = current_time - self.last_auto_update
        human_override_timeout = 2.0  # seconds
        
        # Human has control if there's been recent activity
        if time_since_human_activity < human_override_timeout:
            self.human_controlled = True
        else:
            # Check if we have recent autonomous commands
            if time_since_auto_update < 1.0:  # Auto commands are fresh
                self.human_controlled = False
            else:
                # Default to human control for safety if no recent commands
                self.human_controlled = True
        
        # Publish human control status
        human_msg = Bool(data=self.human_controlled)
        self.pub_human_controlled.publish(human_msg)
        
        # Publish detailed control authority info
        authority_msg = Vector3(
            x=1.0 if self.human_controlled else 0.0,  # Current authority (1=human, 0=robot)
            y=time_since_human_activity,               # Time since human activity
            z=time_since_auto_update                   # Time since auto command
        )
        self.pub_control_authority.publish(authority_msg)
    
    def auto_control_callback(self, msg):
        """Receive autonomous control commands from controller.py."""
        self.last_auto_update = time.time()
        # Store autonomous commands for potential use
        self.auto_rudder = msg.x
        self.auto_sail = msg.y
    
    def _init_local_simulator(self):
        """Initialize local simulator (sailboat-playground or mock)."""
        if SIMULATOR_AVAILABLE:
            try:
                self.get_logger().info('Initializing sailboat-playground simulator...')
                from sailboat_playground.engine import Manager
                import numpy as np
                
                # Configuration file paths
                boat_config = "simulator/customizations/sailboat-playground/boats/sample_boat.json"
                env_config = "simulator/customizations/sailboat-playground/environments/playground.json"
                
                # Check if configuration files exist
                if not os.path.exists(boat_config):
                    raise FileNotFoundError(f"Boat configuration file not found: {boat_config}")
                if not os.path.exists(env_config):
                    raise FileNotFoundError(f"Environment configuration file not found: {env_config}")
                
                # Initialize the sailboat-playground simulation manager with configs
                self.sim_manager = Manager(
                    boat_config,
                    env_config,
                    foils_dir="simulator/foils/",        # Path to foil data files
                    boat_heading=0.0,                    # Start facing north
                    boat_position=np.array([0.0, 0.0]),  # Start at origin
                    debug=False
                )
                
                self.simulator = self.sim_manager  # Use the manager as our simulator interface
                self.use_mock = False
                self.get_logger().info('Real sailboat-playground simulator initialized successfully')
                self.get_logger().info(f'Using boat config: {boat_config}')
                self.get_logger().info(f'Using environment config: {env_config}')
            except Exception as e:
                import traceback
                self.get_logger().warn(f'Failed to initialize real simulator: {e}')
                tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
                self.get_logger().warn(f'Stack trace for simulator initialization failure:\n{tb_str}')
                self.get_logger().info('Falling back to mock simulator (reliable for headless operation)')
                self.simulator = MockSailboatSimulator()
                self.use_mock = True
        else:
            self.get_logger().info('Using mock simulator (reliable for headless operation)')
            self.simulator = MockSailboatSimulator()
            self.use_mock = True
    
    def _init_remote_simulator(self):
        """Initialize remote simulator connection."""
        self.get_logger().info('Remote simulator mode - waiting for connection...')
        # Remote simulator state will be managed by callbacks
    
    def local_simulation_step(self):
        """Main simulation step for local mode - updates physics and publishes sensor data."""
        try:
            # Generate mock human input for testing (optional)
            if self.mock_human_input and self.human_controlled:
                self.publish_mock_human_input()
            
            # Update simulator physics
            if self.use_mock:
                self.boat_state = self.simulator.step()
            else:
                # Handle real sailboat-playground API
                try:
                    # Get current rudder and sail angles (default to 0 if no control received)
                    rudder_angle = getattr(self, 'last_rudder_angle', 0.0)
                    sail_angle = getattr(self, 'last_sail_angle', 0.0)
                    
                    # Step the simulation with control inputs
                    self.sim_manager.step([sail_angle, rudder_angle])
                    
                    # Get the current state from sailboat-playground
                    state = self.sim_manager.agent_state
                    
                    # Convert to our internal boat_state format
                    self.boat_state = {
                        'x': state['position'][0],
                        'y': state['position'][1], 
                        'heading': state['heading'],
                        'speed': np.linalg.norm(state['velocity']) if 'velocity' in state else 2.0,
                        'wind_speed': state.get('wind_speed', 8.0),
                        'wind_direction': state.get('wind_direction', 45.0),
                        'rudder': rudder_angle,
                        'sail': sail_angle
                    }
                except Exception as e:
                    self.get_logger().warn(f'Simulator step failed: {e}')
                    return
            
            # Publish sensor data to Argo
            self.publish_sensor_data()
            
        except Exception as e:
            self.get_logger().error(f'Simulation step error: {e}')
    
    def publish_sensor_data(self):
        """Publish simulated sensor data to Argo topics."""
        if not self.boat_state:
            return
        
        # IMU/Compass data (heading in degrees)
        pose_msg = Vector3(x=0.0, y=0.0, z=self.boat_state['heading'])
        self.pub_pose.publish(pose_msg)
        
        compass_msg = Vector3(x=0.0, y=0.0, z=self.boat_state['heading'])
        self.pub_compass.publish(compass_msg)
        
        # GPS data
        gps_cog_msg = Float64(data=self.boat_state['heading'])  # Course over ground
        self.pub_gps_cog.publish(gps_cog_msg)
        
        # Speed over ground (convert m/s to knots)
        speed_knots = self.boat_state['speed'] * 1.94384  # m/s to knots
        gps_sog_msg = Float64(data=speed_knots)
        self.pub_gps_sog.publish(gps_sog_msg)
        
        # GPS velocity vector (north, east, speed)
        heading_rad = math.radians(self.boat_state['heading'])
        vel_north = self.boat_state['speed'] * math.cos(heading_rad) * 1.94384  # knots
        vel_east = self.boat_state['speed'] * math.sin(heading_rad) * 1.94384   # knots
        gps_vel_msg = Vector3(x=vel_north, y=vel_east, z=speed_knots)
        self.pub_gps_velocity.publish(gps_vel_msg)
        
        # Wind data (speed, angle relative to boat, temperature)
        wind_msg = Vector3(
            x=self.boat_state['wind_speed'],     # m/s
            y=self.boat_state['wind_direction'], # degrees relative to boat
            z=22.5                               # temperature (mock)
        )
        self.pub_wind.publish(wind_msg)

        # Publish satellite count
        sat_msg = UInt8()
        sat_msg.data = 12  # Mock satellite count
        self.pub_gps_satellites.publish(sat_msg)

        # Publish NavSatFix message for mapping
        self.publish_navsat_fix()

        # Publish mock NMEA RMC sentence
        self.publish_mock_nmea()

    def publish_mock_human_input(self):
        """Generate mock human radio input for testing."""
        self.human_input_time += 1.0 / self.simulation_rate
        
        # Generate some human-like control input
        rudder = 0.3 * math.sin(self.human_input_time * 0.5)
        sail = 0.2 + 0.2 * math.cos(self.human_input_time * 0.3)
        
        radio_msg = Vector3(x=rudder, y=sail, z=0.0)
        self.pub_radio.publish(radio_msg)

    def xy_to_latlon(self, x, y):
        """Convert XY meters from base lat/lon to new lat/lon."""
        R = 6378137.0  # Earth radius in meters
        dLat = y / R
        dLon = x / (R * math.cos(math.pi * self.base_latitude / 180))
        lat = self.base_latitude + dLat * 180 / math.pi
        lon = self.base_longitude + dLon * 180 / math.pi
        return lat, lon

    def publish_navsat_fix(self):
        """Publish a NavSatFix message."""
        if not self.boat_state:
            return

        lat, lon = self.xy_to_latlon(self.boat_state['x'], self.boat_state['y'])

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = 'gps'
        fix_msg.status.status = NavSatStatus.STATUS_FIX
        fix_msg.status.service = NavSatStatus.SERVICE_GPS
        fix_msg.latitude = lat
        fix_msg.longitude = lon
        fix_msg.altitude = 0.0  # Mock altitude
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.pub_gps_fix.publish(fix_msg)

    def publish_mock_nmea(self):
        """Publish a mock NMEA RMC sentence."""
        if not self.boat_state:
            return

        lat, lon = self.xy_to_latlon(self.boat_state['x'], self.boat_state['y'])
        
        # Format latitude for NMEA (DDMM.MMMM)
        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lat_dir = 'N' if lat >= 0 else 'S'
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"

        # Format longitude for NMEA (DDDMM.MMMM)
        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60
        lon_dir = 'E' if lon >= 0 else 'W'
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        
        speed_knots = self.boat_state['speed'] * 1.94384
        course_deg = self.boat_state['heading']
        
        timestamp = time.strftime("%H%M%S.00", time.gmtime())
        datestamp = time.strftime("%d%m%y", time.gmtime())

        nmea_sentence = f"$GPRMC,{timestamp},A,{lat_str},{lat_dir},{lon_str},{lon_dir},{speed_knots:.2f},{course_deg:.2f},{datestamp},,,"
        
        # Calculate checksum
        checksum = 0
        for char in nmea_sentence[1:]:
            checksum ^= ord(char)
        
        nmea_sentence += f"*{checksum:02X}"
        
        self.pub_gps_data.publish(String(data=nmea_sentence))

    def control_callback(self, msg):
        """Receive control commands from Argo and apply to simulator."""
        self.last_control_time = time.time()
        
        # Apply control to simulator
        rudder = msg.x  # -1 to +1
        sail = msg.y    # -1 to +1
        
        if self.mode == 'local':
            if self.use_mock:
                self.simulator.set_control(rudder, sail)
            else:
                # Store control values for real simulator (they're applied in simulation_step)
                # sailboat-playground expects angles in degrees, Argo sends normalized values
                self.last_rudder_angle = rudder * 30.0  # Convert to degrees (-30 to +30)
                self.last_sail_angle = sail * 45.0      # Convert to degrees (-45 to +45)
        else:  # remote mode
            # In remote mode, we just forward the control command
            # The actual remote simulator will handle the control
            pass
        
        self.get_logger().debug(f'Applied control: rudder={rudder:.3f}, sail={sail:.3f}')
    
    def human_control_callback(self, msg):
        """Monitor human control status."""
        if self.human_controlled != msg.data:
            self.human_controlled = msg.data
            mode = "HUMAN" if msg.data else "ROBOT"
            self.get_logger().info(f'Control mode: {mode}')
    
    # Remote mode callbacks
    def pose_callback(self, msg):
        """Receive pose data from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            if not self.remote_connected:
                self.remote_connected = True
                self.get_logger().info('✅ Connected to remote simulator')
            # Forward to local topics
            self.pub_pose.publish(msg)
    
    def compass_callback(self, msg):
        """Receive compass data from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            self.pub_compass.publish(msg)
    
    def gps_cog_callback(self, msg):
        """Receive GPS course over ground from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            self.pub_gps_cog.publish(msg)
    
    def gps_sog_callback(self, msg):
        """Receive GPS speed over ground from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            self.pub_gps_sog.publish(msg)
    
    def gps_velocity_callback(self, msg):
        """Receive GPS velocity from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            self.pub_gps_velocity.publish(msg)
    
    def wind_callback(self, msg):
        """Receive wind data from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            self.pub_wind.publish(msg)
    
    def radio_callback(self, msg):
        """Receive radio input from remote simulator"""
        if self.mode == 'remote':
            self.last_remote_data_time = time.time()
            self.pub_radio.publish(msg)
    
    def check_remote_connection(self):
        """Check if remote simulator is still connected"""
        if self.mode == 'remote':
            time_since_data = time.time() - self.last_remote_data_time
            
            if time_since_data > 10.0:  # 10 seconds timeout
                if self.remote_connected:
                    self.remote_connected = False
                    self.get_logger().warn('❌ Lost connection to remote simulator')
            elif not self.remote_connected and time_since_data < 5.0:
                self.remote_connected = True
                self.get_logger().info('✅ Reconnected to remote simulator')
    
    def send_heartbeat(self):
        """Send heartbeat to remote simulator"""
        if self.mode == 'remote' and self.remote_connected:
            # Send current control mode
            control_msg = Bool()
            control_msg.data = self.control_mode
            # This would need to be published to the remote simulator
            # For now, we just log it
            self.get_logger().debug('Sending heartbeat to remote simulator')
    
    def print_status(self):
        """Print simulation status periodically."""
        if self.mode == 'local' and self.boat_state:
            # Use control arbitration timing instead of last_control_time
            if self.human_controlled:
                control_age = time.time() - self.last_human_activity
            else:
                control_age = time.time() - self.last_auto_update
            mode = "HUMAN" if self.human_controlled else "ROBOT"
            
            # Update curses display if available
            if self.display_manager:
                # Use keyboard control values if curses is enabled, otherwise use boat state
                if self.use_curses:
                    rudder = self.display_manager.control_data['rudder']
                    sail = self.display_manager.control_data['sail']
                else:
                    rudder = self.boat_state.get('rudder', 0.0)
                    sail = self.boat_state.get('sail', 0.0)
                
                self.display_manager.update_control_data(
                    rudder=rudder,
                    sail=sail,
                    heading=self.boat_state['heading'],
                    speed=self.boat_state['speed'],
                    wind_speed=self.boat_state['wind_speed'],
                    wind_direction=self.boat_state['wind_direction'],
                    mode=mode,
                    last_cmd=control_age
                )
                self.display_manager.update_display()
            
            # Log status message
            self.get_logger().info(
                f'Boat: heading={self.boat_state["heading"]:.1f}°, '
                f'speed={self.boat_state["speed"]:.1f}m/s, '
                f'wind={self.boat_state["wind_direction"]:.0f}°, '
                f'mode={mode}, '
                f'last_cmd={control_age:.1f}s ago'
            )
        elif self.mode == 'remote':
            status = "connected" if self.remote_connected else "disconnected"
            self.get_logger().info(f'Remote simulator: {status}')
    
    def destroy_node(self):
        """Clean up when node is destroyed."""
        if self.display_manager:
            self.display_manager.cleanup()
        super().destroy_node()

def print_help():
    """Print detailed help information and exit."""
    help_text = """
Argo Unified Simulator Bridge - Local and Remote Simulation Support

DESCRIPTION:
This ROS2 node provides a unified bridge between the Argo autonomous sailboat 
control system and a sailboat simulator, supporting both local and remote operation.

MODES:
1. LOCAL MODE: Runs the simulator directly on this machine
   - Uses sailboat-playground or mock simulator
   - Provides simulated sensor data
   - Handles control commands locally

2. REMOTE MODE: Connects to a remote simulator via ROS2 topics
   - Forwards sensor data from remote simulator
   - Sends control commands to remote simulator
   - Monitors connection status

USAGE:
  python3 argo_unified_simulator_bridge.py --mode local    # Local simulation with curses display
  python3 argo_unified_simulator_bridge.py --mode remote   # Remote simulation with curses display
  python3 argo_unified_simulator_bridge.py --mode local --no-curses  # Local simulation with standard logging

PUBLISHED TOPICS (Simulator → Argo):
  /pose                    - IMU compass heading (Vector3, z=heading degrees)
  /compass                 - Raw compass data (Vector3, z=heading degrees)
  /gps_cog                 - Course over ground (Float64, degrees)
  /gps_sog                 - Speed over ground (Float64, knots)
  /gps_velocity            - GPS velocity vector (Vector3, x=north, y=east, z=speed knots)
  /anem_speed_angle_temp   - Wind data (Vector3, x=speed m/s, y=angle degrees, z=temp °C)
  /rudder_sail_radio       - Mock human input (Vector3, x=rudder, y=sail, z=0)

SUBSCRIBED TOPICS (Argo → Simulator):
  /rudder_sail_servo       - Control commands from Argo (Vector3, x=rudder, y=sail)
  /human_controlled        - Control mode status (Bool, true=human, false=robot)

LOCAL MODE FEATURES:
- Mock sailboat physics with wind effects
- Realistic boat dynamics (speed, turning, wind interaction)
- Configurable wind conditions (speed: 8 m/s, direction: 45°)
- Mock human input generation for testing
- Real-time curses display with control visualization
- Status reporting every second

REMOTE MODE FEATURES:
- Connection monitoring with 10-second timeout
- Automatic reconnection detection
- Heartbeat mechanism for connection health
- Status reporting for connection state

CURSES DISPLAY FEATURES:
- Real-time control visualization with ASCII progress bars
- Rudder and sail position indicators
- Boat status information (heading, speed, wind)
- Scrolling log window with timestamped messages
- Clean terminal restoration on exit
- Responsive display updates at 10Hz

EXAMPLES:
  # Local simulation
  python3 argo_unified_simulator_bridge.py --mode local
  
  # Remote simulation
  python3 argo_unified_simulator_bridge.py --mode remote

For more information, see the Argo documentation or check the source code.
"""
    print(help_text)
    sys.exit(0)

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Argo Unified Simulator Bridge - Local and Remote Simulation Support',
        add_help=False  # We'll handle --help manually
    )
    parser.add_argument('--help', action='store_true', help='Show this help message and exit')
    parser.add_argument('--mode', choices=['local', 'remote'], default='local',
                       help='Simulation mode: local (run simulator here) or remote (connect to remote simulator)')
    parser.add_argument('--no-curses', action='store_true',
                       help='Disable curses display (use standard logging)')
    
    # Parse known args to avoid conflicts with ROS2 args
    parsed_args, unknown_args = parser.parse_known_args(args)
    
    # Handle --help option
    if parsed_args.help:
        print_help()
    
    use_curses = not parsed_args.no_curses
    
    if not use_curses:
        print(f"Starting Argo Unified Simulator Bridge in {parsed_args.mode.upper()} mode...")
        print("This node bridges between Argo control system and sailboat simulator")
        print(f"\nMode: {parsed_args.mode.upper()}")
        if parsed_args.mode == 'local':
            print("  - Running simulator locally")
            print("  - Using sailboat-playground or mock simulator")
        else:
            print("  - Connecting to remote simulator")
            print("  - Forwarding sensor data and control commands")
        
        print("\nPublished topics (Simulator → Argo):")
        print("  /pose - IMU compass heading")
        print("  /compass - Raw compass data") 
        print("  /gps_cog, /gps_sog, /gps_velocity - GPS navigation")
        print("  /anem_speed_angle_temp - Wind data")
        print("  /rudder_sail_radio - Mock human input (when enabled)")
        print("\nSubscribed topics (Argo → Simulator):")
        print("  /rudder_sail_servo - Control commands from Argo")
        print("  /human_controlled - Control mode status")
        print("\nPress Ctrl+C to stop\n")
    
    rclpy.init(args=unknown_args)
    bridge = None
    try:
        bridge = ArgoUnifiedSimulatorBridge(mode=parsed_args.mode, use_curses=use_curses)
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        if not use_curses:
            print(f"\nUnified simulator bridge ({parsed_args.mode} mode) stopped by user.")
    except Exception as e:
        if not use_curses:
            print(f"Error: {e}")
        else:
            # Clean up curses before printing error
            if bridge and bridge.display_manager:
                bridge.display_manager.cleanup()
            print(f"Error: {e}")
    finally:
        # Ensure proper cleanup in all cases
        if bridge:
            if bridge.display_manager:
                bridge.display_manager.cleanup()
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
