#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
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
from sensor_msgs.msg import NavSatFix, NavSatStatus, Joy
import numpy as np
import time
import math
import threading
import argparse
import argcomplete
import sys
import os
import signal
import traceback
import json
import yaml
from pathlib import Path

# Try to import sailboat-playground for local simulation
SIMULATOR_AVAILABLE = False
try:
    # Set headless mode for pyglet (no display required).
    # This prevents it from trying to create a window, but it might still try to
    # initialize a graphics context for loading resources, which we avoid by
    # only importing the 'engine' module.
    os.environ['PYGLET_HEADLESS'] = '1'
    
    # Add the simulator's PARENT directory to the Python path. This makes
    # 'sailboat_playground' a recognizable package, allowing the 'engine'
    # module's internal relative imports to work, while still bypassing the
    # top-level __init__.py that loads the visualization code.
    parent_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'simulator', 'sailboat-playground')
    
    # --- Diagnostic checks for sailboat-playground ---
    print(f"INFO: Attempting to load sailboat-playground from: {parent_path}")
    if not os.path.isdir(parent_path):
        print(f"WARNING: Simulator parent path does not exist or is not a directory.")
        print(f"  - Searched path: {os.path.abspath(parent_path)}")
    else:
        print("INFO: Simulator parent path found.")
        if parent_path not in sys.path:
            sys.path.insert(0, parent_path)
            print(f"INFO: Added to sys.path: {parent_path}")

        # Import the engine module from the package. Because we added the parent
        # dir to the path, this works without triggering the top-level __init__.py
        from sailboat_playground import engine
        print("INFO: sailboat-playground engine module imported successfully")
        SIMULATOR_AVAILABLE = True

except ImportError as e:
    error_msg = str(e)
    module_name = ""
    if "No module named" in error_msg:
        try:
            module_name = error_msg.split("'")[1]
        except IndexError:
            pass  # Keep module_name as empty string

    if module_name == 'engine':
        print("\nERROR: Failed to import the 'engine' module from sailboat-playground.")
        print("       This suggests an issue with the path or submodule structure, not a missing dependency.")
        tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        print(f'Stack trace for import failure:\n{tb_str}')

    elif module_name:
        print(f"WARNING: sailboat-playground not available due to missing Python package: '{module_name}'")
        print(f"         Please install simulation-specific dependencies by running:")
        print(f"           make install-simulation-deps")

    elif 'GLU' in error_msg:
        print(f"WARNING: sailboat-playground not available due to a missing system library (GLU).")
        print(f"         This is required by the pyglet visualization library.")
        print(f"         Please install the required system package by running:")
        print(f"           make install-simulation-deps")

    else:
        # Generic ImportError
        print(f"WARNING: sailboat-playground not available due to an import error.")
        print(f"         {e}")
        tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
        print(f'Stack trace for import failure:\n{tb_str}')

    print(f"         Falling back to mock simulator, ^c to exit.")
    time.sleep(7)
    SIMULATOR_AVAILABLE = False
except Exception as e:
    print(f"WARNING: sailboat-playground failed to initialize: {e}")
    tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
    print(f'Stack trace for initialization failure:\n{tb_str}')
    SIMULATOR_AVAILABLE = False

class MockSailboatSimulator:
    """Mock simulator for testing when sailboat-playground is not available."""
    
    def __init__(self):
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.boat_heading = 180.0  # degrees
        self.boat_speed = 1.0    # m/s
        self.rudder_angle = 0.0  # -1 to +1
        self.sail_angle = 0.0    # -1 to +1
        
        # Wind conditions
        self.wind_speed = 1.0    # m/s
        self.wind_direction = 0.0  # degrees (where wind goes to, simulator convention: 0°=East, 90°=North)
        
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
    
    def __init__(self, mode='local', map_name=None, test_heading=None, force_mock=False, debug=False):
        super().__init__('argo_unified_simulator_bridge')
        self.mode = mode
        self.map_name = map_name  # Store map_name for initial state calculation
        self.test_heading = test_heading  # Test mode: override heading calculation
        self.force_mock = force_mock  # Force mock simulator even if real simulator is available
        
        self.get_logger().info(f'Argo Unified Simulator Bridge starting in {mode.upper()} mode...')
        
        # --- Debug Tracing (declare early so it's available during initialization) ---
        # Enable position tracing to debug oscillation issues
        # CLI --debug option takes precedence over ROS2 parameter
        # If debug is set via CLI, use it directly; otherwise check ROS2 parameter
        if debug:
            self.debug_position_trace = True
        else:
            self.debug_position_trace = self.declare_parameter('debug_position_trace', False).get_parameter_value().bool_value
        
        self.position_trace_counter = 0  # Counter for log sequence numbers
        
        # Note: Position tracing uses INFO level messages, so no need to change logger level
        # This avoids the performance impact of DEBUG-level logging from ROS2 and other nodes
        if self.debug_position_trace:
            self.get_logger().info('🔍 Position trace debugging ENABLED - verbose logging active')
        
        # --- GPS Base Location (for NavSatFix) ---
        # Load from map GeoJSON if specified, otherwise use default
        self.base_latitude = 47.3769  # Default: Zurich, Switzerland
        self.base_longitude = 8.5417
        if map_name:
            self._load_map_home_location(map_name)
        
        # --- Load initial wind_direction from playground.json before simulator creation ---
        # This allows us to use it as the default parameter value
        initial_wind_direction = self._load_initial_wind_direction()
        
        # --- Declare wind_direction parameter (can be changed dynamically via Foxglove) ---
        # Declare it early so it's available throughout initialization
        self.declare_parameter('simulation.wind_direction', float(initial_wind_direction))
        self.wind_direction = self.get_parameter('simulation.wind_direction').get_parameter_value().double_value
        # Normalize to 0-360 range
        self.wind_direction = self.wind_direction % 360.0
        
        # Initialize simulator based on mode
        if mode == 'local':
            self._init_local_simulator()
        else:  # remote mode
            self._init_remote_simulator()
        
        # Common state
        self.last_control_time = time.time()
        self.simulation_running = True
        self.human_controlled = True  # Start in human control
        self.mock_human_input = False  # Disable mock input - use real control
        self.human_input_time = 0.0
        
        # External control state (for Foxglove Teleop, etc.)
        self.external_rudder = 0.0  # -1 to +1
        self.external_sail = 0.0    # -1 to +1
        self.last_external_control_time = 0.0
        
        # Initialize stored control angles (in degrees) for real simulator
        # These are used when robot control is active but no external control is received
        self.last_rudder_angle = 0.0  # degrees (-30 to +30)
        self.last_sail_angle = 0.0    # degrees (-45 to +45)
        
        # Remote mode specific state
        if mode == 'remote':
            self.last_remote_data_time = time.time()
            self.remote_connected = False
            self.control_mode = True
        
        # --- Publishers (Simulator → Argo) ---
        # IMU/Compass data
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        
        # GPS data
        self.pub_gps_cog = self.create_publisher(Float64, '/gps_cog', 10)
        self.pub_gps_sog = self.create_publisher(Float64, '/gps_sog', 10)
        self.pub_gps_velocity = self.create_publisher(Vector3, '/gps_velocity', 10)
        self.pub_gps_satellites = self.create_publisher(UInt8, '/gps_num_satellites', 10)
        self.pub_gps_fix = self.create_publisher(NavSatFix, '/fix', 10)
        self.pub_gps_data = self.create_publisher(String, '/gps_data', 10)

        # Wind data
        self.pub_wind = self.create_publisher(Vector3, '/anem_speed_angle_temp', 10)
        # True wind direction (absolute, in compass convention) for monitoring
        self.pub_true_wind = self.create_publisher(Float64, '/simulator/true_wind_direction', 10)
        
        # --- Subscribers (Argo → Simulator) ---
        # Control commands from Argo (final servo commands)
        self.create_subscription(Vector3, '/rudder_sail_servo', self.control_callback, 10)
        
        # External radio control input (from Foxglove Teleop, keyboard, or other sources)
        # This allows external tools to control the simulator
        # NOTE: We only subscribe, not publish, to avoid schema conflicts in Foxglove
        self.create_subscription(Vector3, '/rudder_sail_radio', self.radio_control_callback, 10)
        
        # Joystick input (from Foxglove Joystick panel or physical gamepad)
        # Converts Joy messages to rudder/sail control
        self.create_subscription(Joy, '/joy', self.joy_control_callback, 10)
        
        # Advertise /joy topic so Foxglove can publish to it
        # Publish a few initial messages to help foxglove_bridge learn the schema, then stop
        # This allows Foxglove to be the primary publisher without interference
        self.pub_joy_dummy = self.create_publisher(Joy, '/joy', 10)
        self.joy_schema_published = False
        self.joy_schema_publish_count = 0
        
        # Publish a few initial messages to teach foxglove_bridge the schema
        # Then stop so Foxglove can be the primary publisher
        def publish_schema_messages():
            if self.joy_schema_publish_count < 3:  # Publish only 3 messages total
                joy_msg = Joy()
                joy_msg.header.stamp = self.get_clock().now().to_msg()
                joy_msg.header.frame_id = 'joy'
                # Provide at least 2 axes for rudder/sail control
                joy_msg.axes = [0.0, 0.0, 0.0, 0.0]  # 4 axes total (some gamepads have more)
                joy_msg.buttons = [0, 0, 0, 0]  # 4 buttons (some gamepads have more)
                try:
                    self.pub_joy_dummy.publish(joy_msg)
                    self.joy_schema_publish_count += 1
                    if self.joy_schema_publish_count >= 3:
                        # Stop the timer after 3 messages
                        self.joy_schema_timer.cancel()
                        self.joy_schema_published = True
                        self.get_logger().info('Published /joy schema messages (3 total), stopped dummy publisher - Foxglove can now publish')
                except Exception as e:
                    # Publisher might not be ready yet, ignore
                    pass
        
        # Publish a few schema messages with short delays, then automatically stop
        # This teaches foxglove_bridge the message format without interfering with Foxglove publishing
        self.joy_schema_timer = self.create_timer(0.2, publish_schema_messages)  # Publish every 0.2s, auto-stops after 3 messages
        
        # Joystick axis mapping configuration
        # Default mapping: axis 0 = rudder (left/right), axis 1 = sail (up/down, inverted)
        self.declare_parameter('joy_rudder_axis', 0)  # Axis index for rudder control
        self.declare_parameter('joy_sail_axis', 1)    # Axis index for sail control
        self.declare_parameter('joy_rudder_invert', False)  # Invert rudder axis
        self.declare_parameter('joy_sail_invert', True)      # Invert sail axis (typically up=out, so invert)
        
        # Monitor control authority
        self.create_subscription(Bool, '/human_controlled', self.human_control_callback, 10)
        
        # Reset service for simulation state reset
        from std_srvs.srv import Trigger
        self.reset_service = self.create_service(Trigger, '/simulator/reset', self.reset_simulation_callback)
        
        # Store initial boat state for reset capability
        self.initial_boat_state = None
        self.initial_boat_heading = 0.0  # Will be set when initial state is calculated
        
        # --- Simulation Parameters ---
        # Read simulation rate from shared simulation parameters (argo.yaml)
        self.declare_parameter('simulation.simulation_rate', 10.0)
        self.simulation_rate = self.get_parameter('simulation.simulation_rate').get_parameter_value().double_value
        if self.simulation_rate <= 0:
            self.simulation_rate = 10.0
        
        self.get_logger().info(f"Simulation rate: {self.simulation_rate:.1f} Hz")
        self.get_logger().info(f"Initial wind direction: {self.wind_direction:.1f}° (loaded from playground.json, can be changed via parameter)")
        
        # Add parameter callback to handle runtime parameter changes (e.g., from Foxglove)
        self.add_on_set_parameters_callback(self._on_parameter_change)
        
        # --- Timers ---
        if mode == 'local':
            self.sim_timer = self.create_timer(1.0/self.simulation_rate, self.local_simulation_step)
        else:  # remote mode
            self.status_timer = self.create_timer(5.0, self.check_remote_connection)
            self.heartbeat_timer = self.create_timer(1.0, self.send_heartbeat)
        
        # Status timer
        self.status_timer = self.create_timer(10.0, self.print_status)
        
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
        
        # Initial state
        self.boat_state = None
    
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
    
    def _create_simulator(self, boat_heading, force_mock=False):
        """Create a simulator instance (real or mock).
        
        Args:
            boat_heading: Initial heading in degrees (0-360)
            force_mock: If True, force use of mock simulator even if real simulator is available
        
        Returns:
            tuple: (simulator_instance, use_mock_bool)
        """
        import numpy as np
        
        # Try real simulator if available and not forcing mock
        if SIMULATOR_AVAILABLE and not force_mock:
            try:
                self.get_logger().info('Creating sailboat-playground simulator...')
                
                # Configuration file paths
                boat_config = "simulator/customizations/sailboat-playground/boats/sample_boat.json"
                env_config = "simulator/customizations/sailboat-playground/environments/playground.json"
                
                # Check if configuration files exist
                if not os.path.exists(boat_config):
                    raise FileNotFoundError(f"Boat configuration file not found: {boat_config}")
                if not os.path.exists(env_config):
                    raise FileNotFoundError(f"Environment configuration file not found: {env_config}")
                
                # The simulator's data files (e.g., foil profiles) are relative to its
                # own directory. We must provide an absolute path to the 'foils'
                # directory to prevent FileNotFoundError when launched via ROS2.
                parent_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'simulator', 'sailboat-playground')
                foils_path = os.path.join(parent_path, 'foils')
                
                # Initial boat state: home location (0, 0) with specified heading
                boat_position = np.array([0.0, 0.0])
                
                # Initialize the sailboat-playground simulation manager with configs
                sim_manager = engine.Manager(
                    boat_config,
                    env_config,
                    foils_dir=foils_path,
                    boat_heading=boat_heading,
                    boat_position=boat_position
                )
                
                # Note: debug_position_trace may not be initialized yet, so check safely
                try:
                    if hasattr(self, 'debug_position_trace') and self.debug_position_trace:
                        self.get_logger().info(f"[POS_TRACE:CREATE] Real simulator created - sim_mgr_id={id(sim_manager)}, boat_id={id(sim_manager.boat)}")
                except AttributeError:
                    pass  # Not initialized yet, skip debug logging
                
                # Explicitly set position and velocity to zero after creation to ensure clean state
                try:
                    sim_manager.boat.set_position(boat_position)  # Ensure position is (0, 0)
                    sim_manager.boat._speed = np.array([0.0, 0.0])  # Zero velocity
                    sim_manager.boat._angular_speed = 0.0  # Zero angular velocity
                except Exception as e:
                    self.get_logger().warn(f"Could not explicitly set simulator position/velocity: {e}")
                
                self.get_logger().info(f'Real simulator created: position=(0.0, 0.0), heading={boat_heading:.1f}°')
                self.get_logger().info(f'Using boat config: {boat_config}')
                self.get_logger().info(f'Using environment config: {env_config}')
                
                return sim_manager, False
                
            except Exception as e:
                import traceback
                self.get_logger().warn(f'Failed to create real simulator: {e}')
                tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
                self.get_logger().warn(f'Stack trace:\n{tb_str}')
                self.get_logger().info('Falling back to mock simulator')
                # Fall through to mock simulator creation
        
        # Create mock simulator
        self.get_logger().info('Creating mock simulator...')
        simulator = MockSailboatSimulator()
        # Mock simulator initializes at (0, 0) with heading 0, so we need to set it
        if hasattr(simulator, 'boat_x'):
            simulator.boat_x = 0.0
            simulator.boat_y = 0.0
            simulator.boat_heading = boat_heading
            simulator.boat_speed = 0.0
        self.get_logger().info(f'Mock simulator created: position=(0.0, 0.0), heading={boat_heading:.1f}°')
        
        return simulator, True
    
    def _init_local_simulator(self):
        """Initialize local simulator (sailboat-playground or mock)."""
        # Calculate initial boat position and heading
        # Start at home location, pointing toward center of geofence
        if self.test_heading is not None:
            # Test mode: use specified heading
            boat_position = np.array([0.0, 0.0])
            boat_heading = float(self.test_heading)
            self.get_logger().info(f'TEST MODE: Using test heading {boat_heading:.1f}° instead of calculated heading')
        else:
            boat_position, boat_heading = self._calculate_initial_boat_state()
        
        # Store initial heading for reset functionality
        self.initial_boat_heading = boat_heading
        self.get_logger().info(f'Stored initial_boat_heading = {self.initial_boat_heading:.1f}° for reset functionality')
        self.get_logger().info(f'Initial boat state: position={boat_position}, heading={boat_heading:.1f}°')
        
        # Create simulator using shared method
        self.simulator, self.use_mock = self._create_simulator(boat_heading, force_mock=self.force_mock)
        
        if self.debug_position_trace:
            self.get_logger().info(f"[POS_TRACE:INIT] Simulator created - use_mock={self.use_mock}, sim_id={id(self.simulator)}")
        
        # For real simulator, also store as sim_manager
        if not self.use_mock:
            self.sim_manager = self.simulator
            if self.debug_position_trace:
                self.get_logger().info(f"[POS_TRACE:INIT] sim_manager set - sim_mgr_id={id(self.sim_manager)}")
        
        # Apply initial wind_direction parameter to simulator (after creation)
        # This ensures the parameter value (which may have been loaded from playground.json) is applied
        if hasattr(self, 'wind_direction'):
            self._apply_wind_direction_to_simulator(self.wind_direction)
    
    def _init_remote_simulator(self):
        """Initialize remote simulator connection."""
        self.get_logger().info('Remote simulator mode - waiting for connection...')
        # Remote simulator state will be managed by callbacks
    
    def local_simulation_step(self):
        """Main simulation step for local mode - updates physics and publishes sensor data."""
        try:
            self.position_trace_counter += 1
            trace_id = self.position_trace_counter
            
            if self.debug_position_trace:
                self.get_logger().info(f"[POS_TRACE:{trace_id}] === STEP START === sim_id={id(self.sim_manager) if hasattr(self, 'sim_manager') else 'N/A'}")
            
            # Generate mock human input for testing (optional)
            if self.mock_human_input and self.human_controlled:
                self.publish_mock_human_input()
            
            # Determine which control source to use
            # Priority: External control (Teleop/keyboard) > Robot control
            if self.human_controlled and (time.time() - self.last_external_control_time) < 2.0:
                # Use external control if recent
                current_rudder = self.external_rudder
                current_sail = self.external_sail
                control_source = "external"
            else:
                # Use stored robot control values (from control_callback)
                # Convert from degrees back to normalized (-1 to +1)
                current_rudder = self.last_rudder_angle / 30.0  # Convert back from degrees
                current_sail = self.last_sail_angle / 45.0     # Convert back from degrees
                control_source = "robot"
            
            if self.debug_position_trace:
                self.get_logger().info(f"[POS_TRACE:{trace_id}] Control: source={control_source}, rudder={current_rudder:.3f}, sail={current_sail:.3f}")
            
            # Update simulator physics
            if self.use_mock:
                # Mock simulator is recreated on reset, so it starts fresh at (0,0)
                if self.debug_position_trace:
                    old_pos = (self.boat_state.get('x', 0), self.boat_state.get('y', 0)) if self.boat_state else (0, 0)
                    # Convert numpy types to float for logging
                    old_pos = (float(old_pos[0]), float(old_pos[1]))
                    self.get_logger().info(f"[POS_TRACE:{trace_id}] MOCK: Before step - pos={old_pos}")
                
                # Get new state from simulator (returns new dict each time)
                new_state = self.simulator.step()
                
                # Convert numpy types to Python floats for consistency
                self.boat_state = {
                    'x': float(new_state['x']),
                    'y': float(new_state['y']),
                    'heading': float(new_state['heading']),
                    'speed': float(new_state['speed']),
                    'wind_speed': float(new_state['wind_speed']),
                    'wind_direction': float(new_state['wind_direction']),
                    'rudder': float(new_state['rudder']),
                    'sail': float(new_state['sail'])
                }
                
                if self.debug_position_trace:
                    new_pos = (self.boat_state['x'], self.boat_state['y'])
                    self.get_logger().info(f"[POS_TRACE:{trace_id}] MOCK: After step - pos={new_pos}, boat_state_id={id(self.boat_state)}")
            else:
                # Handle real sailboat-playground API
                try:
                    # Convert normalized values to degrees for sailboat-playground
                    rudder_angle = current_rudder * 30.0  # Convert to degrees (-30 to +30)
                    sail_angle = current_sail * 45.0      # Convert to degrees (-45 to +45)
                    
                    # Store for reference
                    self.last_rudder_angle = rudder_angle
                    self.last_sail_angle = sail_angle
                    
                    if self.debug_position_trace:
                        old_pos = (self.boat_state.get('x', 0), self.boat_state.get('y', 0)) if self.boat_state else (0, 0)
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] REAL: Before step - pos={old_pos}, sim_mgr_id={id(self.sim_manager)}, rudder={rudder_angle:.1f}°, sail={sail_angle:.1f}°")
                    
                    # Step the simulation with control inputs
                    # Simulator is recreated on reset, so it starts fresh at (0,0) with correct heading
                    # CRITICAL: sailboat-playground requires integer angles for foil lookup tables
                    self.sim_manager.step([int(sail_angle), int(rudder_angle)])
                    
                    # Get the current state from sailboat-playground
                    # Use 'state' (complete state) instead of 'agent_state' to get speed and rudder_angle
                    full_state = self.sim_manager.state
                    agent_state = self.sim_manager.agent_state
                    
                    # Extract position and heading first (needed for debug logging)
                    position = agent_state.get('position', [0.0, 0.0])
                    # Convert numpy array to Python list/float to ensure we have copies, not references
                    if isinstance(position, np.ndarray):
                        position = position.copy().tolist()
                    position_x = float(position[0])
                    position_y = float(position[1])
                    heading_to_use = float(agent_state.get('heading', 0.0))
                    
                    if self.debug_position_trace:
                        raw_pos = agent_state.get('position', [0.0, 0.0])
                        raw_pos_type = type(raw_pos).__name__
                        raw_pos_id = id(raw_pos) if hasattr(raw_pos, '__array__') else 'N/A'
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] REAL: After step - raw_pos={raw_pos}, type={raw_pos_type}, id={raw_pos_id}")
                        # Verify rudder was applied
                        applied_rudder = full_state.get('rudder_angle', 0.0)
                        applied_sail = full_state.get('sail_angle', 0.0)
                        # Get angular speed and heading change info from boat state
                        boat_heading = full_state.get('boat_heading', heading_to_use)
                        # Try to get angular speed from boat object if available
                        angular_speed = None
                        try:
                            if hasattr(self.sim_manager, 'boat') and hasattr(self.sim_manager.boat, 'angular_speed'):
                                angular_speed = self.sim_manager.boat.angular_speed
                        except:
                            pass
                        angular_info = f", angular_speed={angular_speed:.3f}°/s" if angular_speed is not None else ""
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] REAL: Applied controls - rudder={applied_rudder:.1f}°, sail={applied_sail:.1f}° (requested: {rudder_angle:.1f}°, {sail_angle:.1f}°), heading={boat_heading:.1f}°{angular_info}")
                    
                    if self.debug_position_trace:
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] REAL: Extracted - pos_x={position_x:.6f}, pos_y={position_y:.6f}, pos_id={id(position)}")
                    
                    # Calculate speed from boat_speed vector (2D velocity) in full_state
                    # boat_speed is a 2D velocity vector [vx, vy], need to calculate magnitude
                    boat_speed_vector = full_state.get('boat_speed', None)
                    if boat_speed_vector is not None:
                        if isinstance(boat_speed_vector, np.ndarray):
                            boat_speed_vector = boat_speed_vector.copy()
                        speed_to_use = float(np.linalg.norm(boat_speed_vector))
                    else:
                        speed_to_use = 0.0
                    
                    if self.debug_position_trace:
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] REAL: Speed calculation - boat_speed_vector={boat_speed_vector}, speed={speed_to_use:.3f} m/s")
                    
                    # Convert to our internal boat_state format
                    self.boat_state = {
                        'x': position_x,
                        'y': position_y, 
                        'heading': heading_to_use,
                        'speed': speed_to_use,
                        'wind_speed': float(agent_state.get('wind_speed', 8.0)),
                        'wind_direction': float(agent_state.get('wind_direction', 45.0)),
                        'rudder': current_rudder,  # Store normalized values
                        'sail': current_sail
                    }
                    
                    if self.debug_position_trace:
                        stored_pos = (self.boat_state['x'], self.boat_state['y'])
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] REAL: Stored - pos={stored_pos}, boat_state_id={id(self.boat_state)}")
                except Exception as e:
                    self.get_logger().warn(f'Simulator step failed: {e}')
                    if self.debug_position_trace:
                        self.get_logger().info(f"[POS_TRACE:{trace_id}] ERROR: {e}")
                    return
            
            # Publish sensor data to Argo
            if self.debug_position_trace:
                pub_pos = (float(self.boat_state.get('x', 0)), float(self.boat_state.get('y', 0)))
                self.get_logger().info(f"[POS_TRACE:{trace_id}] About to publish sensor data - pos={pub_pos}")
            
            self.publish_sensor_data(trace_id=trace_id)
            
            if self.debug_position_trace:
                self.get_logger().info(f"[POS_TRACE:{trace_id}] === STEP END ===")
            
        except Exception as e:
            self.get_logger().error(f'Simulation step error: {e}')
    
    def publish_sensor_data(self, trace_id=None):
        """Publish simulated sensor data to Argo topics.
        
        Args:
            trace_id: Optional simulation step ID for debug tracing
        """
        if not self.boat_state:
            return
        
        # Convert boat heading from simulator convention (0° = East) to compass convention (0° = North)
        # Simulator: 0° = East, 90° = North, 180° = West, 270° = South
        # Compass: 0° = North, 90° = East, 180° = South, 270° = West
        # Conversion: compass_heading = (90 - simulator_heading) % 360
        heading_simulator = self.boat_state['heading']  # Heading in simulator convention
        heading_compass = (90.0 - heading_simulator) % 360.0  # Convert to compass convention
        
        # IMU/Compass data (heading in degrees, compass convention)
        pose_msg = Vector3(x=0.0, y=0.0, z=heading_compass)
        self.pub_pose.publish(pose_msg)
        
        compass_msg = Vector3(x=0.0, y=0.0, z=heading_compass)
        self.pub_compass.publish(compass_msg)
        
        # GPS data (compass convention)
        gps_cog_msg = Float64(data=heading_compass)  # Course over ground
        self.pub_gps_cog.publish(gps_cog_msg)
        
        # Speed over ground (convert m/s to knots)
        speed_knots = self.boat_state['speed'] * 1.94384  # m/s to knots
        gps_sog_msg = Float64(data=speed_knots)
        self.pub_gps_sog.publish(gps_sog_msg)
        
        # GPS velocity vector (north, east, speed)
        # Use compass heading for velocity calculation
        heading_compass_rad = math.radians(heading_compass)
        vel_north = self.boat_state['speed'] * math.cos(heading_compass_rad) * 1.94384  # knots
        vel_east = self.boat_state['speed'] * math.sin(heading_compass_rad) * 1.94384   # knots
        gps_vel_msg = Vector3(x=vel_north, y=vel_east, z=speed_knots)
        self.pub_gps_velocity.publish(gps_vel_msg)
        
        # Wind data (speed, angle relative to boat, temperature)
        wind_msg = Vector3(
            x=self.boat_state['wind_speed'],     # m/s
            y=self.boat_state['wind_direction'], # degrees relative to boat
            z=22.5                               # temperature (mock)
        )
        self.pub_wind.publish(wind_msg)
        
        # Publish true wind direction (absolute, in compass convention) for monitoring
        # Get true wind direction from simulator's Environment
        true_wind_direction_compass = None
        if self.mode == 'local' and not self.use_mock:
            # Real simulator: get from Environment config
            if hasattr(self, 'sim_manager') and self.sim_manager is not None:
                if hasattr(self.sim_manager, 'environment') and self.sim_manager.environment is not None:
                    env = self.sim_manager.environment
                    if hasattr(env, '_config'):
                        true_wind_dir_simulator = env._config.get('wind_direction', 0.0)
                        # Convert from simulator convention (where wind goes to) to compass convention (where wind comes from)
                        # Step 1: Convert convention: "going to" to "coming from" (subtract 180°)
                        true_wind_dir_temp = (true_wind_dir_simulator - 180.0) % 360.0
                        # Step 2: Convert coordinate system: simulator to compass
                        true_wind_direction_compass = (90.0 - true_wind_dir_temp) % 360.0
        elif self.mode == 'local' and self.use_mock:
            # Mock simulator: convert the wind_direction (which is in simulator convention, "going to")
            if hasattr(self, 'simulator') and self.simulator is not None:
                true_wind_dir_simulator = getattr(self.simulator, 'wind_direction', 0.0)
                # Convert from simulator convention (where wind goes to) to compass convention (where wind comes from)
                # Step 1: Convert convention: "going to" to "coming from" (subtract 180°)
                true_wind_dir_temp = (true_wind_dir_simulator - 180.0) % 360.0
                # Step 2: Convert coordinate system: simulator to compass
                true_wind_direction_compass = (90.0 - true_wind_dir_temp) % 360.0
        
        if true_wind_direction_compass is not None:
            true_wind_msg = Float64(data=true_wind_direction_compass)
            self.pub_true_wind.publish(true_wind_msg)

        # Publish satellite count
        sat_msg = UInt8()
        sat_msg.data = 12  # Mock satellite count
        self.pub_gps_satellites.publish(sat_msg)

        # Publish NavSatFix message for mapping
        self.publish_navsat_fix(trace_id=trace_id)

        # Publish mock NMEA RMC sentence
        self.publish_mock_nmea()

    def publish_mock_human_input(self):
        """Generate mock human radio input for testing (deprecated - use external control instead)."""
        # This function is kept for backwards compatibility but mock input is disabled
        # External control via /rudder_sail_radio topic should be used instead
        pass

    def _load_map_home_location(self, map_name):
        """Load the 'home' waypoint from a GeoJSON map file to set the start location."""
        try:
            # Determine maps directory (foxglove/maps contains GeoJSON files)
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[1]  # nodes -> argo
            maps_dir = argo_dir / "foxglove" / "maps"
            geojson_path = maps_dir / f"{map_name}.geojson"
            
            if not geojson_path.exists():
                self.get_logger().warn(f"Map file not found: {geojson_path}, using default location")
                return
            
            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            # Find the "home" waypoint
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                if props.get('name') == 'home' and props.get('type') == 'waypoint':
                    coords = feature['geometry']['coordinates']
                    # GeoJSON format: [longitude, latitude, altitude]
                    self.base_longitude = coords[0]
                    self.base_latitude = coords[1]
                    self.get_logger().info(f"Loaded start home waypoint location from map '{map_name}': "
                                         f"lat={self.base_latitude:.6f}, lon={self.base_longitude:.6f}")
                    return
            
            # If no "home" waypoint found, try to calculate center of sailing area
            self.get_logger().warn(f"No 'home' waypoint found in map '{map_name}', calculating center")
            self._calculate_map_center(geojson_data)
            
        except Exception as e:
            self.get_logger().error(f"Failed to load map '{map_name}': {e}, using default location")
    
    def _calculate_map_center(self, geojson_data):
        """Calculate the center of all coordinates in the map as fallback."""
        all_lons = []
        all_lats = []
        
        for feature in geojson_data.get('features', []):
            geom = feature.get('geometry', {})
            coords = geom.get('coordinates', [])
            
            if geom.get('type') == 'Point':
                all_lons.append(coords[0])
                all_lats.append(coords[1])
            elif geom.get('type') == 'LineString':
                for coord in coords:
                    all_lons.append(coord[0])
                    all_lats.append(coord[1])
            elif geom.get('type') == 'Polygon':
                for ring in coords:
                    for coord in ring:
                        all_lons.append(coord[0])
                        all_lats.append(coord[1])
        
        if all_lons and all_lats:
            self.base_longitude = sum(all_lons) / len(all_lons)
            self.base_latitude = sum(all_lats) / len(all_lats)
            self.get_logger().info(f"Calculated map center: lat={self.base_latitude:.6f}, lon={self.base_longitude:.6f}")
    
    def _calculate_initial_boat_state(self):
        """Calculate initial boat position and heading from map data.
        
        Returns:
            tuple: (boat_position, boat_heading)
            - boat_position: numpy array [x, y] in meters (home location at origin = [0, 0])
            - boat_heading: float, heading in degrees (0-360) pointing toward geofence center
        """
        import numpy as np
        
        # Home location is at origin (0, 0) in local coordinates
        # This is the base_latitude/base_longitude already loaded
        home_x = 0.0
        home_y = 0.0
        
        # Calculate center of geofence area (sailing area boundaries)
        geofence_center_x, geofence_center_y = self._calculate_geofence_center()
        
        self.get_logger().info(f"Geofence center calculation result: x={geofence_center_x}, y={geofence_center_y}")
        
        # Calculate heading from home toward geofence center
        if geofence_center_x is not None and geofence_center_y is not None and not (abs(geofence_center_x) < 0.1 and abs(geofence_center_y) < 0.1):
            # Calculate heading in degrees (0-360, where 0° = North, 90° = East)
            # In local XY coordinates: +X = East, +Y = North
            dx = geofence_center_x - home_x
            dy = geofence_center_y - home_y
            
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                # If center is very close to home, default to North (0°)
                boat_heading = 0.0
                self.get_logger().warn("Geofence center is very close to home, using default heading 0° (North)")
            else:
                # atan2(y, x) gives angle from +X axis, but we need compass heading
                # Compass heading: 0° = North (+Y), 90° = East (+X), 180° = South (-Y), 270° = West (-X)
                # In local XY: +X = East, +Y = North
                # atan2(dy, dx) gives angle from +X axis (East), measured counterclockwise
                # To convert to compass heading (0° = North), we need to rotate by 90° clockwise
                # So: compass_heading = 90° - atan2(dy, dx)
                # But we want the angle from North, so: heading = 90° - atan2(dy, dx)
                # However, atan2 gives positive angles counterclockwise from +X, so:
                # - If dx>0, dy>0 (NE quadrant): atan2 positive, heading should be 0-90°
                # - If dx<0, dy>0 (NW quadrant): atan2 positive >90°, heading should be 90-180°
                # - If dx<0, dy<0 (SW quadrant): atan2 negative, heading should be 180-270°
                # - If dx>0, dy<0 (SE quadrant): atan2 negative, heading should be 270-360°
                # Formula: heading = 90° - atan2(dy, dx), then normalize to 0-360°
                angle_rad = math.atan2(dy, dx)
                # atan2 gives angle from +X (East), we want angle from +Y (North)
                # North is 90° clockwise from East, so: heading = 90° - atan2_angle
                boat_heading = 90.0 - math.degrees(angle_rad)
                boat_heading = boat_heading % 360.0  # Normalize to 0-360
                
                self.get_logger().info(f"Geofence center: ({geofence_center_x:.1f}, {geofence_center_y:.1f}) m from home")
                self.get_logger().info(f"Delta from home: dx={dx:.1f}m (East), dy={dy:.1f}m (North)")
                self.get_logger().info(f"Calculated heading: {boat_heading:.1f}° (toward geofence center)")
        else:
            # No geofence found, default to North (0°)
            boat_heading = 0.0
            self.get_logger().warn("No geofence found, using default heading 0° (North)")
        
        # Boat starts at home location (origin)
        boat_position = np.array([home_x, home_y])
        
        return boat_position, boat_heading
    
    def _calculate_geofence_center(self):
        """Calculate the center of the sailing area geofence from GeoJSON map data.
        
        Returns:
            tuple: (center_x, center_y) in meters from home location, or (None, None) if not found
        """
        try:
            # Get map name from stored attribute or use default
            map_name = getattr(self, 'map_name', None)
            if not map_name:
                self.get_logger().warn("No map_name available, cannot calculate geofence center")
                self.get_logger().warn(f"Available attributes: map_name={getattr(self, 'map_name', 'NOT SET')}")
                return None, None
            
            # Load GeoJSON file
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[1]  # nodes -> argo
            maps_dir = argo_dir / "foxglove" / "maps"
            geojson_path = maps_dir / f"{map_name}.geojson"
            
            if not geojson_path.exists():
                self.get_logger().warn(f"Map file not found: {geojson_path}")
                return None, None
            
            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            # Find all sailing area boundaries (Polygon features with type "sailing_area")
            all_coords = []
            feature_types_found = {}
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                geom = feature.get('geometry', {})
                geom_type = geom.get('type', 'unknown')
                prop_type = props.get('type', 'unknown')
                
                # Track what we find
                key = f"{geom_type} ({prop_type})"
                feature_types_found[key] = feature_types_found.get(key, 0) + 1
                
                # Look for sailing area boundaries (Polygon type)
                if geom_type == 'Polygon' and prop_type == 'sailing_area':
                    coords = geom.get('coordinates', [])
                    if coords:
                        # Get outer ring (first ring in coordinates)
                        outer_ring = coords[0]
                        for coord in outer_ring:
                            all_coords.append(coord)
                        self.get_logger().info(f"Found sailing_area Polygon with {len(outer_ring)} coordinates")
            
            # If no sailing_area polygons found, try LineString boundaries
            if not all_coords:
                self.get_logger().info(f"No sailing_area Polygons found, trying LineString boundaries...")
                for feature in geojson_data.get('features', []):
                    props = feature.get('properties', {})
                    geom = feature.get('geometry', {})
                    
                    if geom.get('type') == 'LineString' and props.get('type') == 'sailing_boundary':
                        coords = geom.get('coordinates', [])
                        for coord in coords:
                            all_coords.append(coord)
                        self.get_logger().info(f"Found sailing_boundary LineString with {len(coords)} coordinates")
            
            # Log what we found
            if feature_types_found:
                self.get_logger().info(f"Feature types in map: {feature_types_found}")
            
            if not all_coords:
                self.get_logger().warn("No sailing area boundaries found in map")
                self.get_logger().warn(f"Checked for: Polygon (sailing_area) and LineString (sailing_boundary)")
                return None, None
            
            # Calculate center of all boundary coordinates
            all_lons = [coord[0] for coord in all_coords]
            all_lats = [coord[1] for coord in all_coords]
            
            center_lon = sum(all_lons) / len(all_lons)
            center_lat = sum(all_lats) / len(all_lats)
            
            self.get_logger().info(f"Geofence boundary coordinates: {len(all_coords)} points")
            self.get_logger().info(f"Geofence center (lat/lon): {center_lat:.6f}°, {center_lon:.6f}°")
            self.get_logger().info(f"Home location (lat/lon): {self.base_latitude:.6f}°, {self.base_longitude:.6f}°")
            
            # Convert center lat/lon to local XY coordinates (meters from home)
            center_x, center_y = self.lonlat_to_xy(center_lon, center_lat)
            
            # Calculate the actual heading from home to center for verification
            if abs(center_x) > 0.1 or abs(center_y) > 0.1:
                dx = center_x
                dy = center_y
                angle_rad = math.atan2(dy, dx)
                calculated_heading = 90.0 - math.degrees(angle_rad)
                calculated_heading = calculated_heading % 360.0
                self.get_logger().info(f"Geofence center relative to home: dx={dx:.1f}m (East), dy={dy:.1f}m (North)")
                self.get_logger().info(f"Verification: heading from home to center = {calculated_heading:.1f}°")
            
            return center_x, center_y
            
        except Exception as e:
            self.get_logger().error(f"Error calculating geofence center: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None, None
    
    def lonlat_to_xy(self, lon, lat):
        """Convert longitude/latitude to local x/y meters from base location.
        
        Args:
            lon: Longitude in degrees
            lat: Latitude in degrees
            
        Returns:
            tuple: (x, y) in meters from base location
            - x: East-West component (positive = East)
            - y: North-South component (positive = North)
        """
        R = 6378137.0  # Earth radius in meters
        x = (math.radians(lon) - math.radians(self.base_longitude)) * R * math.cos(math.radians(self.base_latitude))
        y = (math.radians(lat) - math.radians(self.base_latitude)) * R
        return x, y
    
    def xy_to_latlon(self, x, y):
        """Convert XY meters from base lat/lon to new lat/lon."""
        R = 6378137.0  # Earth radius in meters
        dLat = y / R
        dLon = x / (R * math.cos(math.pi * self.base_latitude / 180))
        lat = self.base_latitude + dLat * 180 / math.pi
        lon = self.base_longitude + dLon * 180 / math.pi
        return lat, lon

    def reset_simulation_callback(self, request, response):
        """Reset simulation to initial state by recreating the simulator."""
        try:
            self.get_logger().info("Resetting simulation to initial state (recreating simulator)...")
            
            # Get initial heading (calculated at startup, pointing toward geofence center)
            initial_heading = getattr(self, 'initial_boat_heading', 0.0)
            
            # If heading is 0.0, try to recalculate it (may have failed during initialization)
            if initial_heading == 0.0 or abs(initial_heading) < 0.1:
                self.get_logger().warn(f"⚠️ Initial heading is {initial_heading:.1f}° - attempting to recalculate from geofence...")
                try:
                    _, recalculated_heading = self._calculate_initial_boat_state()
                    if recalculated_heading != 0.0 and abs(recalculated_heading) > 0.1:
                        initial_heading = recalculated_heading
                        self.initial_boat_heading = recalculated_heading
                        self.get_logger().info(f"✅ Recalculated initial heading: {initial_heading:.1f}°")
                    else:
                        self.get_logger().error(f"⚠️ Recalculation also returned 0.0° - geofence calculation may have failed")
                except Exception as e:
                    self.get_logger().error(f"⚠️ Failed to recalculate initial heading: {e}")
            
            self.get_logger().info(f"Recreating simulator with initial heading: {initial_heading:.1f}°")
            
            # Preserve wind settings from current simulator before recreation
            wind_speed = 8.0
            wind_direction = 45.0
            if hasattr(self, 'simulator'):
                if hasattr(self.simulator, 'wind_speed'):
                    wind_speed = self.simulator.wind_speed
                if hasattr(self.simulator, 'wind_direction'):
                    wind_direction = self.simulator.wind_direction
                elif hasattr(self, 'boat_state') and self.boat_state:
                    # Try to get from boat_state if simulator doesn't have it
                    wind_speed = self.boat_state.get('wind_speed', 8.0)
                    wind_direction = self.boat_state.get('wind_direction', 45.0)
            
            # Recreate the simulator (cleaner than trying to reset state)
            # Only recreate if we're in local mode
            if self.mode == 'local':
                # Use shared method to create simulator
                self.simulator, self.use_mock = self._create_simulator(initial_heading, force_mock=self.force_mock)
                
                # For real simulator, also store as sim_manager
                if not self.use_mock:
                    self.sim_manager = self.simulator
                    # Verify the simulator's actual position after creation
                    try:
                        state = self.sim_manager.agent_state
                        actual_pos = state.get('position', [0.0, 0.0])
                        # Convert numpy array to list if needed
                        if isinstance(actual_pos, np.ndarray):
                            actual_pos = actual_pos.copy().tolist()
                        actual_heading = state.get('heading', initial_heading)
                        self.get_logger().info(f"Simulator agent_state after creation: position=({actual_pos[0]:.2f}, {actual_pos[1]:.2f}), heading={actual_heading:.1f}°")
                    except Exception as e:
                        self.get_logger().warn(f"Could not verify simulator position: {e}")
                
                self.get_logger().info(f"✅ Simulator recreated: position=(0.0, 0.0), heading={initial_heading:.1f}° ({'real' if not self.use_mock else 'mock'})")
                
                # Apply current wind_direction parameter value to recreated simulator
                # This ensures user-set wind direction is preserved after reset
                if hasattr(self, 'wind_direction'):
                    self._apply_wind_direction_to_simulator(self.wind_direction)
                    # Update wind_direction variable used below to match parameter
                    wind_direction = self.wind_direction
            
            # Get actual boat_state from simulator after recreation (simulator starts at (0,0) with correct heading)
            if self.mode == 'local' and hasattr(self, 'simulator'):
                if self.use_mock:
                    # Mock simulator state
                    self.boat_state = {
                        'x': self.simulator.boat_x,
                        'y': self.simulator.boat_y,
                        'heading': self.simulator.boat_heading,
                        'speed': self.simulator.boat_speed,
                        'wind_speed': wind_speed,
                        'wind_direction': wind_direction,
                        'rudder': 0.0,
                        'sail': 0.0
                    }
                else:
                    # Real simulator state
                    try:
                        full_state = self.sim_manager.state
                        agent_state = self.sim_manager.agent_state
                        # Extract position - ensure we copy numpy array values to avoid reference issues
                        position = agent_state.get('position', [0.0, 0.0])
                        if isinstance(position, np.ndarray):
                            position = position.copy().tolist()
                        position_x = float(position[0])
                        position_y = float(position[1])
                        
                        # Calculate speed from boat_speed vector (2D velocity) in full_state
                        boat_speed_vector = full_state.get('boat_speed', None)
                        if boat_speed_vector is not None:
                            if isinstance(boat_speed_vector, np.ndarray):
                                boat_speed_vector = boat_speed_vector.copy()
                            speed = float(np.linalg.norm(boat_speed_vector))
                        else:
                            speed = 0.0
                        
                        self.boat_state = {
                            'x': position_x,
                            'y': position_y,
                            'heading': float(agent_state.get('heading', initial_heading)),
                            'speed': speed,
                            'wind_speed': float(agent_state.get('wind_speed', wind_speed)),
                            'wind_direction': float(agent_state.get('wind_direction', wind_direction)),
                            'rudder': 0.0,
                            'sail': 0.0
                        }
                    except Exception as e:
                        self.get_logger().warn(f"Could not read simulator state after recreation: {e}, using defaults")
                        self.boat_state = {
                            'x': 0.0,
                            'y': 0.0,
                            'heading': initial_heading,
                            'speed': 0.0,
                            'wind_speed': wind_speed,
                            'wind_direction': wind_direction,
                            'rudder': 0.0,
                            'sail': 0.0
                        }
            else:
                # Fallback if simulator not available
                self.boat_state = {
                    'x': 0.0,
                    'y': 0.0,
                    'heading': initial_heading,
                    'speed': 0.0,
                    'wind_speed': wind_speed,
                    'wind_direction': wind_direction,
                    'rudder': 0.0,
                    'sail': 0.0
                }
            
            self.get_logger().info(f"boat_state after reset: position=({self.boat_state['x']:.2f}, {self.boat_state['y']:.2f}), heading={self.boat_state['heading']:.1f}°")
            
            # Update initial_boat_state for reference
            self.initial_boat_state = self.boat_state.copy()
            
            # Reset control state
            self.last_control_time = time.time()
            self.human_controlled = True
            
            response.success = True
            response.message = "Simulation reset to initial state (home waypoint)"
            self.get_logger().info("✅ Simulation reset successful (simulator recreated)")
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error resetting simulation: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            response.success = False
            response.message = f"Reset failed: {str(e)}"
            return response
    
    def _load_initial_wind_direction(self):
        """Load initial wind_direction from playground.json configuration file.
        
        Returns:
            float: Initial wind direction in degrees (0-360) in COMPASS convention (0° = North), or 0.0 if not found
            This represents where the wind is COMING FROM (nautical convention).
            
        Note:
            The playground.json file uses simulator convention (0° = East, 90° = North) and represents
            wind direction as where the wind is GOING TO (opposite of nautical convention).
            
            Conversion requires two steps:
            1. Convention: "going to" to "coming from": subtract 180°
            2. Coordinate system: simulator (0°=E) to compass (0°=N): (90 - simulator) % 360
            
            Combined: compass = (90 - simulator + 180) % 360 = (270 - simulator) % 360
        """
        try:
            env_config = "simulator/customizations/sailboat-playground/environments/playground.json"
            if os.path.exists(env_config):
                with open(env_config, 'r') as f:
                    config = json.load(f)
                    wind_dir_simulator = config.get('wind_direction', 0.0)
                    # Convert from simulator convention (where wind goes to) to compass convention (where wind comes from)
                    # Step 1: Convert convention: "going to" to "coming from" (subtract 180°)
                    wind_dir_compass_temp = (wind_dir_simulator - 180.0) % 360.0
                    # Step 2: Convert coordinate system: simulator to compass
                    wind_dir_compass = (90.0 - wind_dir_compass_temp) % 360.0
                    self.get_logger().info(
                        f"Loaded initial wind_direction from {env_config}: "
                        f"{wind_dir_simulator:.1f}° (simulator) = {wind_dir_compass:.1f}° (compass)"
                    )
                    return float(wind_dir_compass)
            else:
                self.get_logger().warn(f"Environment config file not found: {env_config}, using default wind_direction=0.0°")
                return 0.0
        except Exception as e:
            self.get_logger().warn(f"Failed to load wind_direction from playground.json: {e}, using default 0.0°")
            return 0.0
    
    def _apply_wind_direction_to_simulator(self, wind_direction_deg):
        """Apply wind direction change to the simulator's environment.
        
        Args:
            wind_direction_deg: Wind direction in degrees (0-360) in COMPASS convention (0° = North)
            This represents where the wind is COMING FROM (nautical convention).
            
        Note:
            The simulator uses trigonometric convention (0° = East, 90° = North) and represents
            wind direction as where the wind is GOING TO (opposite of nautical convention).
            
            Conversion requires two steps:
            1. Coordinate system: compass (0°=N) to simulator (0°=E): (90 - compass) % 360
            2. Convention: "coming from" to "going to": add 180°
            
            Combined: simulator = (270 - compass) % 360
            Examples: 
              - Compass 0° (wind from N, blowing S) -> Sim 270° (wind going S)
              - Compass 90° (wind from E, blowing W) -> Sim 180° (wind going W)
        """
        # Normalize to 0-360 range (compass convention: 0° = North, where wind comes from)
        wind_direction_deg = wind_direction_deg % 360.0
        
        # Convert from compass convention (0° = North, where wind comes from) to simulator convention
        # Compass: 0° = North (wind from N), 90° = East (wind from E), 180° = South (wind from S), 270° = West (wind from W)
        # Simulator: 0° = East (wind going E), 90° = North (wind going N), 180° = West (wind going W), 270° = South (wind going S)
        # Step 1: Convert coordinate system: compass to simulator
        simulator_wind_direction = (90.0 - wind_direction_deg) % 360.0
        # Step 2: Convert convention: "coming from" (nautical) to "going to" (sailboat-playground)
        simulator_wind_direction = (simulator_wind_direction + 180.0) % 360.0
        
        if self.mode == 'local' and not self.use_mock:
            # Update real simulator's environment config
            if hasattr(self, 'sim_manager') and self.sim_manager is not None:
                try:
                    # Access the environment's config dictionary directly
                    if hasattr(self.sim_manager, 'environment') and self.sim_manager.environment is not None:
                        env = self.sim_manager.environment
                        if hasattr(env, '_config'):
                            old_wind_dir_simulator = env._config.get('wind_direction', 0.0)
                            # Convert old simulator angle back to compass for logging
                            # Reverse conversion: first convert "going to" to "coming from" (subtract 180°)
                            # Then convert coordinate system: simulator to compass
                            old_wind_dir_compass = ((90.0 - old_wind_dir_simulator + 180.0) % 360.0)
                            env._config['wind_direction'] = float(simulator_wind_direction)
                            self.get_logger().info(
                                f"Wind direction changed from {old_wind_dir_compass:.1f}° to {wind_direction_deg:.1f}° "
                                f"(compass, {simulator_wind_direction:.1f}° simulator) via parameter change"
                            )
                        else:
                            self.get_logger().warn("Simulator environment does not have _config attribute")
                    else:
                        self.get_logger().warn("Simulator manager does not have environment attribute")
                except Exception as e:
                    self.get_logger().error(f"Failed to update wind direction in simulator: {e}")
        elif self.mode == 'local' and self.use_mock:
            # Update mock simulator's wind direction
            # Note: Mock simulator also uses trigonometric convention, so apply same conversion
            if hasattr(self, 'simulator') and self.simulator is not None:
                self.simulator.wind_direction = float(simulator_wind_direction)
                self.get_logger().info(
                    f"Mock simulator wind direction changed to {wind_direction_deg:.1f}° "
                    f"(compass, {simulator_wind_direction:.1f}° simulator)"
                )
    
    def _on_parameter_change(self, parameters):
        """Handle runtime parameter changes (called when parameters are set via ros2 param set or Foxglove)"""
        from rcl_interfaces.msg import SetParametersResult
        
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            if param.name == 'simulation.simulation_rate':
                old_rate = self.simulation_rate
                new_rate = param.get_parameter_value().double_value
                if new_rate > 0:
                    self.simulation_rate = new_rate
                    # Cancel and recreate timer with new rate (only for local mode)
                    if self.mode == 'local' and hasattr(self, 'sim_timer') and self.sim_timer is not None:
                        self.sim_timer.cancel()
                        self.sim_timer = self.create_timer(1.0/self.simulation_rate, self.local_simulation_step)
                        self.get_logger().info(
                            f"Simulation rate changed from {old_rate:.1f} Hz to {self.simulation_rate:.1f} Hz "
                            f"(change via ros2 param set or Foxglove)"
                        )
                else:
                    result.successful = False
                    result.reason = f"Invalid simulation_rate: {new_rate} (must be > 0)"
            elif param.name == 'simulation.wind_direction':
                old_wind_dir = self.wind_direction
                new_wind_dir = param.get_parameter_value().double_value
                # Normalize to 0-360 range
                new_wind_dir = new_wind_dir % 360.0
                
                self.wind_direction = new_wind_dir
                # Apply the change to the simulator
                self._apply_wind_direction_to_simulator(new_wind_dir)
                
                self.get_logger().info(
                    f"Wind direction parameter changed from {old_wind_dir:.1f}° to {new_wind_dir:.1f}° "
                    f"(change via ros2 param set or Foxglove)"
                )
            else:
                # Allow other parameters (don't fail on unknown parameters)
                pass
        
        return result
    
    def publish_navsat_fix(self, trace_id=None):
        """Publish a NavSatFix message.
        
        Args:
            trace_id: Optional simulation step ID for debug tracing
        """
        if not self.boat_state:
            return
        
        # Store initial state on first valid boat_state for reset capability
        if self.initial_boat_state is None:
            self.initial_boat_state = self.boat_state.copy()

        # Validate boat position before converting to GPS coordinates
        # Check for NaN or invalid values that could cause GPS jumps
        boat_x = self.boat_state.get('x', 0.0)
        boat_y = self.boat_state.get('y', 0.0)
        
        if math.isnan(boat_x) or math.isnan(boat_y) or math.isinf(boat_x) or math.isinf(boat_y):
            self.get_logger().warn(f"Invalid boat position detected: x={boat_x}, y={boat_y} - skipping GPS publish")
            return
        
        # Check for suspiciously large position values that would cause GPS jumps
        # If boat has moved more than 10km from origin, something is wrong
        distance_from_origin = math.sqrt(boat_x**2 + boat_y**2)
        if distance_from_origin > 10000.0:  # 10km
            self.get_logger().warn(f"Boat position suspiciously far from origin: {distance_from_origin:.1f}m (x={boat_x:.1f}, y={boat_y:.1f}) - skipping GPS publish")
            return

        lat, lon = self.xy_to_latlon(boat_x, boat_y)
        
        # Validate GPS coordinates before publishing
        if math.isnan(lat) or math.isnan(lon) or math.isinf(lat) or math.isinf(lon):
            self.get_logger().warn(f"Invalid GPS coordinates calculated: lat={lat}, lon={lon} - skipping GPS publish")
            return

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

    def joy_control_callback(self, msg):
        """Receive joystick control commands (from Foxglove Joystick panel or gamepad) and convert to rudder/sail control."""
        # Get joystick axis mapping parameters
        rudder_axis = self.get_parameter('joy_rudder_axis').get_parameter_value().integer_value
        sail_axis = self.get_parameter('joy_sail_axis').get_parameter_value().integer_value
        rudder_invert = self.get_parameter('joy_rudder_invert').get_parameter_value().bool_value
        sail_invert = self.get_parameter('joy_sail_invert').get_parameter_value().bool_value
        
        # Extract axis values (clamp to valid range)
        if len(msg.axes) == 0:
            # No axes available
            return
        
        if len(msg.axes) > max(rudder_axis, sail_axis):
            rudder_value = msg.axes[rudder_axis] if rudder_axis < len(msg.axes) else 0.0
            sail_value = msg.axes[sail_axis] if sail_axis < len(msg.axes) else 0.0
            
            # Apply inversion if configured
            if rudder_invert:
                rudder_value = -rudder_value
            if sail_invert:
                sail_value = -sail_value
            
            # Clamp to -1.0 to 1.0 range
            rudder_value = max(-1.0, min(1.0, rudder_value))
            sail_value = max(-1.0, min(1.0, sail_value))
            
            # Create Vector3 message and forward to radio_control_callback
            control_msg = Vector3()
            control_msg.x = rudder_value
            control_msg.y = sail_value
            control_msg.z = 0.0
            
            self.radio_control_callback(control_msg)
            
            self.get_logger().debug(f'Joy control: axis[{rudder_axis}]={rudder_value:.3f} (rudder), axis[{sail_axis}]={sail_value:.3f} (sail)')
        else:
            self.get_logger().warn(f'Joy message has {len(msg.axes)} axes, but need at least {max(rudder_axis, sail_axis) + 1} axes')
    
    def radio_control_callback(self, msg):
        """Receive external radio control commands (from Foxglove Teleop, keyboard, etc.) and apply to simulator."""
        current_time = time.time()
        self.last_external_control_time = current_time
        self.last_human_activity = current_time
        
        # Store external control values
        self.external_rudder = msg.x  # -1 to +1
        self.external_sail = msg.y    # -1 to +1
        
        # Mark as human controlled when receiving external input
        self.human_controlled = True
        
        # Apply control to simulator immediately
        self._apply_control_to_simulator(self.external_rudder, self.external_sail)
        
        # Check for significant changes to update human activity
        if hasattr(self, 'prev_external_rudder') and hasattr(self, 'prev_external_sail'):
            rudder_change = abs(self.external_rudder - self.prev_external_rudder)
            sail_change = abs(self.external_sail - self.prev_external_sail)
            
            # Update human activity if there's significant change
            if rudder_change > 0.01 or sail_change > 0.01:
                self.last_human_activity = current_time
        
        # Store for next comparison
        self.prev_external_rudder = self.external_rudder
        self.prev_external_sail = self.external_sail
        
        self.get_logger().debug(f'External control: rudder={self.external_rudder:.3f}, sail={self.external_sail:.3f}')
    
    def _apply_control_to_simulator(self, rudder, sail):
        """Apply control commands to the simulator."""
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
    
    def control_callback(self, msg):
        """Receive control commands from Argo (final servo commands) and apply to simulator."""
        self.last_control_time = time.time()
        
        # Only apply if we're not in human control mode (external control takes priority)
        if not self.human_controlled:
            # Apply control to simulator
            rudder = msg.x  # -1 to +1
            sail = msg.y    # -1 to +1
            self._apply_control_to_simulator(rudder, sail)
            self.get_logger().debug(f'Robot control: rudder={rudder:.3f}, sail={sail:.3f}')
    
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
        """Receive radio input from remote simulator (deprecated - use radio_control_callback instead)"""
        if self.mode == 'remote':
            # Forward to radio_control_callback for consistent handling
            self.radio_control_callback(msg)
    
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
        super().destroy_node()

def load_map_from_yaml():
    """Load map name from argo_nodes.yaml configuration file."""
    try:
        # Determine path to argo_nodes.yaml (launch/ directory)
        script_path = Path(__file__).resolve()
        argo_dir = script_path.parents[1]  # nodes -> argo
        yaml_path = argo_dir / "launch" / "argo_nodes.yaml"
        
        if not yaml_path.exists():
            return None
        
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Extract map_name from simulation_config
        simulation_config = config.get('simulation_config', {})
        map_name = simulation_config.get('map_name')
        return map_name
    except Exception as e:
        # Silently fail - map_name will be None and defaults will be used
        return None

def print_help():
    """Print detailed help information."""
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

MAP CONFIGURATION:
  Map name is automatically loaded from launch/argo_nodes.yaml.
  Use --map to override the default map for this run.

USAGE:
  python3 argo_unified_simulator_bridge.py --mode local              # Local simulation (uses map from YAML)
  python3 argo_unified_simulator_bridge.py --mode remote             # Remote simulation (uses map from YAML)
  python3 argo_unified_simulator_bridge.py --mode local --map "Map Name"  # Override map from YAML
  python3 argo_unified_simulator_bridge.py --mode local --debug      # Enable debug tracing (verbose logging)
"""
    print(help_text)

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Argo Unified Simulator Bridge - Local and Remote Simulation Support',
        add_help=False  # We'll handle --help manually
    )
    parser.add_argument('-h', '--help', action='store_true', help='Show this help message and exit')
    parser.add_argument('--mode', choices=['local', 'remote'],
                       help='Simulation mode: local (run simulator here) or remote (connect to remote simulator)')
    parser.add_argument('--map', type=str,
                       help='Map name (without .geojson extension) - overrides map from argo_nodes.yaml')
    parser.add_argument('--test-heading', type=float,
                       help='Test mode: override initial heading calculation with specified heading in degrees (0-360)')
    parser.add_argument('--force-mock', action='store_true',
                       help='Force use of mock simulator even if real simulator (sailboat-playground) is available')
    parser.add_argument('--debug', action='store_true',
                       help='Enable debug tracing (verbose position and control logging)')
    
    # Parse known args to avoid conflicts with ROS2 args
    parsed_args, unknown_args = parser.parse_known_args(args)
    
    # Handle --help option or missing mode argument
    if parsed_args.help:
        print_help()
        sys.exit(0)
    
    if not parsed_args.mode:
        print("ERROR: You must supply a mode argument.")
        print("       Please choose either --mode local or --mode remote.")
        print("       For more information, run with --help")
        sys.exit(1)
    
    # Load map name from YAML if not provided via command line
    map_name = parsed_args.map
    if not map_name:
        map_name = load_map_from_yaml()
        if map_name:
            print(f"📍 Using map from argo_nodes.yaml: '{map_name}'")
    
    print(f"Starting Argo Unified Simulator Bridge in {parsed_args.mode.upper()} mode...")
    print("This node bridges between Argo control system and sailboat simulator")
    print(f"\nMode: {parsed_args.mode.upper()}")
    if parsed_args.mode == 'local':
        print("  - Running simulator locally")
        if parsed_args.force_mock:
            print("  - Using mock simulator (--force-mock enabled)")
        else:
            print("  - Using sailboat-playground or mock simulator")
    else:
        print("  - Connecting to remote simulator")
        print("  - Forwarding sensor data and control commands")
    if parsed_args.debug:
        print("  - Debug tracing ENABLED (--debug)")
    if map_name:
        print(f"  - Map: {map_name} (config file: nodes/argo_nodes.yaml)")
        time.sleep(1)
    
    print("\nPublished topics (Simulator → Argo):")
    print("  /pose - IMU compass heading")
    print("  /compass - Raw compass data") 
    print("  /gps_cog, /gps_sog, /gps_velocity - GPS navigation")
    print("  /anem_speed_angle_temp - Wind data")
    print("\nSubscribed topics (Argo → Simulator):")
    print("  /rudder_sail_radio - External control input (Vector3, x=rudder, y=sail)")
    print("  /joy - Joystick control input (Joy message from Foxglove Joystick panel)")
    print("  /rudder_sail_servo - Final servo commands from Argo")
    print("  /human_controlled - Control mode status")
    print("\nExternal Control:")
    print("  1. Keyboard Control (separate node):")
    print("     python3 nodes/argo_keyboard_control.py")
    print("  2. Terminal (Vector3):")
    print("     ros2 topic pub /rudder_sail_radio geometry_msgs/msg/Vector3 '{x: 0.5, y: -0.3, z: 0.0}'")
    print("\nPress Ctrl+C to stop\n")
    
    rclpy.init(args=unknown_args)
    bridge = None
    try:
        bridge = ArgoUnifiedSimulatorBridge(mode=parsed_args.mode, map_name=map_name, test_heading=parsed_args.test_heading, force_mock=parsed_args.force_mock, debug=parsed_args.debug)
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print(f"\nUnified simulator bridge ({parsed_args.mode} mode) stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bridge:
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
