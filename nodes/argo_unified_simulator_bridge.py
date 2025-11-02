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

    print(f"         Falling back to mock simulator.")
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
    
    def __init__(self, mode='local', map_name=None):
        super().__init__('argo_unified_simulator_bridge')
        self.mode = mode
        
        self.get_logger().info(f'Argo Unified Simulator Bridge starting in {mode.upper()} mode...')
        
        # --- GPS Base Location (for NavSatFix) ---
        # Load from map GeoJSON if specified, otherwise use default
        self.base_latitude = 47.3769  # Default: Zurich, Switzerland
        self.base_longitude = 8.5417
        if map_name:
            self._load_map_home_location(map_name)
        
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
    
    def _init_local_simulator(self):
        """Initialize local simulator (sailboat-playground or mock)."""
        if SIMULATOR_AVAILABLE:
            try:
                self.get_logger().info('Initializing sailboat-playground simulator...')
                import numpy as np
                
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

                # Initialize the sailboat-playground simulation manager with configs
                self.sim_manager = engine.Manager(
                    boat_config,
                    env_config,
                    foils_dir=foils_path  # Provide the absolute path here
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
            
            # Determine which control source to use
            # Priority: External control (Teleop/keyboard) > Robot control
            if self.human_controlled and (time.time() - self.last_external_control_time) < 2.0:
                # Use external control if recent
                current_rudder = self.external_rudder
                current_sail = self.external_sail
            else:
                # Use stored robot control values (from control_callback)
                current_rudder = getattr(self, 'last_rudder_angle', 0.0) / 30.0  # Convert back from degrees
                current_sail = getattr(self, 'last_sail_angle', 0.0) / 45.0     # Convert back from degrees
            
            # Update simulator physics
            if self.use_mock:
                self.boat_state = self.simulator.step()
            else:
                # Handle real sailboat-playground API
                try:
                    # Convert normalized values to degrees for sailboat-playground
                    rudder_angle = current_rudder * 30.0  # Convert to degrees (-30 to +30)
                    sail_angle = current_sail * 45.0      # Convert to degrees (-45 to +45)
                    
                    # Store for reference
                    self.last_rudder_angle = rudder_angle
                    self.last_sail_angle = sail_angle
                    
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
                        'rudder': current_rudder,  # Store normalized values
                        'sail': current_sail
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
                    self.get_logger().info(f"Loaded start location from map '{map_name}': "
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
    
    def xy_to_latlon(self, x, y):
        """Convert XY meters from base lat/lon to new lat/lon."""
        R = 6378137.0  # Earth radius in meters
        dLat = y / R
        dLon = x / (R * math.cos(math.pi * self.base_latitude / 180))
        lat = self.base_latitude + dLat * 180 / math.pi
        lon = self.base_longitude + dLon * 180 / math.pi
        return lat, lon

    def reset_simulation_callback(self, request, response):
        """Reset simulation to initial state (home waypoint, zero speed/heading)."""
        try:
            self.get_logger().info("Resetting simulation to initial state...")
            
            # Reset real sailboat-playground simulator state
            if not self.use_mock and hasattr(self, 'sim_manager') and self.sim_manager:
                try:
                    import numpy as np
                    # Reset boat position to origin (home waypoint)
                    self.sim_manager.boat.set_position(np.array([0.0, 0.0]))
                    self.sim_manager.boat.set_heading(0.0)
                    # Reset boat velocity
                    self.sim_manager.boat._speed = np.array([0.0, 0.0])
                    self.sim_manager.boat._angular_speed = 0.0
                    self.get_logger().info("Real simulator reset to origin")
                except Exception as e:
                    self.get_logger().warn(f"Failed to reset real simulator: {e}")
            
            # Reset mock simulator state
            if self.use_mock and hasattr(self.simulator, 'boat_x'):
                self.simulator.boat_x = 0.0
                self.simulator.boat_y = 0.0
                self.simulator.boat_heading = 0.0
                self.simulator.boat_speed = 0.0
                self.simulator.rudder_angle = 0.0
                self.simulator.sail_angle = 0.0
                self.get_logger().info("Mock simulator reset to origin")
            
            # Reset boat_state to origin (home waypoint at base_latitude/base_longitude)
            # Always reset to (0,0) in local coordinates, which maps to home waypoint
            wind_speed = self.simulator.wind_speed if hasattr(self.simulator, 'wind_speed') else 8.0
            wind_direction = self.simulator.wind_direction if hasattr(self.simulator, 'wind_direction') else 45.0
            self.boat_state = {
                'x': 0.0,  # Always reset to origin (home waypoint)
                'y': 0.0,
                'heading': 0.0,
                'speed': 0.0,
                'wind_speed': wind_speed,
                'wind_direction': wind_direction,
                'rudder': 0.0,
                'sail': 0.0
            }
            # Update initial_boat_state so future resets use this as reference
            self.initial_boat_state = self.boat_state.copy()
            
            # Reset control state
            self.last_control_time = time.time()
            self.human_controlled = True
            
            response.success = True
            response.message = "Simulation reset to initial state (home waypoint)"
            self.get_logger().info("✅ Simulation reset successful")
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error resetting simulation: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            response.success = False
            response.message = f"Reset failed: {str(e)}"
            return response
    
    def publish_navsat_fix(self):
        """Publish a NavSatFix message."""
        if not self.boat_state:
            return
        
        # Store initial state on first valid boat_state for reset capability
        if self.initial_boat_state is None:
            self.initial_boat_state = self.boat_state.copy()

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

USAGE:
  python3 argo_unified_simulator_bridge.py --mode local    # Local simulation
  python3 argo_unified_simulator_bridge.py --mode remote   # Remote simulation
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
                       help='Map name (without .geojson extension) - start location will be at the "home" waypoint')
    
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
        bridge = ArgoUnifiedSimulatorBridge(mode=parsed_args.mode, map_name=parsed_args.map)
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
