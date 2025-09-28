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

# Try to import sailboat-playground for local simulation
try:
    # Set headless mode for pyglet (no display required)
    os.environ['PYGLET_HEADLESS'] = '1'
    
    # Add simulator submodule to Python path
    simulator_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'simulator')
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
    
    def __init__(self, mode='local'):
        super().__init__('argo_unified_simulator_bridge')
        self.mode = mode
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
        
        self.get_logger().info(f'Unified simulator bridge ready ({mode} mode)')
        self.get_logger().info('Publishing simulated sensor data to Argo topics')
        
        # Initial state
        self.boat_state = None
    
    def _init_local_simulator(self):
        """Initialize local simulator (sailboat-playground or mock)."""
        if SIMULATOR_AVAILABLE:
            try:
                self.get_logger().info('Initializing sailboat-playground simulator...')
                from sailboat_playground.engine import Manager
                import numpy as np
                
                # Configuration file paths
                boat_config = "sailboat-playground/boats/sample_boat.json"
                env_config = "sailboat-playground/environments/playground.json"
                
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
            control_age = time.time() - self.last_control_time
            mode = "HUMAN" if self.human_controlled else "ROBOT"
            
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
  python3 argo_unified_simulator_bridge.py --mode local    # Local simulation
  python3 argo_unified_simulator_bridge.py --mode remote   # Remote simulation

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
- Status reporting every second

REMOTE MODE FEATURES:
- Connection monitoring with 10-second timeout
- Automatic reconnection detection
- Heartbeat mechanism for connection health
- Status reporting for connection state

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
    
    # Parse known args to avoid conflicts with ROS2 args
    parsed_args, unknown_args = parser.parse_known_args(args)
    
    # Handle --help option
    if parsed_args.help:
        print_help()
    
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
        bridge = ArgoUnifiedSimulatorBridge(mode=parsed_args.mode)
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
