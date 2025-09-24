#!/usr/bin/env python3
# Argo Simulator Bridge - Connects Argo control system with sailboat-playground simulator
# Provides simulated sensor data to Argo and receives control commands

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Vector3
import numpy as np
import time
import math
import threading

try:
    # Set headless mode for pyglet (no display required)
    import os
    os.environ['PYGLET_HEADLESS'] = '1'
    from sailboat_playground import SailboatPlayground
    SIMULATOR_AVAILABLE = True
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

class ArgoSimulatorBridge(Node):
    """Bridge between Argo control system and sailing simulator."""
    
    def __init__(self):
        super().__init__('argo_simulator_bridge')
        self.get_logger().info('Argo Simulator Bridge starting...')
        
        # Initialize simulator - always use mock for now as it's more reliable
        self.get_logger().info('Using mock simulator (reliable for headless operation)')
        self.simulator = MockSailboatSimulator()
        self.use_mock = True
        
        # Could try real simulator here if needed:
        # if SIMULATOR_AVAILABLE:
        #     try:
        #         self.get_logger().info('Initializing sailboat-playground simulator...')
        #         self.simulator = SailboatPlayground()
        #         self.use_mock = False
        #         self.get_logger().info('Real simulator initialized successfully')
        #     except Exception as e:
        #         self.get_logger().warn(f'Failed to initialize real simulator: {e}')
        #         self.get_logger().info('Falling back to mock simulator')
        #         self.simulator = MockSailboatSimulator()
        #         self.use_mock = True
        
        # Simulation state
        self.last_control_time = time.time()
        self.simulation_running = True
        
        # --- Publishers (Simulator → Argo) ---
        # IMU/Compass data
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        
        # GPS data
        self.pub_gps_cog = self.create_publisher(Float64, '/gps_cog', 10)
        self.pub_gps_sog = self.create_publisher(Float64, '/gps_sog', 10)
        self.pub_gps_velocity = self.create_publisher(Vector3, '/gps_velocity', 10)
        
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
        self.human_controlled = True  # Start in human control
        
        # Mock human input for testing
        self.mock_human_input = True
        self.human_input_time = 0.0
        
        # --- Timers ---
        self.sim_timer = self.create_timer(1.0/self.simulation_rate, self.simulation_step)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        self.get_logger().info(f'Simulator bridge ready ({"mock" if self.use_mock else "real"} simulator)')
        self.get_logger().info('Publishing simulated sensor data to Argo topics')
        
        # Initial state
        self.boat_state = None
    
    def simulation_step(self):
        """Main simulation step - updates physics and publishes sensor data."""
        try:
            # Generate mock human input for testing (optional)
            if self.mock_human_input and self.human_controlled:
                self.publish_mock_human_input()
            
            # Update simulator physics
            if self.use_mock:
                self.boat_state = self.simulator.step()
            else:
                # Handle real simulator API (would need to check actual API)
                try:
                    self.boat_state = self.simulator.step()
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
    
    def publish_mock_human_input(self):
        """Generate mock human radio input for testing."""
        self.human_input_time += 1.0 / self.simulation_rate
        
        # Generate some human-like control input
        rudder = 0.3 * math.sin(self.human_input_time * 0.5)
        sail = 0.2 + 0.2 * math.cos(self.human_input_time * 0.3)
        
        radio_msg = Vector3(x=rudder, y=sail, z=0.0)
        self.pub_radio.publish(radio_msg)
    
    def control_callback(self, msg):
        """Receive control commands from Argo and apply to simulator."""
        self.last_control_time = time.time()
        
        # Apply control to simulator
        rudder = msg.x  # -1 to +1
        sail = msg.y    # -1 to +1
        
        if self.use_mock:
            self.simulator.set_control(rudder, sail)
        else:
            # Handle real simulator control API
            try:
                self.simulator.set_control(rudder, sail)
            except Exception as e:
                self.get_logger().warn(f'Failed to set control: {e}')
        
        self.get_logger().debug(f'Applied control: rudder={rudder:.3f}, sail={sail:.3f}')
    
    def human_control_callback(self, msg):
        """Monitor human control status."""
        if self.human_controlled != msg.data:
            self.human_controlled = msg.data
            mode = "HUMAN" if msg.data else "ROBOT"
            self.get_logger().info(f'Control mode: {mode}')
    
    def print_status(self):
        """Print simulation status periodically."""
        if self.boat_state:
            control_age = time.time() - self.last_control_time
            mode = "HUMAN" if self.human_controlled else "ROBOT"
            
            self.get_logger().info(
                f'Boat: heading={self.boat_state["heading"]:.1f}°, '
                f'speed={self.boat_state["speed"]:.1f}m/s, '
                f'wind={self.boat_state["wind_direction"]:.0f}°, '
                f'mode={mode}, '
                f'last_cmd={control_age:.1f}s ago'
            )

def main(args=None):
    print("Starting Argo Simulator Bridge...")
    print("This node bridges between Argo control system and sailboat simulator")
    print("\nPublished topics (Simulator → Argo):")
    print("  /pose - IMU compass heading")
    print("  /compass - Raw compass data") 
    print("  /gps_cog, /gps_sog, /gps_velocity - GPS navigation")
    print("  /anem_speed_angle_temp - Wind data")
    print("  /rudder_sail_radio - Mock human input (when enabled)")
    print("\nSubscribed topics (Argo → Simulator):")
    print("  /rudder_sail_servo - Control commands from Argo")
    print("  /human_controlled - Control mode status")
    print("\nTo test with Argo two-node system:")
    print("  1. Run this simulator bridge")
    print("  2. Run: python3 nodes/rudder_sail_radio.py --ros-args --params-file nodes/argo.yaml")
    print("  3. Run: python3 nodes/controller.py --ros-args --params-file nodes/argo.yaml")
    print("\nPress Ctrl+C to stop\n")
    
    rclpy.init(args=args)
    bridge = None
    try:
        bridge = ArgoSimulatorBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("\nSimulator bridge stopped by user.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if bridge:
            bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
