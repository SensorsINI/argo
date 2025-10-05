#!/usr/bin/env python3
# High-level autonomous controller for Argo sailboat
# Subscribes to sensor data and publishes control commands to rudder_sail_radio.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float32
from geometry_msgs.msg import Vector3
from rclpy.parameter import Parameter

import yaml
import argparse
from pathlib import Path
import math
import time
import numpy as np
from dataclasses import dataclass, asdict
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, List
import json
import threading
import os
import psutil


def signed_angle_difference_degrees(angle1_deg, angle2_deg):
    """
    Computes the signed difference between two angles in degrees,
    returning a result in the range [-180, 180].
    """
    diff_deg = angle1_deg - angle2_deg
    return (diff_deg + 180.0) % 360.0 - 180.0


@dataclass
class BoatState:
    """Complete state representation of the boat from all sensors."""
    # Time
    timestamp: float = 0.0

    # Navigation
    compass_heading: Optional[float] = None  # degrees (0-360)
    gps_cog: Optional[float] = None          # course over ground, degrees true
    gps_sog: Optional[float] = None          # speed over ground, knots
    gps_velocity: Optional[Vector3] = None   # x=north, y=east, z=speed

    # IMU
    accel: Optional[Vector3] = None          # accelerometer, g units
    gyro: Optional[Vector3] = None           # gyroscope, deg/s
    compass_raw: Optional[Vector3] = None    # magnetometer, µT

    # Wind
    wind_speed: Optional[float] = None       # m/s
    wind_angle: Optional[float] = None       # degrees CW from front of boat
    wind_temp: Optional[float] = None        # celsius

    # Radio/Human input (from rudder_sail_radio.py)
    radio_rudder: Optional[float] = None     # -1:+1 left:right
    radio_sail: Optional[float] = None       # -1:+1 in:out
    human_controlled: bool = True            # control mode

    # Controller state
    target_heading: Optional[float] = None   # degrees

    # Battery/Water monitoring (from battery_water node with persistent QoS)
    battery_voltage: Optional[float] = None      # volts
    battery_remaining_pct: Optional[float] = None  # percentage
    battery_low_alert: bool = False              # low battery alert
    saltwater_alert: bool = False                # saltwater intrusion alert
    humidity_alert: bool = False                 # high humidity alert

    def is_valid_for_control(self) -> bool:
        """Check if we have minimum required data for autonomous control."""
        return (self.compass_heading is not None and
                self.target_heading is not None)

    def has_critical_alerts(self) -> bool:
        """Check if any critical alerts are active that should affect control."""
        return self.battery_low_alert or self.saltwater_alert

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for data logging."""
        data = asdict(self)
        # Convert Vector3 objects to dicts
        for key, value in data.items():
            if hasattr(value, 'x'):  # Vector3 object
                data[key] = {'x': value.x, 'y': value.y, 'z': value.z}
        return data


@dataclass
class ControlCommand:
    """Control output commands for rudder and sail."""
    rudder: float = 0.0     # -1:+1 left:right
    sail: float = 0.0       # -1:+1 in:out
    timestamp: float = 0.0

    def to_vector3(self) -> Vector3:
        """Convert to ROS Vector3 message."""
        return Vector3(x=self.rudder, y=self.sail, z=0.0)

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for data logging."""
        return asdict(self)


class BaseController(ABC):
    """Abstract base class for all controllers."""

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.name = self.__class__.__name__

    @abstractmethod
    def generate_control(self, state: BoatState) -> ControlCommand:
        """
        Main control function that takes boat state and returns control commands.

        Args:
            state: Current boat state from all sensors

        Returns:
            ControlCommand with rudder and sail commands
        """
        pass

    def reset(self):
        """Reset controller state (called when switching to this controller)."""
        pass

    def update_config(self, config: Dict[str, Any]):
        """Update controller configuration."""
        self.config.update(config)


class ProportionalHeadingController(BaseController):
    """Simple proportional controller for heading maintenance."""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate control commands using proportional heading control."""
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)

        # Calculate heading error (target - current)
        compass_err = signed_angle_difference_degrees(
            state.target_heading, state.compass_heading)

        # Proportional controller for rudder
        cmd_rudder = self.rudder_gain * \
            (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = np.clip(cmd_rudder, -1.0, 1.0)

        # Pass through sail command from radio (could be enhanced later)
        cmd_sail = state.radio_sail if state.radio_sail is not None else 0.0

        return ControlCommand(
            rudder=cmd_rudder,
            sail=cmd_sail,
            timestamp=state.timestamp
        )


class WindAwareController(BaseController):
    """Enhanced controller that considers wind conditions."""

    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate control commands considering wind conditions."""
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)

        # Heading control (same as proportional)
        compass_err = signed_angle_difference_degrees(
            state.target_heading, state.compass_heading)
        cmd_rudder = self.rudder_gain * \
            (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = np.clip(cmd_rudder, -1.0, 1.0)

        # Wind-aware sail control
        # TODO check that this makes sense given simulator, since it might not treat sail winch correctly as setting maximum sail angle
        cmd_sail = state.radio_sail if state.radio_sail is not None else 0.0

        if state.wind_angle is not None:
            # Simple sail control based on wind angle
            # Wind from ahead (0°): pull sail in (-1)
            # Wind from side (90°): moderate sail position (0)
            # Wind from behind (180°): let sail out (+1)
            wind_sail_cmd = (state.wind_angle - 90.0) / 90.0
            wind_sail_cmd = np.clip(wind_sail_cmd, -1.0, 1.0)

            # Blend radio command with wind-based command
            cmd_sail = (1 - self.sail_wind_gain) * cmd_sail + \
                self.sail_wind_gain * wind_sail_cmd

        return ControlCommand(
            rudder=cmd_rudder,
            sail=cmd_sail,
            timestamp=state.timestamp
        )


class DataCollector:
    """Collects state-action pairs for training data."""

    def __init__(self, data_dir: str = None):
        if data_dir is None:
            # Default to user's home directory + argo_data for write permissions
            self.data_dir = Path.home() / "argo_data" / "training_data"
        else:
            self.data_dir = Path(data_dir)

        try:
            self.data_dir.mkdir(parents=True, exist_ok=True)
        except PermissionError as e:
            print(
                f"Warning: Cannot create training data directory {self.data_dir}: {e}")
            print("Training data collection will be disabled.")
            self.data_dir = None
        self.current_session_data = []
        self.session_start_time = None
        self.lock = threading.Lock()
        self.enabled = False

    def start_session(self):
        """Start a new data collection session."""
        with self.lock:
            if self.data_dir is None:
                print(
                    "Warning: Training data directory not available. Data collection disabled.")
                self.enabled = False
                return
            self.session_start_time = time.time()
            self.current_session_data = []
            self.enabled = True
            print(f"Started data collection session at {time.ctime()}")

    def stop_session(self):
        """Stop current session and save data."""
        with self.lock:
            if not self.enabled or not self.current_session_data or self.data_dir is None:
                return

            # Save session data
            session_filename = f"session_{int(self.session_start_time)}.json"
            session_path = self.data_dir / session_filename

            session_metadata = {
                'start_time': self.session_start_time,
                'end_time': time.time(),
                'sample_count': len(self.current_session_data),
                'data': self.current_session_data
            }

            with open(session_path, 'w') as f:
                json.dump(session_metadata, f, indent=2)

            print(
                f"Saved {len(self.current_session_data)} samples to {session_path}")
            self.current_session_data = []
            self.enabled = False

    def record_sample(self, state: BoatState, human_action: ControlCommand):
        """Record a state-action pair during human control."""
        with self.lock:
            if not self.enabled:
                return

            sample = {
                'state': state.to_dict(),
                'action': human_action.to_dict(),
                'relative_time': state.timestamp - self.session_start_time
            }

            self.current_session_data.append(sample)


class ControllerNode(Node):
    """High-level autonomous controller node."""

    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Controller node starting...')

        # --- Parameters ---
        self.declare_parameter('param_file_path', 'argo.yaml')
        self.declare_parameter('controller_type', 'proportional')
        self.declare_parameter('data_collection_enabled', False)
        self.declare_parameter('training_data_dir', '')  # Empty = use default
        self.declare_parameter('rudder_gain', 1.0)
        self.declare_parameter('rudder_full_scale_deg', 60.0)
        # Allow disabling param file monitoring
        self.declare_parameter('enable_param_reload', True)

        self.param_file = Path(self.get_parameter(
            'param_file_path').get_parameter_value().string_value)
        self._last_param_mtime = 0

        # Load initial parameters
        self.check_and_reload_params(is_initial=True)


        # --- State and Control ---
        self.boat_state = BoatState()
        self.controller = None

        # Initialize data collector with configurable directory
        training_data_dir = self.get_parameter(
            'training_data_dir').get_parameter_value().string_value
        if not training_data_dir:
            training_data_dir = None  # Use default
        self.data_collector = DataCollector(training_data_dir)

        self.last_logged_human_control = None

        # CPU monitoring
        self.cpu_monitor_enabled = True
        self.last_cpu_check = time.time()
        self.cpu_check_interval = 30.0  # Check CPU usage every 30 seconds

        # Initialize controller
        self._initialize_controller()

        # --- Publishers ---
        # Real-time control commands
        self.pub_rudder_sail_cmd = self.create_publisher(
            Vector3, '/rudder_sail_cmd', 10)

        # Health status publisher
        self.pub_health = self.create_publisher(Bool, '/controller_health', 10)
        self.health_status = False  # Track current health status

        # --- Subscribers ---
        # Control status from rudder_sail_radio.py
        self.create_subscription(
            Bool, '/human_controlled', self.human_control_callback, 10)
        self.create_subscription(
            Vector3, '/control_authority', self.control_authority_callback, 10)

        # Radio input for reference (real-time data)
        self.create_subscription(
            Vector3, '/rudder_sail_radio', self.radio_reference_callback, 10)

        # Navigation sensors (real-time data)
        self.create_subscription(
            Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(
            Float64, '/gps_cog', self.gps_cog_callback, 10)
        self.create_subscription(
            Float64, '/gps_sog', self.gps_sog_callback, 10)
        self.create_subscription(
            Vector3, '/gps_velocity', self.gps_velocity_callback, 10)

        # IMU sensors (real-time data)
        self.create_subscription(
            Vector3, '/accel', self.accel_callback, 10)
        self.create_subscription(
            Vector3, '/gyro', self.gyro_callback, 10)
        self.create_subscription(
            Vector3, '/compass', self.compass_callback, 10)

        # Wind sensor (real-time data)
        self.create_subscription(
            Vector3, '/anem_speed_angle_temp', self.wind_callback, 10)

        # Battery/Water monitoring
        self.create_subscription(
            Float32, '/battery_voltage', self.battery_voltage_callback, 10)
        self.create_subscription(Float32, '/battery_remaining_pct',
                                 self.battery_remaining_callback, 10)
        self.create_subscription(
            Bool, '/battery_low_alert', self.battery_low_alert_callback, 10)
        self.create_subscription(
            Bool, '/saltwater_alert', self.saltwater_alert_callback, 10)
        self.create_subscription(
            Bool, '/humidity_alert', self.humidity_alert_callback, 10)

        # --- Timers ---
        # 5 Hz (reduced from 10 Hz for lower CPU usage)
        self.control_loop_period = 0.2
        self.timer = self.create_timer(
            self.control_loop_period, self.timer_callback)

        # Publish initial health status as healthy
        self._publish_health_status(True)

        # Only create parameter reload timer if enabled (CPU optimization)
        if self.get_parameter('enable_param_reload').get_parameter_value().bool_value:
            # Reduced frequency from 3s to 10s for lower CPU usage
            self.param_reload_check_period = 10.0
            self.param_timer = self.create_timer(
                self.param_reload_check_period, self.check_and_reload_params)
        else:
            self.param_timer = None

    def _publish_health_status(self, is_healthy: bool):
        """Publish health status and update internal state"""
        if self.health_status != is_healthy:
            self.health_status = is_healthy
            health_msg = Bool()
            health_msg.data = is_healthy
            self.pub_health.publish(health_msg)

            if is_healthy:
                self.get_logger().info("Controller health status: HEALTHY")
            else:
                self.get_logger().warn("Controller health status: FAILED")
            self.get_logger().info("Parameter file monitoring disabled for performance")

    def _initialize_controller(self):
        """Initialize the controller based on parameters."""
        controller_type = self.get_parameter(
            'controller_type').get_parameter_value().string_value

        config = {
            'rudder_gain': self.get_parameter('rudder_gain').get_parameter_value().double_value,
            'rudder_full_scale_deg': self.get_parameter('rudder_full_scale_deg').get_parameter_value().double_value,
        }

        if controller_type == 'proportional':
            self.controller = ProportionalHeadingController(config)
        elif controller_type == 'wind_aware':
            config['sail_wind_gain'] = 0.5  # Could be a parameter
            self.controller = WindAwareController(config)
        else:
            self.get_logger().warn(
                f"Unknown controller type '{controller_type}', using proportional")
            self.controller = ProportionalHeadingController(config)

        self.get_logger().info(
            f"Initialized controller: {self.controller.name}")

    def switch_controller(self, controller_type: str):
        """Switch to a different controller type."""
        old_controller = self.controller.name if self.controller else "None"

        # Update parameter and reinitialize
        self.set_parameter(Parameter('controller_type', value=controller_type))
        self._initialize_controller()

        if self.controller:
            self.controller.reset()

        self.get_logger().info(
            f"Switched controller from {old_controller} to {self.controller.name}")

    # --- Sensor Callbacks ---
    def human_control_callback(self, msg):
        """Receive control authority status from rudder_sail_radio.py."""
        old_human_control = self.boat_state.human_controlled
        self.boat_state.human_controlled = msg.data

        # Handle data collection session management
        if old_human_control != self.boat_state.human_controlled:
            if self.boat_state.human_controlled and self.get_parameter('data_collection_enabled').get_parameter_value().bool_value:
                self.data_collector.start_session()
            elif not self.boat_state.human_controlled:
                self.data_collector.stop_session()

    def control_authority_callback(self, msg):
        """Receive detailed control authority info from rudder_sail_radio.py."""
        # msg.x = current authority (1=human, 0=robot)
        # msg.y = time since human activity
        # msg.z = time since auto command
        pass  # For monitoring/debugging if needed

    def radio_reference_callback(self, msg):
        """Receive radio input reference for controller use."""
        self.boat_state.radio_rudder = msg.x
        self.boat_state.radio_sail = msg.y

    def pose_callback(self, msg):
        self.boat_state.compass_heading = msg.z

    def gps_cog_callback(self, msg):
        self.boat_state.gps_cog = msg.data

    def gps_sog_callback(self, msg):
        self.boat_state.gps_sog = msg.data

    def gps_velocity_callback(self, msg):
        self.boat_state.gps_velocity = msg

    def accel_callback(self, msg):
        self.boat_state.accel = msg

    def gyro_callback(self, msg):
        self.boat_state.gyro = msg

    def compass_callback(self, msg):
        self.boat_state.compass_raw = msg

    def wind_callback(self, msg):
        self.boat_state.wind_speed = msg.x
        self.boat_state.wind_angle = msg.y
        self.boat_state.wind_temp = msg.z

    # --- Battery/Water Monitoring Callbacks ---
    def battery_voltage_callback(self, msg):
        """Receive battery voltage from battery_water node"""
        self.boat_state.battery_voltage = msg.data

    def battery_remaining_callback(self, msg):
        """Receive battery remaining percentage from battery_water node"""
        self.boat_state.battery_remaining_pct = msg.data

    def battery_low_alert_callback(self, msg):
        """Receive battery low alert from battery_water node"""
        self.boat_state.battery_low_alert = msg.data
        if msg.data:
            self.get_logger().warn("🔋 LOW BATTERY ALERT - Consider returning to shore")

    def saltwater_alert_callback(self, msg):
        """Receive saltwater intrusion alert from battery_water node"""
        self.boat_state.saltwater_alert = msg.data
        if msg.data:
            self.get_logger().warn("💧 SALTWATER INTRUSION ALERT - Check hull integrity")

    def humidity_alert_callback(self, msg):
        """Receive humidity alert from battery_water node"""
        self.boat_state.humidity_alert = msg.data
        if msg.data:
            self.get_logger().warn("💦 HIGH HUMIDITY ALERT - Check ventilation")

    def timer_callback(self):
        """Main control loop - generates autonomous control commands."""
        try:
            self.boat_state.timestamp = time.time()

            # CPU monitoring (periodic check)
            if self.cpu_monitor_enabled and (time.time() - self.last_cpu_check) > self.cpu_check_interval:
                try:
                    process = psutil.Process()
                    cpu_percent = process.cpu_percent()
                    memory_info = process.memory_info()
                    self.get_logger().info(
                        f"Controller CPU: {cpu_percent:.1f}%, Memory: {memory_info.rss/1024/1024:.1f}MB")
                    self.last_cpu_check = time.time()
                except Exception as e:
                    self.get_logger().debug(f"CPU monitoring error: {e}")

            # Log human control state changes
            if self.boat_state.human_controlled != self.last_logged_human_control:
                if self.boat_state.human_controlled:
                    self.get_logger().info("Human has control authority.")
                else:
                    self.get_logger().info("Robot has control authority.")
                self.last_logged_human_control = self.boat_state.human_controlled

            # Check for minimum required data
            if self.boat_state.compass_heading is None:
                self.get_logger().debug("Waiting for initial compass heading...", throttle_duration_sec=5)
                return

            if self.boat_state.human_controlled:
                # Human control mode - update target heading for when robot takes over
                self.boat_state.target_heading = self.boat_state.compass_heading

                # Collect training data if enabled
                if (self.data_collector.enabled and
                    self.boat_state.radio_rudder is not None and
                        self.boat_state.radio_sail is not None):

                    human_action = ControlCommand(
                        rudder=self.boat_state.radio_rudder,
                        sail=self.boat_state.radio_sail,
                        timestamp=self.boat_state.timestamp
                    )
                    self.data_collector.record_sample(
                        self.boat_state, human_action)

                # No autonomous command published - rudder_sail_radio.py handles human input

            else:
                # Autonomous control mode
                if self.boat_state.target_heading is None:
                    self.get_logger().warn("Robot control active, but no target heading. Waiting for human to set a course.",
                                           throttle_duration_sec=5)
                    return

                if self.controller is None:
                    self.get_logger().error("No controller initialized!", throttle_duration_sec=5)
                    return

                # Check for critical alerts that should modify autonomous behavior
                if self.boat_state.has_critical_alerts():
                    if self.boat_state.battery_low_alert:
                        self.get_logger().warn("🔋 BATTERY LOW - Autonomous control may be limited",
                                               throttle_duration_sec=10)
                    if self.boat_state.saltwater_alert:
                        self.get_logger().warn("💧 SALTWATER DETECTED - Consider emergency return",
                                               throttle_duration_sec=10)

                # **THE CORE ARCHITECTURE: Single generate_control function**
                control_command = self.controller.generate_control(
                    self.boat_state)

                # Publish control command to rudder_sail_radio.py
                if control_command:
                    self.pub_rudder_sail_cmd.publish(
                        control_command.to_vector3())

                    # Publish health status as healthy
                    self._publish_health_status(True)

                    # Only format debug string if debug logging is enabled (CPU optimization)
                    if self.get_logger().get_effective_level() <= 10:  # DEBUG level
                        self.get_logger().debug(
                            f"Target: {self.boat_state.target_heading:.1f}, Current: {self.boat_state.compass_heading:.1f}, "
                            f"Controller: {self.controller.name}, Rudder: {control_command.rudder:.2f}, Sail: {control_command.sail:.2f}"
                        )
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")
            self._publish_health_status(False)

    def check_and_reload_params(self, is_initial=False):
        """Checks if the param file has changed and reloads it."""
        try:
            if not self.param_file.is_file():
                if self._last_param_mtime != 0:
                    self.get_logger().warn(
                        f"Parameter file '{self.param_file}' not found.")
                self._last_param_mtime = 0
                return

            mtime = self.param_file.stat().st_mtime
            if mtime > self._last_param_mtime:
                self.get_logger().info(
                    f"Parameter file '{self.param_file}' changed, reloading...")
                self._last_param_mtime = mtime

                with open(self.param_file, 'r') as f:
                    data = yaml.safe_load(f)

                node_name = self.get_name()
                if node_name in data and 'ros__parameters' in data[node_name]:
                    params_to_set = []
                    new_params = data[node_name]['ros__parameters']
                    for name, value in new_params.items():
                        if self.has_parameter(name):
                            params_to_set.append(Parameter(name, value=value))

                    if params_to_set:
                        self.set_parameters(params_to_set)

                        # Reinitialize controller if type changed
                        if any(p.name == 'controller_type' for p in params_to_set):
                            self._initialize_controller()
                        elif self.controller:  # Update existing controller config
                            config_update = {p.name: p.value for p in params_to_set
                                             if p.name in ['rudder_gain', 'rudder_full_scale_deg']}
                            if config_update:
                                self.controller.update_config(config_update)
                else:
                    self.get_logger().warn(
                        f"Could not find parameters for node '{node_name}' in '{self.param_file}'.")

        except Exception as e:
            self.get_logger().error(f"Error reloading parameters: {e}")


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Controller Node - High-level autonomous control for Argo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
This ROS2 node implements high-level autonomous control for the Argo sailboat:

ARCHITECTURE:
- Subscribes to all sensor data (GPS, IMU, wind, compass)
- Implements modular controller architecture with swappable algorithms
- Publishes autonomous commands to rudder_sail_radio.py for execution
- Handles data collection during human control for training

CONTROLLERS:
- ProportionalHeadingController: Simple heading maintenance
- WindAwareController: Enhanced with wind-based sail control
- Easily extensible for new control algorithms

TOPICS:
  Publishes:
    /rudder_sail_cmd: Vector3 - Autonomous control commands to rudder_sail_radio.py

  Subscribes:
    /human_controlled: Bool - Control authority status from rudder_sail_radio.py
    /control_authority: Vector3 - Detailed control status
    /rudder_sail_radio: Vector3 - Radio input reference
    /pose: Vector3 - IMU data (compass heading in z)
    /gps_*, /accel, /gyro, /compass, /anem_speed_angle_temp - Sensor data

PARAMETERS:
  controller_type: 'proportional' or 'wind_aware' (default: proportional)  
  data_collection_enabled: true/false (default: false)
  rudder_gain, rudder_full_scale_deg: controller parameters

CONTROL FLOW:
1. Sensor data updates boat state
2. During human control: Update target heading, collect training data
3. During robot control: Generate autonomous commands via controller.generate_control()
4. Commands sent to rudder_sail_radio.py for final arbitration and execution
        """
    )

    parsed_args, unknown_args = parser.parse_known_args(args)

    rclpy.init(args=unknown_args)
    controller_node = None
    try:
        controller_node = ControllerNode()
        rclpy.spin(controller_node)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt, shutting down controller...")
    except rclpy.executors.ExternalShutdownException:
        print("External shutdown signal received, exiting gracefully.")
    finally:
        try:
            if controller_node and hasattr(controller_node, 'data_collector'):
                controller_node.data_collector.stop_session()
        except Exception:
            pass  # Ignore errors during shutdown
        try:
            if controller_node:
                # Publish health status as failed on shutdown
                controller_node._publish_health_status(False)
                controller_node.destroy_node()
        except Exception:
            pass  # Ignore errors during shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Ignore errors during shutdown


if __name__ == '__main__':
    main()
