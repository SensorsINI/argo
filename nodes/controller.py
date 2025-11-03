#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
#
# controller.py - High-Level Autonomous Controller for Argo Sailboat
# ====================================================================
#
# ARCHITECTURE OVERVIEW:
#   This node implements a modular, swappable controller architecture for autonomous
#   sailboat control. It aggregates all sensor data into a unified BoatState, runs
#   the active controller algorithm, and publishes control commands to the rudder/sail
#   control system.
#
# CONTROL FLOW:
#   1. Sensor callbacks update BoatState (GPS, IMU, wind, compass, battery, LoRa)
#   2. During human control: Target heading tracks current heading for smooth handoff
#   3. During autonomous control: Active controller generates commands via generate_control()
#   4. Commands published to /rudder_sail_cmd for execution by rudder_sail_radio.py
#   5. Control arbitration in rudder_sail_radio.py decides human vs robot authority
#
# PAUSE MODE:
#   - When paused (via /controller_node/pause service or single button tap):
#     * No autonomous commands are published
#     * rudder_sail_radio.py automatically defaults to human control (stale auto commands)
#     * Controller resumes from current heading when unpaused
#     * Pause state published to /controller_pause_state topic for monitoring
#
# AVAILABLE CONTROLLERS:
#   1. ProportionalHeadingController (DEFAULT):
#      - Simple proportional rudder control to maintain heading
#      - Target heading = last human-controlled heading at handoff
#      - Rudder command proportional to heading error (target - current)
#      - Sail command passes through from radio input
#      - Good for: Basic autonomous heading hold, testing, development
#
#   2. WindAwareController:
#      - Enhanced proportional heading control with wind-based sail control
#      - Automatically adjusts sail position based on apparent wind angle
#      - Rudder control same as proportional controller
#      - Good for: More realistic sailing behavior, wind-relative control
#
#   3. ReturnToHomeController:
#      - GPS-based navigation to return to starting position
#      - Activates on shore connection loss or explicit LoRa command
#      - Calculates bearing to home and steers toward it
#      - Wind-aware sail control for efficient return
#      - Good for: Emergency return, lost connection scenarios, automated return
#
# CONTROLLER SWITCHING:
#   - Via parameter: Set 'controller_type' in argo.yaml ('proportional', 'wind_aware', 'return_to_home')
#   - Via service: Call /controller_node/switch_controller (Trigger service)
#   - Automatic: RTH controller activates on connection loss timeout
#   - Hot-reload: Parameter file changes detected and applied automatically
#
# TARGET HEADING BEHAVIOR:
#   - During human control: target_heading continuously updated to current compass heading
#   - On handoff to autonomous: target_heading frozen at handoff value
#   - Robot steers to maintain this target heading using active controller
#   - This provides smooth, predictable handoff behavior
#
# DATA COLLECTION:
#   - Optional training data collection during human control
#   - Records state-action pairs for machine learning
#   - Enable via 'data_collection_enabled' parameter
#   - Saved to ~/argo_data/training_data/
#
# KEY TOPICS:
#   Published:
#     /rudder_sail_cmd (Vector3) - Autonomous control commands (-1 to +1)
#     /controller_state (String) - Current controller name for monitoring
#     /controller_pause_state (Bool) - Current pause state for system monitoring
#
#   Subscribed:
#     /human_controlled (Bool) - Control authority from rudder_sail_radio.py
#     /rudder_sail_radio (Vector3) - Radio input for reference
#     /pose (Vector3) - Compass heading (z component)
#     /gps_cog, /gps_sog, /gps_velocity - GPS navigation data
#     /accel, /gyro, /magnetometer - IMU sensor data
#     /anem_speed_angle_temp (Vector3) - Wind sensor data
#     /battery_voltage, /battery_low_alert - Battery monitoring
#     /lora_connection_status, /lora_remote_command - LoRa connectivity
#     /fix (NavSatFix) - GPS position for home tracking
#
# PARAMETERS:
#   controller_type: 'proportional' (default), 'wind_aware', 'return_to_home'
#   rudder_gain: Proportional gain for rudder control (default: 1.0)
#   rudder_full_scale_deg: Heading error for full rudder deflection (default: 60.0°)
#   data_collection_enabled: Enable training data recording (default: false)
#   enable_param_reload: Hot-reload parameter file changes (default: true)
#
# SERVICES:
#   /controller_node/pause (SetBool) - Pause/unpause controller (True=pause, False=unpause, None=query)
#   /controller_node/switch_controller (Trigger) - Switch to return-to-home controller
#
# EXTENDING THE CONTROLLER:
#   To add a new controller:
#   1. Create a new class inheriting from BaseController
#   2. Implement generate_control(state: BoatState) -> ControlCommand
#   3. Add initialization case in _initialize_controller()
#   4. Add controller_type string to parameter handling
#
# CURRENT DEFAULT CONTROLLER: ProportionalHeadingController
#   - Proportional rudder control to steer back to last human-controlled heading
#   - Formula: rudder_cmd = rudder_gain * (target_heading - current_heading) / rudder_full_scale_deg
#   - Clamped to [-1, +1] range
#   - Target heading set during human control, maintained during autonomous control
#
# SAFE PUBLISHING:
#   - Uses safe_publish utility to prevent "publisher's context is invalid" errors
#   - Validates ROS2 context before all publishing operations
#   - Graceful shutdown with proper timer cancellation
#   - Context validation in all timer callbacks

import sys
import os

# Remove old pause service import - implementing new boolean pause service directly
import psutil

# Import safe publishing utilities and ArgoBaseNode
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'support'))
from safe_publish import safe_publish, safe_log, is_context_valid
import threading
import json
from typing import Optional, Dict, Any, List
from abc import ABC, abstractmethod
from dataclasses import dataclass, asdict
import numpy as np
import time
import math
from pathlib import Path
import argparse
import argcomplete
import yaml
import rclpy
# Removed QoS imports - using default QoS only
from std_msgs.msg import Bool, Float64, Float32, String
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger, SetBool
from rclpy.parameter import Parameter



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

    # Battery/Water monitoring (from argo_battery_water node with persistent QoS)
    battery_voltage: Optional[float] = None      # volts
    battery_remaining_pct: Optional[float] = None  # percentage
    battery_low_alert: bool = False              # low battery alert
    saltwater_alert: bool = False                # saltwater intrusion alert
    humidity_alert: bool = False                 # high humidity alert

    # Connectivity monitoring (from lora node)
    shore_connected: bool = False                # LoRa connection to shore
    last_shore_contact: Optional[float] = None   # timestamp of last contact
    remote_command: Optional[str] = None         # latest remote command

    # Home position tracking (for return-to-home)
    home_latitude: Optional[float] = None        # degrees
    home_longitude: Optional[float] = None       # degrees
    current_latitude: Optional[float] = None     # degrees
    current_longitude: Optional[float] = None    # degrees
    return_to_home_active: bool = False          # RTH mode active

    def is_valid_for_control(self) -> bool:
        """Check if we have minimum required data for autonomous control."""
        return (self.compass_heading is not None and
                self.target_heading is not None)

    def has_critical_alerts(self) -> bool:
        """Check if any critical alerts are active that should affect control."""
        return self.battery_low_alert or self.saltwater_alert

    def get_bearing_to_home(self) -> Optional[float]:
        """Calculate bearing from current position to home position in degrees."""
        if (self.home_latitude is None or self.home_longitude is None or
            self.current_latitude is None or self.current_longitude is None):
            return None

        # Convert to radians
        lat1 = math.radians(self.current_latitude)
        lon1 = math.radians(self.current_longitude)
        lat2 = math.radians(self.home_latitude)
        lon2 = math.radians(self.home_longitude)

        # Calculate bearing using Haversine formula
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        
        # Convert to degrees (0-360)
        bearing_deg = (math.degrees(bearing) + 360) % 360
        return bearing_deg

    def get_distance_to_home(self) -> Optional[float]:
        """Calculate distance to home position in nautical miles."""
        if (self.home_latitude is None or self.home_longitude is None or
            self.current_latitude is None or self.current_longitude is None):
            return None

        # Convert to radians
        lat1 = math.radians(self.current_latitude)
        lon1 = math.radians(self.current_longitude)
        lat2 = math.radians(self.home_latitude)
        lon2 = math.radians(self.home_longitude)

        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        # Radius of earth in nautical miles = 3440.065
        distance_nm = 3440.065 * c
        return distance_nm

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


class ReturnToHomeController(BaseController):
    """
    Return-to-home controller that navigates back to starting position.
    
    Activates automatically when:
    1. Shore connection is lost AND return_to_home_active flag is set, OR
    2. Remote "return_home" command is received via LoRa
    
    Uses GPS-based navigation to calculate bearing to home and sail toward it.
    """

    def __init__(self, config: Dict[str, Any]):
        super().__init__(config)
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)
        self.connection_timeout = config.get('shore_connection_timeout', 120.0)  # seconds
        self.arrival_distance = config.get('arrival_distance_nm', 0.05)  # 0.05nm = ~90 meters
        
    def should_activate(self, state: BoatState) -> bool:
        """
        Determine if return-to-home mode should activate.
        
        Returns True if:
        - RTH explicitly commanded via LoRa, OR
        - Shore connection lost for more than timeout period
        """
        # Explicit RTH command
        if state.return_to_home_active:
            return True
        
        # Automatic RTH on connection loss
        if state.last_shore_contact is not None:
            time_since_contact = time.time() - state.last_shore_contact
            if time_since_contact > self.connection_timeout and not state.shore_connected:
                return True
        
        return False

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate control commands to navigate toward home position."""
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)

        # Check if we have GPS home position
        bearing_to_home = state.get_bearing_to_home()
        distance_to_home = state.get_distance_to_home()
        
        if bearing_to_home is None or distance_to_home is None:
            # Fall back to maintaining current heading
            compass_err = signed_angle_difference_degrees(
                state.target_heading, state.compass_heading)
            cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
            cmd_rudder = np.clip(cmd_rudder, -1.0, 1.0)
            cmd_sail = 0.0  # Neutral sail
            return ControlCommand(
                rudder=cmd_rudder,
                sail=cmd_sail,
                timestamp=state.timestamp
            )
        
        # Check if we've arrived at home
        if distance_to_home < self.arrival_distance:
            # Arrived! Switch to drift mode with neutral controls
            return ControlCommand(
                rudder=0.0,
                sail=0.0,
                timestamp=state.timestamp
            )
        
        # Navigate toward home bearing
        # Use bearing to home as target heading
        state.target_heading = bearing_to_home
        
        compass_err = signed_angle_difference_degrees(
            bearing_to_home, state.compass_heading)
        cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = np.clip(cmd_rudder, -1.0, 1.0)
        
        # Wind-aware sail control (same as WindAwareController)
        cmd_sail = 0.0
        if state.wind_angle is not None:
            wind_sail_cmd = (state.wind_angle - 90.0) / 90.0
            wind_sail_cmd = np.clip(wind_sail_cmd, -1.0, 1.0)
            cmd_sail = self.sail_wind_gain * wind_sail_cmd
        
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


class ControllerNode(ArgoBaseNode):
    """High-level autonomous controller node."""

    def __init__(self, debug_mode=False):
        super().__init__('controller_node')
        self.get_logger().info('Controller node starting...')
        
        # Set initial health status - controller is healthy when running
        self.set_healthy("Controller node running")
        
        # Set log level if debug mode enabled
        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Initialize boolean pause service
        self._is_paused = False
        self.pause_service = self.create_service(
            SetBool,
            f'{self.get_name()}/pause',
            self._handle_pause_service
        )
        
        # Publish pause state for other nodes to monitor
        self.pause_state_pub = self.create_publisher(
            Bool, '/controller_pause_state', 10
        )

        # Service for controller switching (for web dashboard)
        self.switch_controller_service = self.create_service(
            Trigger,
            f'{self.get_name()}/switch_controller',
            self._handle_switch_controller
        )

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

        # --- Using default QoS for all publishers/subscribers ---

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
        self.cpu_monitor_enabled = False
        self.last_cpu_check = time.time()
        self.cpu_check_interval = 30.0  # Check CPU usage every 30 seconds
        if self.cpu_monitor_enabled:
            self.get_logger().info(
                f"CPU monitoring enabled: True (check interval: {self.cpu_check_interval}s)"
            )
        else:
            self.get_logger().info(
                "CPU monitoring disabled. To enable, set the parameter 'self.cpu_monitor_enabled' to True."
            )

        # Initialize controller
        self._initialize_controller()
        
        # Publish initial pause state
        self._publish_pause_state()
        
        # Create timer to publish pause state periodically (every 5 seconds)
        self.pause_state_timer = self.create_timer(5.0, self._publish_pause_state)

        # --- Publishers ---
        # Real-time control commands (use default QoS)
        self.pub_rudder_sail_cmd = self.create_publisher(
            Vector3, '/rudder_sail_cmd', 10)
        
        # Controller state for web dashboard
        self.pub_controller_state = self.create_publisher(
            String, '/controller_state', 10)

        # --- Subscribers ---
        # Control status from rudder_sail_radio.py (use default QoS)
        self.create_subscription(
            Bool, '/human_controlled', self.human_control_callback, 10)
        self.create_subscription(
            Vector3, '/control_authority', self.control_authority_callback, 10)

        # Radio input for reference (real-time data)
        self.create_subscription(
            Vector3, '/rudder_sail_radio', self.radio_reference_callback, 10)

        # Navigation sensors (real-time data)
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(
            Float64, '/gps_cog', self.gps_cog_callback, 10)
        self.create_subscription(
            Float64, '/gps_sog', self.gps_sog_callback, 10)
        self.create_subscription(
            Vector3, '/gps_velocity', self.gps_velocity_callback, 10)

        # IMU sensors (real-time data)
        self.create_subscription(Vector3, '/accel', self.accel_callback, 10)
        self.create_subscription(Vector3, '/gyro', self.gyro_callback, 10)
        self.create_subscription(
            Vector3, '/magnetometer', self.compass_callback, 10)

        # Wind sensor (real-time data)
        self.create_subscription(
            Vector3, '/anem_speed_angle_temp', self.wind_callback, 10)

        # Battery/Water monitoring (use default QoS)
        self.create_subscription(
            Float32, '/battery_voltage', self.battery_voltage_callback, 10)
        self.create_subscription(
            Float32, '/battery_remaining_pct', self.battery_remaining_callback, 10)
        self.create_subscription(
            Bool, '/battery_low_alert', self.battery_low_alert_callback, 10)
        self.create_subscription(
            Bool, '/saltwater_alert', self.saltwater_alert_callback, 10)
        self.create_subscription(
            Bool, '/humidity_alert', self.humidity_alert_callback, 10)

        # LoRa connectivity monitoring (for return-to-home)
        self.create_subscription(
            Bool, '/lora_connection_status', self.lora_connection_callback, 10)
        self.create_subscription(
            String, '/lora_remote_command', self.lora_command_callback, 10)

        # GPS position for home tracking and navigation
        self.create_subscription(
            NavSatFix, '/fix', self.gps_position_callback, 10)

        # --- Timers ---
        # 5 Hz (reduced from 10 Hz for lower CPU usage)
        self.control_loop_period = 0.2
        self.timer = self.create_timer(
            self.control_loop_period, self.timer_callback)

        # Only create parameter reload timer if enabled (CPU optimization)
        if self.get_parameter('enable_param_reload').get_parameter_value().bool_value:
            # Reduced frequency from 3s to 10s for lower CPU usage
            self.param_reload_check_period = 10.0
            self.param_timer = self.create_timer(
                self.param_reload_check_period, self.check_and_reload_params)
        else:
            self.param_timer = None
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
        elif controller_type == 'return_to_home':
            config['sail_wind_gain'] = 0.5
            config['shore_connection_timeout'] = 120.0
            config['arrival_distance_nm'] = 0.05
            self.controller = ReturnToHomeController(config)
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
    
    def _handle_switch_controller(self, request, response):
        """Service handler for switching controller types via web dashboard."""
        try:
            # For Trigger service, we'll use a workaround since it doesn't have a data field
            # The web dashboard should pass controller type via a parameter or separate topic
            # For now, we'll cycle through controller types or default to RTH
            # TODO: Implement proper service type with string parameter
            
            # Default to return_to_home for emergency RTH button
            controller_type = 'return_to_home'
            
            old_controller = self.controller.name if self.controller else "None"
            self.switch_controller(controller_type)
            
            response.success = True
            response.message = f"Switched from {old_controller} to {self.controller.name}"
            self.get_logger().info(f"Controller switched via service: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error switching controller: {str(e)}"
            self.get_logger().error(response.message)
        
        return response

    def _handle_pause_service(self, request, response):
        """
        Handle boolean pause service requests.
        
        Args:
            request.data: True = pause, False = unpause, None = return current state
            request.success: Not used in SetBool service
        
        Returns:
            response.success: True if operation succeeded
            response.message: Status message
        """
        try:
            if request.data is True:
                # Pause requested
                if not self._is_paused:
                    self._is_paused = True
                    self.get_logger().info("Controller PAUSED - human control takes over")
                    response.success = True
                    response.message = "Controller paused successfully"
                else:
                    response.success = True
                    response.message = "Controller already paused"
            elif request.data is False:
                # Unpause requested
                if self._is_paused:
                    self._is_paused = False
                    self.get_logger().info("Controller UNPAUSED - autonomous control resumed")
                    response.success = True
                    response.message = "Controller unpaused successfully"
                else:
                    response.success = True
                    response.message = "Controller already unpaused"
            else:
                # None - return current state
                response.success = True
                response.message = f"Controller is {'paused' if self._is_paused else 'unpaused'}"
            
            # Publish current pause state
            self._publish_pause_state()
            
        except Exception as e:
            self.get_logger().error(f"Error handling pause service: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"
        
        return response

    def _publish_pause_state(self, timer=None):
        """Publish current pause state for other nodes to monitor."""
        pause_msg = Bool()
        pause_msg.data = self._is_paused
        
        if safe_publish(self.pause_state_pub, pause_msg, self):
            safe_log(self, 'debug', f"Published pause state: {self._is_paused}")

    def is_paused(self) -> bool:
        """Check if the controller is currently paused."""
        return self._is_paused

    # --- Sensor Callbacks ---
    def human_control_callback(self, msg):
        """Receive control authority status from rudder_sail_radio.py."""
        # Always process control authority changes, even when paused
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
        if self.is_paused():
            return
        self.boat_state.compass_heading = msg.z

    def gps_cog_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.gps_cog = msg.data

    def gps_sog_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.gps_sog = msg.data

    def gps_velocity_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.gps_velocity = msg

    def accel_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.accel = msg

    def gyro_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.gyro = msg

    def compass_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.compass_raw = msg

    def wind_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.wind_speed = msg.x
        self.boat_state.wind_angle = msg.y
        self.boat_state.wind_temp = msg.z

    # --- Battery/Water Monitoring Callbacks ---
    def battery_voltage_callback(self, msg):
        """Receive battery voltage from argo_battery_water node"""
        # Always process battery data for safety, even when paused
        self.boat_state.battery_voltage = msg.data

    def battery_remaining_callback(self, msg):
        """Receive battery remaining percentage from argo_battery_water node"""
        # Always process battery data for safety, even when paused
        self.boat_state.battery_remaining_pct = msg.data

    def battery_low_alert_callback(self, msg):
        """Receive battery low alert from argo_battery_water node"""
        # Always process safety alerts, even when paused
        self.boat_state.battery_low_alert = msg.data
        if msg.data:
            self.get_logger().warn("🔋 LOW BATTERY ALERT - Consider returning to shore")

    def saltwater_alert_callback(self, msg):
        """Receive saltwater intrusion alert from argo_battery_water node"""
        # Always process safety alerts, even when paused
        self.boat_state.saltwater_alert = msg.data
        if msg.data:
            self.get_logger().warn("💧 SALTWATER INTRUSION ALERT - Check hull integrity")

    def humidity_alert_callback(self, msg):
        """Receive humidity alert from argo_battery_water node"""
        # Always process safety alerts, even when paused
        self.boat_state.humidity_alert = msg.data
        if msg.data:
            self.get_logger().warn("💦 HIGH HUMIDITY ALERT - Check ventilation")

    # --- LoRa Connectivity Callbacks ---
    def lora_connection_callback(self, msg):
        """Receive LoRa connection status from lora node"""
        # Always process connectivity status, even when paused
        old_status = self.boat_state.shore_connected
        self.boat_state.shore_connected = msg.data
        
        if msg.data:
            self.boat_state.last_shore_contact = time.time()
            if not old_status:
                self.get_logger().info("📡 Shore connection ESTABLISHED via LoRa")
        else:
            if old_status:
                self.get_logger().warn("📡 Shore connection LOST - Return-to-home may activate")

    def lora_command_callback(self, msg):
        """Receive remote commands from shore via LoRa"""
        # Always process remote commands, even when paused
        command = msg.data.lower().strip()
        self.boat_state.remote_command = command
        
        self.get_logger().info(f"📡 Received remote command: '{command}'")
        
        # Process return-to-home command
        if command == 'return_home':
            if self.boat_state.home_latitude is not None and self.boat_state.home_longitude is not None:
                self.boat_state.return_to_home_active = True
                bearing = self.boat_state.get_bearing_to_home()
                distance = self.boat_state.get_distance_to_home()
                self.get_logger().info(
                    f"🏠 RETURN TO HOME activated - Bearing: {bearing:.1f}°, Distance: {distance:.2f}nm")
            else:
                self.get_logger().warn("🏠 Cannot return home - no home position set")
        elif command == 'autonomous':
            self.boat_state.return_to_home_active = False
            self.get_logger().info("🤖 Normal autonomous operation resumed")
        elif command == 'stop':
            # Set neutral commands - implementation can be enhanced
            self.get_logger().warn("⚠️ STOP command received - consider implementing safe stop mode")

    def gps_position_callback(self, msg):
        """Receive GPS position from gps node"""
        if self.is_paused():
            return
        
        # Update current position
        self.boat_state.current_latitude = msg.latitude
        self.boat_state.current_longitude = msg.longitude
        
        # Set home position on first valid GPS fix
        if (self.boat_state.home_latitude is None and 
            self.boat_state.home_longitude is None and
            msg.latitude != 0.0 and msg.longitude != 0.0):
            self.boat_state.home_latitude = msg.latitude
            self.boat_state.home_longitude = msg.longitude
            self.get_logger().info(
                f"🏠 Home position set: {msg.latitude:.6f}°, {msg.longitude:.6f}°")

    def timer_callback(self):
        """Main control loop - generates autonomous control commands."""
        # Check if ROS2 context is still valid before processing
        if not is_context_valid(self):
            return
            
        # Check if node is paused - when paused, unconditionally default to human control
        if self.is_paused():
            # When paused, no autonomous commands are published, which causes
            # rudder_sail_radio.py to default to human control due to stale auto commands.
            # This effectively hands control back to the human operator immediately.
            return  # Skip processing when paused

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

            # Check if return-to-home should activate
            if isinstance(self.controller, ReturnToHomeController):
                if self.controller.should_activate(self.boat_state):
                    # Log RTH status periodically
                    distance = self.boat_state.get_distance_to_home()
                    bearing = self.boat_state.get_bearing_to_home()
                    if distance is not None and bearing is not None:
                        self.get_logger().info(
                            f"🏠 RETURN TO HOME - Distance: {distance:.2f}nm, Bearing: {bearing:.1f}°, "
                            f"Current heading: {self.boat_state.compass_heading:.1f}°",
                            throttle_duration_sec=10)
            elif hasattr(self.controller, 'should_activate'):
                # For any controller with should_activate method, check if RTH needed
                if isinstance(self.controller, ReturnToHomeController) == False:
                    # Create temporary RTH controller to check
                    rth_config = {
                        'rudder_gain': self.get_parameter('rudder_gain').get_parameter_value().double_value,
                        'rudder_full_scale_deg': self.get_parameter('rudder_full_scale_deg').get_parameter_value().double_value,
                        'sail_wind_gain': 0.5,
                        'shore_connection_timeout': 120.0,
                        'arrival_distance_nm': 0.05
                    }
                    temp_rth = ReturnToHomeController(rth_config)
                    if temp_rth.should_activate(self.boat_state):
                        self.get_logger().warn(
                            "🏠 Switching to RETURN TO HOME mode due to connectivity loss",
                            throttle_duration_sec=5)
                        self.controller = temp_rth

            # **THE CORE ARCHITECTURE: Single generate_control function**
            control_command = self.controller.generate_control(self.boat_state)

            # Publish control command to rudder_sail_radio.py
            if control_command:
                safe_publish(self.pub_rudder_sail_cmd, control_command.to_vector3(), self)
            
            # Publish current controller state for web dashboard
            if self.controller:
                state_msg = String(data=self.controller.name)
                safe_publish(self.pub_controller_state, state_msg, self)

                # Only format debug string if debug logging is enabled (CPU optimization)
                if self.get_logger().get_effective_level() <= 10:  # DEBUG level
                    debug_msg = (f"Target: {self.boat_state.target_heading:.1f}, Current: {self.boat_state.compass_heading:.1f}, "
                                f"Controller: {self.controller.name}, Rudder: {control_command.rudder:.2f}, Sail: {control_command.sail:.2f}")
                    safe_log(self, 'debug', debug_msg)

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

    def _cleanup_on_exit(self):
        """Override ArgoBaseNode cleanup to stop data collection session"""
        try:
            if hasattr(self, 'data_collector'):
                self.data_collector.stop_session()
        except Exception:
            pass  # Ignore errors during shutdown


def main(args=None):
    parser = ArgoBaseNode.create_standard_parser(
        description='Controller Node - High-level autonomous control for Argo',
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
    /controller_node_health: Bool - Node health status (ArgoBaseNode)

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

HEALTH MONITORING:
  - Service: ros2 service call /controller_node/health std_srvs/srv/Trigger
  - Topic: ros2 topic echo /controller_node_health
  - Healthy when controller is running (currently always healthy when running)
        """
    )

    # Use ArgoBaseNode's standardized runner
    ArgoBaseNode.run_node(ControllerNode, args, parser)


if __name__ == '__main__':
    main()
