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
#      - Negative sign: positive error (target right of current) -> positive rudder (starboard) -> turn right
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
#     /controller/captains_log (String) - Captain's log entries for analysis and visualization
#     /controller/state (String) - Controller state (e.g., "broad_reach", "tacking") for visualization
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
# CAPTAIN'S LOG SYSTEM:
#   Controllers can publish log entries and state information:
#   - self.log_entry(message, level="INFO") - Publish log entry to /controller/captains_log
#   - self.publish_state(state_name) - Publish state to /controller/state for visualization
#   - self.log_periodic_state(state, interval_sec=30.0) - Log periodic boat/environmental state
#   Log entries are formatted as "[LEVEL] message" and can be visualized in Foxglove
#   and recorded in bag files for post-analysis.
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
#   - Positive error (target right) -> positive rudder (starboard) -> turn right
#   - Clamped to [-1, +1] range
#   - Target heading set during human control, maintained during autonomous control
#
# SENSOR PROCESSING AND FUSION:
#   Common sensor processing is centralized in controller.py to avoid duplication:
#   - Speed estimation: GPS position-based fallback when GPS SOG is stale (0.2 Hz updates)
#   - Future: Compass-GPS heading fusion (compass fast/low-noise, GPS absolute reference)
#   - Future: Absolute wind direction calculation (centralized, added to BoatState)
#   - Future: Sensor filtering and outlier rejection
#   
#   Controllers should use processed/fused values from BoatState rather than
#   duplicating filtering or fusion logic. This ensures consistency and reduces
#   code duplication across controllers.
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
from geofence_manager import GeofenceManager
import threading
import json
from typing import Optional, Dict, Any, List, Tuple
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
from rcl_interfaces.msg import SetParametersResult


# Add controllers package to path and import controllers/types
sys.path.append(os.path.join(os.path.dirname(__file__), 'controllers'))
from controllers import (
    ProportionalHeadingController,
    WindAwareController,
    ReturnToHomeController,
    PatrolController,
    CrosserController,
    HumanController,
    BoatState,
    ControlCommand,
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
    
    # Grace period after startup to ignore I2C failures (allows nodes to start up)
    I2C_FAILURE_GRACE_PERIOD_SECONDS = 30.0

    def __init__(self, debug_mode=False):
        super().__init__('controller_node')
        self.get_logger().info('Controller node starting...')
        
        # Track startup time for grace period enforcement
        self._startup_time = time.time()
        
        # Set initial health status - controller is healthy when running
        self.set_healthy("Controller node running")
        
        # Set log level if debug mode enabled
        if debug_mode:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Initialize boolean pause service
        # Read initial pause state from parameter
        self.declare_parameter('controller_initially_paused', False)
        initial_pause_state = self.get_parameter('controller_initially_paused').get_parameter_value().bool_value
        self._is_paused = initial_pause_state
        
        self.pause_service = self.create_service(
            SetBool,
            f'{self.get_name()}/pause',
            self._handle_pause_service
        )
        
        # Publish pause state for other nodes to monitor
        self.pause_state_pub = self.create_publisher(
            Bool, '/controller_pause_state', 10
        )
        
        # Publish initial pause state
        self._publish_pause_state()
        if self._is_paused:
            self.get_logger().info("Controller started in PAUSED state (manual control)")
        else:
            self.get_logger().info("Controller started in UNPAUSED state (autonomous control)")

        # Service for controller switching (legacy - kept for backwards compatibility)
        # Controller switching now happens automatically via parameter change callback.
        # The web dashboard only needs to set the 'controller_type' parameter.
        self.switch_controller_service = self.create_service(
            Trigger,
            f'{self.get_name()}/switch_controller',
            self._handle_switch_controller
        )

        # --- Parameters ---
        self.declare_parameter('param_file_path', 'argo.yaml')
        self.declare_parameter('controller_type', 'proportional')
        self.declare_parameter('data_collection_enabled', False)
        # Simulation parameters (from /** namespace)
        self.declare_parameter('simulation.grounding_behavior', 'terminate')
        self.declare_parameter('training_data_dir', '')  # Empty = use default
        self.declare_parameter('rudder_gain', 1.0)
        self.declare_parameter('rudder_full_scale_deg', 60.0)
        self.declare_parameter('p_heading_gain', 0.3)  # Proportional gain for heading control (P controller, can be extended to PID)
        # Allow disabling param file monitoring
        self.declare_parameter('enable_param_reload', True)
        # Patrol controller parameters
        self.declare_parameter('patrol_lookahead_time', 15.0)
        self.declare_parameter('boundary_turn_threshold', 15.0)
        self.declare_parameter('arrival_distance_m', 10.0)  # Crosser: middle waypoint; RTH: distance to home to consider "arrived" (meters)
        self.declare_parameter('tack_angle', 90.0)
        self.declare_parameter('tack_min_angle_from_wind', 50.0)  # Minimum angle from wind during tack to avoid stays
        self.declare_parameter('min_rudder_near_boundary', 0.3)  # Minimum rudder command when close to boundary
        self.declare_parameter('boundary_emergency_threshold', 5.0)  # Meters from boundary for emergency turn
        self.declare_parameter('turn_rudder_gain_multiplier', 2.0)  # Multiplier for rudder gain during turns (more aggressive)
        self.declare_parameter('broad_reach_angle', 110.0)
        self.declare_parameter('no_go_zone_angle', 45.0)  # Degrees from wind where sailing is impossible
        self.declare_parameter('geofence_map_name', 'Argo Irchel pond sailing area')
        # CrosserController only — tacking_upwind beat timing (ignored by other controllers)
        self.declare_parameter('upwind_layline_angle_deg', 50.0)
        self.declare_parameter('tack_cooldown_s', 12.0)
        self.declare_parameter('tacking_upwind_on_target_deg', 15.0)
        self.declare_parameter('tacking_upwind_stable_before_switch_s', 4.0)

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
        
        # GPS speed estimation tracking (for fallback when GPS SOG is stale)
        self._last_gps_position_timestamp = None
        self._last_gps_sog_timestamp = 0.0
        
        # Sensor fusion state (for future compass-GPS heading fusion)
        # TODO: Implement compass-GPS heading fusion when boat is moving forward
        # Compass heading is faster and less noisy than GPS COG, and is reliable
        # when boat is moving forward (hull and rudder provide good heading reference)
        self._last_compass_heading = None
        self._last_compass_timestamp = None
        self._last_gps_cog = None
        self._last_gps_cog_timestamp = None
        # Future: fused_heading will combine compass (fast, low noise) with GPS COG (absolute reference)
        # For now, controllers use compass_heading directly

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
        
        # Add parameter change callback to automatically switch controller when parameter changes
        self.add_on_set_parameters_callback(self._on_parameters_set)
        
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
        
        # Geofence and grounding state (for simulator bridge and monitoring)
        # Bool and Float64 are already imported at top of file
        self.pub_geofence_distance = self.create_publisher(
            Float64, '/geofence/distance_to_boundary', 10)
        self.pub_geofence_violation = self.create_publisher(
            Bool, '/geofence/violation', 10)
        self.pub_grounding = self.create_publisher(
            Bool, '/grounding/detected', 10)

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
        # Subscribe to true wind direction from bridge for verification/consistency
        # Float64 is already imported at top of file
        self.create_subscription(Float64, '/simulator/true_wind_direction', self.true_wind_callback, 10)
        self.true_wind_direction_from_bridge = None
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
        
        # Critical I2C failure monitoring (triggers automatic RTH)
        self.create_subscription(
            Bool, '/argo/critical/i2c_failure', self.i2c_failure_callback, 10)
        self._i2c_failure_active = False
        self._rth_switched_due_to_i2c = False

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

    def _initialize_controller(self, controller_type: str = None):
        """Initialize the controller based on parameters.
        
        Args:
            controller_type: Controller type to initialize. If None, reads from parameter.
        """
        if controller_type is None:
            controller_type = self.get_parameter(
                'controller_type').get_parameter_value().string_value
        # Normalize to handle both parameter values and direct calls
        controller_type = controller_type.strip().lower() if isinstance(controller_type, str) else str(controller_type).strip().lower()

        config = {
            'rudder_gain': self.get_parameter('rudder_gain').get_parameter_value().double_value,
            'rudder_full_scale_deg': self.get_parameter('rudder_full_scale_deg').get_parameter_value().double_value,
        }

        # Get logger and parent node reference to pass to controllers
        logger = self.get_logger()
        parent_node = self  # Pass reference to this ControllerNode
        
        if controller_type == 'proportional':
            self.controller = ProportionalHeadingController(config, logger=logger, parent_node=parent_node)
        elif controller_type == 'wind_aware':
            config['sail_wind_gain'] = 0.5  # Could be a parameter
            self.controller = WindAwareController(config, logger=logger, parent_node=parent_node)
        elif controller_type == 'return_to_home':
            config['sail_wind_gain'] = 0.5
            config['shore_connection_timeout'] = 120.0
            arrival_m = self.get_parameter('arrival_distance_m').get_parameter_value().double_value
            config['arrival_distance_m'] = arrival_m
            config['resume_navigation_distance_m'] = max(arrival_m * 1.5, arrival_m + 8.0)
            self.controller = ReturnToHomeController(config, logger=logger, parent_node=parent_node)
        elif controller_type == 'patrol':
            config['sail_wind_gain'] = 0.5
            config['patrol_lookahead_time'] = self.get_parameter('patrol_lookahead_time').get_parameter_value().double_value
            config['boundary_turn_threshold'] = self.get_parameter('boundary_turn_threshold').get_parameter_value().double_value
            config['tack_angle'] = self.get_parameter('tack_angle').get_parameter_value().double_value
            config['broad_reach_angle'] = self.get_parameter('broad_reach_angle').get_parameter_value().double_value
            config['geofence_map_name'] = self.get_parameter('geofence_map_name').get_parameter_value().string_value
            self.controller = PatrolController(config, logger=logger, parent_node=parent_node)
        elif controller_type == 'crosser':
            config['sail_wind_gain'] = 0.5
            config['p_heading_gain'] = self.get_parameter('p_heading_gain').get_parameter_value().double_value if self.has_parameter('p_heading_gain') else 0.3
            config['arrival_distance_m'] = self.get_parameter('arrival_distance_m').get_parameter_value().double_value if self.has_parameter('arrival_distance_m') else 10.0
            config['boundary_turn_threshold'] = self.get_parameter('boundary_turn_threshold').get_parameter_value().double_value if self.has_parameter('boundary_turn_threshold') else 15.0
            config['patrol_lookahead_time'] = self.get_parameter('patrol_lookahead_time').get_parameter_value().double_value if self.has_parameter('patrol_lookahead_time') else 20.0
            config['tack_angle'] = self.get_parameter('tack_angle').get_parameter_value().double_value if self.has_parameter('tack_angle') else 90.0
            config['tack_min_angle_from_wind'] = self.get_parameter('tack_min_angle_from_wind').get_parameter_value().double_value if self.has_parameter('tack_min_angle_from_wind') else 50.0
            config['no_go_zone_angle'] = self.get_parameter('no_go_zone_angle').get_parameter_value().double_value if self.has_parameter('no_go_zone_angle') else 45.0
            config['upwind_layline_angle_deg'] = self.get_parameter('upwind_layline_angle_deg').get_parameter_value().double_value if self.has_parameter('upwind_layline_angle_deg') else 50.0
            config['tack_cooldown_s'] = self.get_parameter('tack_cooldown_s').get_parameter_value().double_value if self.has_parameter('tack_cooldown_s') else 12.0
            config['tacking_upwind_on_target_deg'] = self.get_parameter('tacking_upwind_on_target_deg').get_parameter_value().double_value if self.has_parameter('tacking_upwind_on_target_deg') else 15.0
            config['tacking_upwind_stable_before_switch_s'] = (
                self.get_parameter('tacking_upwind_stable_before_switch_s').get_parameter_value().double_value
                if self.has_parameter('tacking_upwind_stable_before_switch_s') else 4.0)
            config['min_rudder_near_boundary'] = self.get_parameter('min_rudder_near_boundary').get_parameter_value().double_value if self.has_parameter('min_rudder_near_boundary') else 0.3
            config['boundary_emergency_threshold'] = self.get_parameter('boundary_emergency_threshold').get_parameter_value().double_value if self.has_parameter('boundary_emergency_threshold') else 5.0
            config['turn_rudder_gain_multiplier'] = self.get_parameter('turn_rudder_gain_multiplier').get_parameter_value().double_value if self.has_parameter('turn_rudder_gain_multiplier') else 2.0
            config['geofence_map_name'] = self.get_parameter('geofence_map_name').get_parameter_value().string_value
            self.controller = CrosserController(config, logger=logger, parent_node=parent_node)
        elif controller_type == 'human':
            # Human controller does nothing - allows full manual control
            self.controller = HumanController(config, logger=logger, parent_node=parent_node)
        else:
            self.get_logger().warn(
                f"Unknown controller type '{controller_type}', using proportional")
            self.controller = ProportionalHeadingController(config, logger=logger, parent_node=parent_node)

        self.get_logger().info(
            f"Initialized controller: {self.controller.name}")
        
        # Call reset() to ensure controller is properly initialized
        # This is especially important for HumanController to publish release_servos=True
        if self.controller:
            self.controller.reset()

    def switch_controller(self, controller_type: str = None):
        """Switch to a different controller type.
        
        Args:
            controller_type: Controller type to switch to. If None, reads from parameter.
        
        Note: When called from parameter callback, pass the controller_type directly
        to avoid reading stale parameter value.
        """
        old_controller = self.controller.name if self.controller else "None"

        # Reinitialize controller with the new type
        # If controller_type is provided, use it directly; otherwise read from parameter
        if controller_type is not None:
            self._initialize_controller(controller_type=controller_type)
        else:
            self._initialize_controller()

        if self.controller:
            self.controller.reset()

        self.get_logger().info(
            f"Switched controller from {old_controller} to {self.controller.name}")
    
    def _on_parameters_set(self, params):
        """Callback for parameter changes - automatically switches controller when controller_type changes."""
        result = SetParametersResult()
        result.successful = True
        
        for param in params:
            if param.name == 'controller_type' and param.type_ == Parameter.Type.STRING:
                # Get the NEW parameter value from the callback (this is the value being set)
                # param.value can be either a string directly or a ParameterValue object
                # Try string_value first (ParameterValue object), then fall back to direct value
                if hasattr(param.value, 'string_value'):
                    new_controller_type = param.value.string_value.strip().lower()
                elif isinstance(param.value, str):
                    new_controller_type = param.value.strip().lower()
                else:
                    new_controller_type = str(param.value).strip().lower()
                
                self.get_logger().debug(f"Parameter callback: controller_type set to '{new_controller_type}' (current: {self.controller.name if self.controller else 'None'})")
                
                # Validate controller type
                valid_types = ['proportional', 'wind_aware', 'return_to_home', 'patrol', 'crosser', 'human']
                if new_controller_type not in valid_types:
                    result.successful = False
                    result.reason = f"Invalid controller type '{new_controller_type}'. Valid types: {', '.join(valid_types)}"
                    self.get_logger().warn(result.reason)
                    return result
                
                # Check if controller is already the requested type
                current_controller_name = self.controller.name if self.controller else "None"
                if (new_controller_type == 'proportional' and current_controller_name == 'ProportionalHeadingController') or \
                   (new_controller_type == 'wind_aware' and current_controller_name == 'WindAwareController') or \
                   (new_controller_type == 'return_to_home' and current_controller_name == 'ReturnToHomeController') or \
                   (new_controller_type == 'patrol' and current_controller_name == 'PatrolController') or \
                   (new_controller_type == 'crosser' and current_controller_name == 'CrosserController') or \
                   (new_controller_type == 'human' and current_controller_name == 'HumanController'):
                    self.get_logger().debug(f"Controller is already {current_controller_name} (requested: {new_controller_type}), no switch needed")
                    return result
                
                # Switch to the new controller type
                # Pass new_controller_type directly to avoid reading parameter (which might be stale)
                old_controller = current_controller_name
                try:
                    self.switch_controller(controller_type=new_controller_type)
                    self.get_logger().info(f"Controller automatically switched from {old_controller} to {self.controller.name} via parameter change (new type: {new_controller_type})")
                except Exception as e:
                    result.successful = False
                    result.reason = f"Error switching controller: {str(e)}"
                    self.get_logger().error(result.reason)
                    return result
        
        return result
    
    def _handle_switch_controller(self, request, response):
        """Service handler for switching controller types (legacy - now handled by parameter callback).
        
        This service is kept for backwards compatibility, but controller switching
        now happens automatically via parameter change callback when the parameter is set.
        """
        try:
            # Read the current controller_type parameter value
            controller_type_param = self.get_parameter('controller_type')
            controller_type = controller_type_param.get_parameter_value().string_value.strip().lower()
            
            current_controller_name = self.controller.name if self.controller else "None"
            response.success = True
            response.message = f"Controller is {current_controller_name}. To switch, set the 'controller_type' parameter."
            self.get_logger().info(f"Switch controller service called - current: {current_controller_name}")
            
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
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
            
            # If human just took control, notify active controller if it supports
            # takeover/reset handling (e.g. patrol, crosser, future stateful controllers).
            if self.boat_state.human_controlled and self.controller is not None:
                on_human_control_started = getattr(self.controller, 'on_human_control_started', None)
                if callable(on_human_control_started):
                    try:
                        on_human_control_started(self.boat_state)
                    except Exception as e:
                        self.get_logger().warn(
                            f"Controller {self.controller.name} on_human_control_started() failed: {e}"
                        )

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

    def true_wind_callback(self, msg):
        """Receive true wind direction from bridge (compass convention, where wind comes from)."""
        self.true_wind_direction_from_bridge = msg.data
        # Pass true wind to controllers that interpret anem wind (sim: absolute; hardware: relative)
        if isinstance(self.controller, (PatrolController, CrosserController)):
            self.controller.true_wind_direction_from_bridge = msg.data
    
    def pose_callback(self, msg):
        if self.is_paused():
            return
        # Convert from mathematical convention (0°=East, CCW) to compass convention (0°=North, CW)
        # Same conversion as visualization: compass = (450.0 - math) % 360.0
        heading_math = float(msg.z) % 360.0
        compass_heading = (450.0 - heading_math) % 360.0
        
        # Track compass heading for sensor fusion
        self._last_compass_heading = compass_heading
        self._last_compass_timestamp = time.time()
        
        self.boat_state.compass_heading = compass_heading

    def gps_cog_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.gps_cog = msg.data
        
        # Track GPS COG for sensor fusion
        self._last_gps_cog = msg.data
        self._last_gps_cog_timestamp = time.time()

    def gps_sog_callback(self, msg):
        if self.is_paused():
            return
        self.boat_state.gps_sog = msg.data
        # Track timestamp of GPS SOG update (updates at ~0.2 Hz, so may be stale)
        self._last_gps_sog_timestamp = time.time()

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
        # In simulation: bridge publishes ABSOLUTE wind direction (0-360, compass, where wind comes from)
        # On real boat: anemometer publishes RELATIVE wind angle (0-360, CW from front of boat)
        # The controller will detect which convention is being used based on context
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
    
    def i2c_failure_callback(self, msg):
        """Receive critical I2C failure status - automatically switch to RTH if failure detected"""
        # Always process critical failures, even when paused
        i2c_failed = msg.data
        old_state = self._i2c_failure_active
        
        # Check if we're still in the grace period after startup
        time_since_startup = time.time() - self._startup_time
        in_grace_period = time_since_startup < self.I2C_FAILURE_GRACE_PERIOD_SECONDS
        
        if i2c_failed and not old_state:
            # I2C failure just detected
            if in_grace_period:
                # Ignore I2C failures during startup grace period (nodes may still be starting)
                remaining_grace = self.I2C_FAILURE_GRACE_PERIOD_SECONDS - time_since_startup
                self.get_logger().debug(
                    f"I2C failure detected during startup grace period (ignoring, {remaining_grace:.1f}s remaining). "
                    f"This is normal during node startup.")
                # Don't update _i2c_failure_active state during grace period
                return
            else:
                # I2C failure detected after grace period - switch to RTH immediately
                self._i2c_failure_active = i2c_failed
                self.get_logger().error(
                    "🔴 CRITICAL I2C BUS FAILURE DETECTED - Battery/wind sensors unavailable. "
                    "Automatically switching to Return-to-Home mode for safety.")
                
                # Switch to RTH controller if not already in RTH mode
                if not isinstance(self.controller, ReturnToHomeController):
                    old_controller_name = self.controller.name if self.controller else "None"
                    self.switch_controller('return_to_home')
                    self._rth_switched_due_to_i2c = True
                    self.get_logger().error(
                        f"Controller automatically switched from {old_controller_name} to ReturnToHomeController "
                        f"due to critical I2C bus failure")
                else:
                    self.get_logger().info("Already in RTH mode - maintaining RTH due to I2C failure")
        
        elif not i2c_failed and old_state:
            # I2C recovery detected
            self._i2c_failure_active = i2c_failed
            self.get_logger().info(
                "✅ I2C BUS RECOVERY DETECTED - Critical sensors restored. "
                "Controller will remain in current mode (RTH or manual switch required to exit RTH).")
            # Note: We don't automatically switch away from RTH - user must manually switch
            # This prevents oscillation if I2C is intermittent

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
        
        # Track previous position for speed estimation (fallback when GPS SOG is stale)
        prev_lat = self.boat_state.current_latitude
        prev_lon = self.boat_state.current_longitude
        prev_timestamp = getattr(self, '_last_gps_position_timestamp', None)
        
        # Update current position
        self.boat_state.current_latitude = msg.latitude
        self.boat_state.current_longitude = msg.longitude
        current_timestamp = time.time()
        
        # Estimate speed from position changes as fallback (GPS position updates at 1 Hz vs SOG at 0.2 Hz)
        # This provides more timely speed data for safety-critical boundary prediction
        if (prev_lat is not None and prev_lon is not None and prev_timestamp is not None and
            current_timestamp > prev_timestamp):
            dt = current_timestamp - prev_timestamp
            if dt > 0.1:  # At least 0.1s between updates (avoid division by zero)
                # Calculate distance using haversine formula
                lat1 = math.radians(prev_lat)
                lon1 = math.radians(prev_lon)
                lat2 = math.radians(msg.latitude)
                lon2 = math.radians(msg.longitude)
                dlon = lon2 - lon1
                a = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
                c = 2 * math.asin(math.sqrt(a))
                distance_m = 6378137.0 * c  # Earth radius in meters
                
                speed_ms = distance_m / dt
                speed_knots = speed_ms / 0.514444  # Convert m/s to knots
                
                # Store estimated speed (will be used as fallback if GPS SOG is stale)
                # Only update if GPS SOG is None or very stale (>3 seconds old)
                if self.boat_state.gps_sog is None:
                    self.boat_state.gps_sog = speed_knots
                else:
                    # Check if GPS SOG is stale (last update > 3 seconds ago)
                    last_sog_update = getattr(self, '_last_gps_sog_timestamp', 0.0)
                    if current_timestamp - last_sog_update > 3.0:
                        # GPS SOG is stale - use position-based estimate
                        self.boat_state.gps_sog = speed_knots
        
        self._last_gps_position_timestamp = current_timestamp
        
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
        try:
            # Check if ROS2 context is still valid before processing
            if not is_context_valid(self):
                return
                
            # Publish controller state even when paused or in human mode (for dashboard display)
            if self.controller:
                state_msg = String(data=self.controller.name)
                safe_publish(self.pub_controller_state, state_msg, self)
            
            # Check if node is paused - when paused, skip all processing and logging
            # EXCEPTION: HumanController must still run to publish release_servos=True
            # This ensures servos stay in high impedance mode for human control
            is_human_controller = isinstance(self.controller, HumanController)
            
            if self.is_paused() and not is_human_controller:
                # When paused, no autonomous commands are published, which causes
                # rudder_sail_radio.py to default to human control due to stale auto commands.
                # This effectively hands control back to the human operator immediately.
                # Also prevents redundant logging (captain's log and terminal log) when paused.
                return  # Skip all processing and logging when paused

            self.boat_state.timestamp = time.time()
            
            # Process sensor data (speed estimation, fusion, etc.) before controller uses state
            # This centralizes sensor processing so controllers don't need to duplicate logic
            self._process_sensor_data()

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
            # EXCEPTION: HumanController doesn't need compass_heading - it just passes through radio commands
            if self.boat_state.compass_heading is None and not is_human_controller:
                self.get_logger().debug("Waiting for initial compass heading...", throttle_duration_sec=5)
                return

            if self.boat_state.human_controlled:
                # Human control mode - update target heading for when robot takes over
                # Only update if compass_heading is available (not needed for HumanController)
                if self.boat_state.compass_heading is not None:
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

                # CRITICAL: HumanController must run even in human control mode to publish release_servos=True
                # This ensures servos stay in high impedance mode for direct radio control
                if isinstance(self.controller, HumanController):
                    if self.controller is not None:
                        control_command = self.controller.generate_control(self.boat_state)
                        # Publish the command (HumanController passes through radio commands)
                        if control_command:
                            safe_publish(self.pub_rudder_sail_cmd, control_command.to_vector3(), self)
                        else:
                            self.get_logger().warn("HumanController.generate_control() returned None")
                    else:
                        self.get_logger().warn("HumanController instance is None")
                else:
                    # No autonomous command published - rudder_sail_radio.py handles human input
                    self.get_logger().debug(f"Controller is not HumanController (type: {type(self.controller).__name__}) - no command published")
                    pass

            else:
                # Autonomous control mode
                # For patrol controller, it sets target_heading internally, so allow it to run even if None initially
                # For other controllers, target_heading must be set (from human control handoff)
                if self.boat_state.target_heading is None:
                    if isinstance(self.controller, PatrolController):
                        # Patrol controller will set target_heading in generate_control()
                        # Initialize to current heading as fallback
                        if self.boat_state.compass_heading is not None:
                            self.boat_state.target_heading = self.boat_state.compass_heading
                            self.get_logger().info(f"Patrol controller: Initializing target_heading to current heading {self.boat_state.target_heading:.1f}°")
                        else:
                            self.get_logger().warn("Patrol controller: No compass heading available yet, waiting...",
                                                   throttle_duration_sec=5)
                            return
                    else:
                        # Other controllers need target_heading from human control handoff
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
                        rth_arrival_m = self.get_parameter('arrival_distance_m').get_parameter_value().double_value
                        rth_config = {
                            'rudder_gain': self.get_parameter('rudder_gain').get_parameter_value().double_value,
                            'rudder_full_scale_deg': self.get_parameter('rudder_full_scale_deg').get_parameter_value().double_value,
                            'sail_wind_gain': 0.5,
                            'shore_connection_timeout': 120.0,
                            'arrival_distance_m': rth_arrival_m,
                            'resume_navigation_distance_m': max(rth_arrival_m * 1.5, rth_arrival_m + 8.0),
                        }
                        temp_rth = ReturnToHomeController(rth_config, logger=self.get_logger(), parent_node=self)
                        if temp_rth.should_activate(self.boat_state):
                            self.get_logger().warn(
                                "🏠 Switching to RETURN TO HOME mode due to connectivity loss",
                                throttle_duration_sec=5)
                            self.controller = temp_rth

                # **THE CORE ARCHITECTURE: Single generate_control function**
                # (Pause check already done at start of timer_callback - no need to check again)
                # CRITICAL: HumanController must always run to publish release_servos=True
                # This ensures servos stay in high impedance mode even after human control timeout
                control_command = self.controller.generate_control(self.boat_state)

                # Publish control command to rudder_sail_radio.py
                if control_command:
                    safe_publish(self.pub_rudder_sail_cmd, control_command.to_vector3(), self)
                
                # Note: Controller state is already published at the start of timer_callback
                # so it's available even when in human mode or paused
                
                # Only format debug string if debug logging is enabled (CPU optimization)
                if self.get_logger().get_effective_level() <= 10:  # DEBUG level
                    debug_msg = (f"Target: {self.boat_state.target_heading:.1f}, Current: {self.boat_state.compass_heading:.1f}, "
                                f"Controller: {self.controller.name}, Rudder: {control_command.rudder:.2f}, Sail: {control_command.sail:.2f}")
                    safe_log(self, 'debug', debug_msg)
        
        except Exception as e:
            # Log fatal errors but let the exception propagate
            # The lifecycle manager will detect the node crash and terminate the simulation
            error_msg = f"FATAL ERROR in controller timer_callback: {e}"
            self.get_logger().error(error_msg)
            import traceback
            tb_str = ''.join(traceback.format_exception(type(e), e, e.__traceback__))
            self.get_logger().error(f"Traceback:\n{tb_str}")
            
            # Log to captain's log for visibility (even when paused - fatal errors are important)
            if self.controller:
                # Fatal errors should always be logged, even when paused
                self.controller.log_entry(f"FATAL ERROR: {e}", level="ERROR")
            
            # Re-raise the exception to let the node crash
            # The lifecycle manager will detect the node failure and terminate the simulation
            raise

    def _process_sensor_data(self):
        """
        Centralized sensor processing and fusion for all controllers.
        
        This method processes raw sensor data and prepares fused/estimated values
        that controllers can use. Common processing includes:
        - Speed estimation from GPS position (fallback when GPS SOG is stale)
        - Future: Compass-GPS heading fusion (compass is faster/less noisy, GPS provides absolute reference)
        - Future: Wind direction calculations (absolute vs relative)
        - Future: Sensor filtering and outlier rejection
        
        This centralizes sensor processing so controllers don't need to duplicate
        filtering, fusion, or estimation logic.
        """
        # Speed estimation from GPS position is already handled in gps_position_callback
        # (runs before this method is called)
        
        # TODO: Compass-GPS heading fusion
        # When boat is moving forward (speed > threshold), compass heading is reliable
        # and should be fused with GPS COG for better heading estimate:
        # - Compass: Fast updates (~10 Hz), low noise, but can drift
        # - GPS COG: Slow updates (~0.2 Hz), absolute reference, but noisy at low speeds
        # - Fusion: Use compass for fast updates, GPS COG for calibration/drift correction
        # 
        # Implementation approach:
        # 1. When speed > threshold (e.g., 0.5 m/s): compass is reliable
        # 2. Use compass heading as primary (fast, low noise)
        # 3. Periodically correct compass drift using GPS COG (when GPS COG is fresh)
        # 4. When speed < threshold: rely more on GPS COG (compass less reliable when stationary)
        #
        # For now: Controllers use compass_heading directly (no fusion)
        # Future: Add fused_heading field to BoatState and populate it here
        
        # TODO: Absolute wind direction calculation
        # Some controllers calculate absolute wind from relative wind + heading
        # This should be centralized here and added to BoatState
        # Current: Controllers calculate this individually (crosser.py, patrol.py)
        # Future: Calculate once here and add to BoatState.absolute_wind_direction
        
        pass
    
    def _calculate_absolute_wind_direction(self, state: BoatState) -> Optional[float]:
        """
        Calculate absolute wind direction from relative wind angle and boat heading.
        
        This is a common calculation used by multiple controllers, so it's centralized here.
        Controllers can use this method instead of duplicating the calculation.
        
        Args:
            state: BoatState with wind_angle (relative) and compass_heading
            
        Returns:
            Absolute wind direction in degrees (0-360, compass convention, where wind comes from),
            or None if calculation not possible
        """
        if state.wind_angle is None or state.compass_heading is None:
            return None
        
        # wind_angle is relative to boat heading (0° = wind from front, 180° = wind from back)
        # Convert to absolute: absolute_wind = (compass_heading + wind_angle) % 360.0
        absolute_wind = (state.compass_heading + state.wind_angle) % 360.0
        return absolute_wind

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
