#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
BNO085 IMU Management Tool
==========================

Single unified tool for managing the BNO085 9-axis IMU sensor.

ARCHITECTURE
------------
The BNO085 system consists of two components:
  1. C++ Driver (bno08x_driver) - Hardware interface via I2C
     - Communicates with BNO085 sensor at 0x4a on I2C bus 0
     - Publishes standard ROS2 sensor_msgs (Imu, MagneticField)
     
  2. Python Bridge (this tool in 'bridge' mode) - Data conversion
     - Converts sensor_msgs to Argo Vector3 format
     - Publishes to /compass, /pose, /accel, /gyro topics
     - Monitors health and publishes /imu_health status

SENSOR FEATURES
---------------
The BNO085 provides:
  • Rotation Vector (datasheet 2.2.4): Quaternion orientation with on-chip
    sensor fusion (accelerometer + gyroscope + magnetometer)
  • Magnetic north reference: Compass heading corrected for tilt
  • Dynamic calibration: Auto-calibrates during operation, saves to flash
  • High accuracy: ±0.01° heading stability when calibrated

LAUNCHING
---------
The BNO085 is automatically launched by the Argo lifecycle manager.
It can also be launched manually:

  1. Via Launch File (recommended):
     cd ~/argo/nodes
     make bno08x-launch-full
     # Launches both C++ driver and Python bridge

  2. Via Lifecycle Manager:
     python3 ~/argo/launch/argo_lifecycle_manager.py run
     # Discovers and launches bno085.py in bridge mode

  3. Standalone Bridge (if driver already running):
     bno085.py bridge
     # Runs as persistent ROS2 node

TOPICS PUBLISHED
----------------
  /compass      (Vector3) - Heading in degrees (z=0-360, 0=N, 90=E, compass CW from North)
  /pose         (Vector3) - Mathematical yaw in degrees on z (0°=East, CCW), matching
                 argo_unified_simulator_bridge and argo_transform_publisher (ENU)
  /accel        (Vector3) - Acceleration in g, ROS base_link (X=fwd, Y=port, Z=up)
  /gyro         (Vector3) - Angular velocity in degrees/second
  /imu_health   (Bool)    - Health status (true=healthy, false=unhealthy)

CALIBRATION
-----------
The BNO085 uses unified sensor fusion - all sensors calibrate together.
Enhanced interactive calibration with visual feedback:

  1. Real-time Visual Feedback:
     - Magnetometer min/max ranges displayed with progress bars
     - Coverage quality indicator (POOR → FAIR → GOOD → EXCELLENT)
     - Sensor accuracy levels (UNRELIABLE → LOW → MEDIUM → HIGH)
     
  2. Interactive Control:
     - Watch magnetometer ranges expand as you move the sensor
     - Press Ctrl+C when satisfied with calibration quality
     - No need to wait for timer - finish when ranges look good
     
  3. Magnetometer (most important):
     - Move sensor in figure-8 pattern through ALL orientations so that 
       each magnetometer axis is aligned with and opposite to the Earth's magnetic field.
     - Target: 40-60 µT range on each axis (X, Y, Z)
     - Monitor coverage quality reaching EXCELLENT (80%+)
     
  4. Accelerometer & Gyroscope (automatic):
     - Auto-calibrate during motion and stationary periods
     - Benefit from varied orientations during magnetometer calibration
     
  5. Calibration Persistence:
     - Automatically saved to sensor's internal flash memory
     - Persists across power cycles and reboots
     - No external calibration files needed
     
  Run calibration:
     bno085.py calibrate --duration 120
     # Interactive tool with visual range tracking

  Web dashboard (phone-friendly, no SSH):
     Open Argo Web Dashboard → IMU Compass Tools card
     # Same calibration/verify logic via /api/imu/calibration/* and /api/imu/verify/*

COMMANDS
--------
  bridge       Run as data bridge (for launch files/lifecycle manager)
               - Persistent ROS2 node
               - Converts C++ driver topics to Argo format
               - Publishes health status
               
  calibrate    Interactive calibration with visual feedback
               - Real-time magnetometer range visualization with bars
               - Coverage quality indicator (POOR → EXCELLENT)
               - Shows accuracy levels (UNRELIABLE → LOW → MEDIUM → HIGH)
               - Press Ctrl+C when satisfied with calibration
               - Comprehensive final report with statistics
               
  verify       Interactive ASCII display for sensor testing
               - Real-time visualization of all sensor data
               - Shows accelerometer, gyroscope, magnetometer bars
               - Displays compass heading with visual compass
               - Validates quaternion, Euler angles, heading stability
               - Perfect for interactive sensor testing and debugging
               
  status       Check BNO085 system health
               - Shows driver and bridge status
               - Lists topic availability
               - Displays health status
               - Checks I2C hardware detection

EXAMPLES
--------
  # Check system status
  bno085.py status
  
  # Calibrate sensor (interactive with visual feedback)
  bno085.py calibrate
  # Watch magnetometer ranges expand, press Ctrl+C when satisfied
  
  # Quick 1-minute calibration
  bno085.py calibrate --duration 60
  
  # Extended 5-minute calibration for best results
  bno085.py calibrate --duration 300
  # TIP: Monitor coverage quality - finish early if EXCELLENT
  
  # Interactive sensor display (10 seconds)
  bno085.py verify --duration 10
  
  # Extended interactive testing (60 seconds)
  bno085.py verify --duration 60
  
  # Run as bridge (normally done by launch file)
  bno085.py bridge

HEALTH MONITORING
-----------------
The bridge publishes /imu_health for lifecycle management (see BNO085Bridge class
constants for full tuning notes):
  • Healthy (true):  Sustained /imu stream — not stale, past post-gap cooldown,
    and stable for _HEALTH_STABLE_S (default 5 s)
  • Unhealthy (false): Stale /imu, post-gap cooldown, or stream not yet stable
  • Driver watchdog: separate C++ param watchdog.timeout_ms (default 5 s) — see
    vendor/bno08x_driver; journal "Watchdog timeout! No data received from sensor"
  • Auto-recovery: after startup grace, if no /imu for _RESTART_IF_STALE_S (~15 s),
    bridge runs ``sudo systemctl restart argo_bno085.service`` (throttled by
    _MIN_RESTART_INTERVAL_S ~45 s). Requires passwordless sudo for that command.

TROUBLESHOOTING
---------------
  No data in calibrate/verify:
    1. Check driver is running: bno085.py status
    2. Check topics: ros2 topic list | grep imu
    3. Restart system: make bno08x-launch-full
    
  Hardware not detected:
    1. Check I2C: i2cdetect -y 0
    2. Should show "4a" or "UU" at address 0x4a
    3. Check wiring and power
    
  Poor heading accuracy:
    1. Run calibration: bno085.py calibrate
    2. Ensure figure-8 motion covers all orientations
    3. Calibrate away from metal/electronics
    4. Allow 2-3 minutes for full calibration

QUICK START
-----------
  1. Build driver:      cd ~/argo/nodes && make bno08x-build
  2. Test hardware:     make bno08x-test
  3. Launch system:     make bno08x-launch-full
  4. Calibrate sensor:  make bno08x-calibrate
  5. Interactive display: python3 bno085.py verify

For more information, see:
  • nodes/BNO085_USAGE.md - Complete usage guide
  • BNO085_CALIBRATION_GUIDE.md - Detailed calibration instructions
  • BNO085_HEALTH_MONITORING.md - Health monitoring details
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import math
import re
import sys
import time
import json
import argparse
import argcomplete
import subprocess
from datetime import datetime
import collections
from pathlib import Path
from typing import Optional
import yaml

# ruamel.yaml is REQUIRED for safe parameter persistence (preserves YAML comments)
try:
    from ruamel.yaml import YAML
except ImportError as e:
    # Log FATAL error so lifecycle manager can display it
    import logging
    logging.basicConfig(level=logging.FATAL)
    logger = logging.getLogger('bno085')
    logger.fatal("FATAL: ruamel.yaml is required but not installed. Run: make install-python-deps")
    sys.exit(1)

# ============================================================================
# BRIDGE MODE: Convert C++ driver output to Argo format
# ============================================================================

class BNO085Bridge(Node):
    """Bridge node that converts BNO08x driver topics to Argo format with I2C error recovery."""

    # ---- Bridge health & recovery tuning (Python bno085_bridge only) ----
    #
    # NOT the C++ driver watchdog. That lives in bno08x_ros (vendor/bno08x_driver):
    #   watchdog.timeout_ms  default 5000 ms — fires when the hub sends no sensor
    #   reports; driver calls reset(). Override in the launch yaml (e.g.
    #   nodes/vendor/bno085_i2c_argo.yaml) if resets fire during slow I2C re-init.
    #   Journal line: "Watchdog timeout! No data received from sensor. Resetting..."
    #
    # Bridge logic runs on a 1 Hz timer (_check_health) plus imu_callback on each
    # /imu from the driver. Rough state flow:
    #   start UNHEALTHY → stream stable → HEALTHY → gap/stale → UNHEALTHY → cooldown → …
    #
    # _HEALTH_STALE_S — "no recent data"
    #   If time since last /imu exceeds this, bridge marks UNHEALTHY and clears the
    #   stable-period timer. At 20 Hz, nominal gap is ~50 ms; 3 s allows DDS jitter
    #   but still catches a dead stream quickly. Checked in _imu_stream_looks_healthy.
    #
    # _HEALTH_GAP_S — "stream broke" detector
    #   If gap between consecutive /imu messages exceeds this, treat as instability
    #   (driver watchdog reset, I2C stall, brief dropout). Starts cooldown and resets
    #   _healthy_candidate_since so one lucky message cannot flip HEALTHY immediately.
    #   Set above normal inter-message period (1/20 Hz = 0.05 s); 1 s is conservative.
    #
    # _HEALTH_STABLE_S — "sustained good stream" before HEALTHY
    #   After stream resumes (or on first boot), require this many seconds of continuous
    #   /imu before publishing healthy. Stops dashboard/ai2c showing HEALTHY while
    #   journal still logs repeated driver watchdog resets (~50 ms flip before 2026-06).
    #
    # _INSTABILITY_COOLDOWN_S — "don't trust recovery yet"
    #   After a gap > _HEALTH_GAP_S, block HEALTHY until this many seconds after the
    #   gap (wall clock). Messages during cooldown still update last_imu_time and feed
    #   _healthy_candidate_since, but _imu_stream_looks_healthy returns false until
    #   cooldown expires AND _HEALTH_STABLE_S of stable data has elapsed.
    #
    # _STARTUP_GRACE_S — "don't restart systemd during bring-up"
    #   Suppress sudo systemctl restart argo_bno085.service until bridge has been up
    #   this long (driver init + first reports can take tens of seconds).
    #
    # _RESTART_IF_STALE_S — "escalate to full service restart"
    #   After startup grace, if no /imu for this long, _attempt_recovery restarts the
    #   whole argo_bno085 unit (C++ driver + bridge). Works with _MIN_RESTART_INTERVAL_S.
    #
    # _MIN_RESTART_INTERVAL_S — restart throttle
    #   Minimum time between systemd restart attempts to avoid flapping on bad I2C.
    #
    # Inspection notes (2026-06): gap/cooldown/stable logic is newer; field-tested on
    # flaky BNO085 where driver watchdog still fires every ~15–90 s. imu_callback still
    # resets recovery_attempt_count on every /imu, so sporadic messages can delay systemd
    # restarts even when watchdog keeps firing — separate from /imu_health display logic.
    _HEALTH_STALE_S = 3.0
    _HEALTH_GAP_S = 1.0
    _HEALTH_STABLE_S = 5.0
    _INSTABILITY_COOLDOWN_S = 15.0
    _STARTUP_GRACE_S = 45.0
    _RESTART_IF_STALE_S = 15.0
    _MIN_RESTART_INTERVAL_S = 45.0

    def __init__(self):
        super().__init__('bno085_bridge')
        self._bridge_start_time = time.time()
        
        # Publishers (Argo format)
        self.pub_compass = self.create_publisher(Vector3, '/compass', 10)
        self.pub_pose = self.create_publisher(Vector3, '/pose', 10)
        self.pub_accel = self.create_publisher(Vector3, '/accel', 10)
        self.pub_gyro = self.create_publisher(Vector3, '/gyro', 10)
        self.pub_health = self.create_publisher(Bool, '/imu_health', 10)
        
        # Health service (for lifecycle manager queries)
        self.health_service = self.create_service(
            Trigger, '/bno085_bridge/health', self._handle_health_request)
        
        # Subscribers (from BNO08x driver)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.sub_mag = self.create_subscription(MagneticField, '/magnetic_field', self.mag_callback, 10)
        
        # Health monitoring - require sustained /imu before reporting healthy
        self.health_status = False  # Start unhealthy until we get driver data
        self.last_imu_time = None
        self._prev_imu_time = None
        self._healthy_candidate_since = None
        self._instability_cooldown_until = 0.0
        self.health_check_timer = self.create_timer(1.0, self._check_health)
        
        # Publish initial health status
        self._publish_health_status(False)
        
        # I2C error recovery tracking
        self.node_healthy = False  # Track if we're receiving data
        self.recovery_attempt_count = 0
        self.last_recovery_attempt_time = 0.0
        self.last_unreachable_log_time = 0.0
        self.consecutive_failures = 0
        self.total_errors_this_session = 0
        
        # Recovery mode flag (switches to low-frequency checks when unhealthy)
        self.in_recovery_mode = False
        
        self.get_logger().info("BNO085 Bridge: Converting C++ driver → Argo topics")
        self.get_logger().info("Publishing: /compass, /pose, /accel, /gyro, /imu_health")
        self.get_logger().info("I2C error recovery: Enabled with automatic C++ driver restart")

        # Determine Argo repo dir for subprocess calls
        script_path = Path(__file__).resolve()
        self.argo_dir = str(script_path.parents[1])  # nodes -> argo
        
        # IMU mount + trim from argo.yaml (fusion quaternion is already mag-north referenced)
        imu_cfg = self._load_imu_yaml_config()
        self.declare_parameter(
            'imu.mount_forward_axis',
            str(imu_cfg.get('mount_forward_axis', 'y')),
        )
        self.declare_parameter(
            'imu.mount_yaw_deg',
            float(imu_cfg.get('mount_yaw_deg', 0.0)),
        )
        self.declare_parameter(
            'imu.yaw_offset_deg',
            float(imu_cfg.get('yaw_offset_deg', 0.0)),
        )
        self.mount_forward_axis = (
            self.get_parameter('imu.mount_forward_axis')
            .get_parameter_value()
            .string_value
        )
        self.mount_yaw_deg = (
            self.get_parameter('imu.mount_yaw_deg')
            .get_parameter_value()
            .double_value
        )
        self.yaw_offset_deg = (
            self.get_parameter('imu.yaw_offset_deg')
            .get_parameter_value()
            .double_value
            % 360.0
        )

        # Allow runtime updates via parameter callback (yaw_offset persisted to argo.yaml)
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # If North/South are correct but East/West are swapped, enable this (negates east
        # component before atan2 — equivalent to compass = (360 - heading) % 360).
        self.declare_parameter(
            'imu.yaw_invert',
            self._parse_yaml_bool(imu_cfg.get('yaw_invert'), False),
        )
        self.yaw_invert = self._parse_yaml_bool(
            self.get_parameter('imu.yaw_invert').get_parameter_value().bool_value,
            False,
        )

        # Rotate accel/gyro XY so horizontal components match compass frame
        self.apply_axis_rotation = True

        self.get_logger().info(
            f'IMU heading: mount_forward_axis={self.mount_forward_axis}, '
            f'mount_yaw_deg={self.mount_yaw_deg:.1f}, '
            f'yaw_offset_deg={self.yaw_offset_deg:.1f} (trim only), '
            f'yaw_invert={self.yaw_invert}'
        )
    
    def _argo_yaml_path(self) -> Path:
        """Return absolute path to nodes/argo.yaml."""
        return Path(self.argo_dir) / 'nodes' / 'argo.yaml'
    
    def _load_imu_yaml_config(self) -> dict:
        """Load imu section from nodes/argo.yaml."""
        try:
            yaml_path = self._argo_yaml_path()
            if not yaml_path.exists():
                return {}
            with open(yaml_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            root = cfg.get('/**', {}).get('ros__parameters', {})
            return root.get('imu', {}) or {}
        except Exception:
            return {}

    @staticmethod
    def _parse_yaml_bool(value, default: bool = False) -> bool:
        """Parse bool from YAML/ROS (avoid bool('false') == True)."""
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)):
            return value != 0
        if isinstance(value, str):
            return value.strip().lower() in ('true', '1', 'yes', 'on')
        return bool(value)
    
    def _persist_yaw_offset_to_yaml(self, value: float) -> None:
        """Persist yaw offset into nodes/argo.yaml under /**/ros__parameters/imu.
        
        Uses ruamel.yaml to preserve comments and formatting.
        """
        try:
            yaml_path = self._argo_yaml_path()
            
            # Load existing YAML preserving comments
            rt_yaml = YAML()
            rt_yaml.preserve_quotes = True
            data = {}
            if yaml_path.exists():
                with open(yaml_path, 'r') as f:
                    data = rt_yaml.load(f) or {}
            
            # Ensure nested structure exists
            if '/**' not in data:
                data['/**'] = {}
            if 'ros__parameters' not in data['/**']:
                data['/**']['ros__parameters'] = {}
            if 'imu' not in data['/**']['ros__parameters']:
                data['/**']['ros__parameters']['imu'] = {}
            data['/**']['ros__parameters']['imu']['yaw_offset_deg'] = float(value)
            
            # Write atomically: tmp then replace
            tmp_path = yaml_path.with_suffix('.yaml.tmp')
            with open(tmp_path, 'w') as f:
                rt_yaml.dump(data, f)
            tmp_path.replace(yaml_path)
            
            self.get_logger().info(f'Persisted imu.yaw_offset_deg={value:.3f}° to {yaml_path} (comments preserved)')
        except Exception as e:
            self.get_logger().error(f'Failed to persist imu.yaw_offset_deg to argo.yaml: {e}')
            import traceback
            self.get_logger().debug(f'Traceback: {traceback.format_exc()}')
    
    def _yaw_invert_active(self) -> bool:
        """Read imu.yaw_invert from the parameter server (live; not cached)."""
        try:
            return self._parse_yaml_bool(
                self.get_parameter('imu.yaw_invert').get_parameter_value().bool_value,
                False,
            )
        except Exception:
            return bool(self.yaw_invert)

    def _on_parameter_change(self, parameters):
        """Handle runtime parameter updates and persist yaw offset changes."""
        from rcl_interfaces.msg import SetParametersResult
        result = SetParametersResult()
        result.successful = True
        for param in parameters:
            if param.name == 'imu.yaw_offset_deg':
                try:
                    new_val = param.get_parameter_value().double_value % 360.0
                    self.yaw_offset_deg = new_val
                    self._persist_yaw_offset_to_yaml(new_val)
                    self.get_logger().info(f'imu.yaw_offset_deg updated to {new_val:.3f}° (persisted)')
                except Exception as e:
                    result.successful = False
                    result.reason = f'Invalid imu.yaw_offset_deg: {e}'
            elif param.name == 'imu.mount_forward_axis':
                self.mount_forward_axis = param.get_parameter_value().string_value
            elif param.name == 'imu.mount_yaw_deg':
                self.mount_yaw_deg = param.get_parameter_value().double_value
            elif param.name == 'imu.yaw_invert':
                self.yaw_invert = self._parse_yaml_bool(
                    param.get_parameter_value().bool_value,
                    False,
                )
                self.get_logger().info(
                    f'imu.yaw_invert={self.yaw_invert} '
                    f'(E/W reflected: {"on" if self.yaw_invert else "off"})'
                )
        return result
    
    def _sensor_accel_to_base_link(self, sx: float, sy: float, sz: float):
        """Map BNO085 sensor accel (g) to ROS base_link (X=fwd, Y=port, Z=up).

        Default mount (mount_forward_axis=y): sensor +Y=bow, +X=starboard, +Z=up.
        """
        axis = str(self.mount_forward_axis).strip().lower()
        if axis in ('x', '+x'):
            return sx, -sy, sz
        if axis in ('neg_x', '-x'):
            return -sx, sy, sz
        if axis in ('neg_y', '-y'):
            return -sy, sx, sz
        # y / +y (default): bow along sensor +Y.
        # bx=-sy: accelerometer reads -gravity, bow-down → sensor_sy<0 → bx>0 (bow-down positive)
        # by=-sx: starboard is sensor +X, port is -starboard
        return -sy, -sx, sz

    def _sensor_gyro_to_base_link(self, sx: float, sy: float, sz: float):
        """Map BNO085 gyro (deg/s) to ROS base_link angular rates."""
        axis = str(self.mount_forward_axis).strip().lower()
        if axis in ('x', '+x'):
            return sy, -sx, sz
        if axis in ('neg_x', '-x'):
            return -sy, sx, sz
        if axis in ('neg_y', '-y'):
            return -sx, -sy, sz
        return sy, -sx, sz

    def _rotate_xy(self, x: float, y: float, degrees: float):
        """Rotate a 2D vector (x,y) in the XY plane by 'degrees' about +Z."""
        rad = math.radians(degrees)
        c = math.cos(rad)
        s = math.sin(rad)
        xr = c * x - s * y
        yr = s * x + c * y
        return xr, yr

    def _forward_axis_vector(self):
        """Unit vector for bow direction in the BNO085 sensor frame."""
        axis = str(self.mount_forward_axis).strip().lower()
        vectors = {
            'x': (1.0, 0.0, 0.0),
            'y': (0.0, 1.0, 0.0),
            'neg_x': (-1.0, 0.0, 0.0),
            'neg_y': (0.0, -1.0, 0.0),
            '-x': (-1.0, 0.0, 0.0),
            '-y': (0.0, -1.0, 0.0),
        }
        return vectors.get(axis, (0.0, 1.0, 0.0))

    def _quat_rotate_vector(self, w, x, y, z, vx, vy, vz):
        """Rotate vector v by unit quaternion q (Hamilton, active rotation)."""
        qx, qy, qz = float(x), float(y), float(z)
        qw = float(w)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)
        return (
            vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx),
        )

    def _heading_from_rotation_vector(self, w, x, y, z):
        """
        Compass heading from mag-referenced fusion quaternion.

        Rotates the configured bow axis into ENU and uses atan2(east, north).
        0°=North, 90°=East, clockwise from above (does not apply trim/invert).
        """
        fx, fy, fz = self._forward_axis_vector()
        if self.mount_yaw_deg != 0.0:
            rad = math.radians(self.mount_yaw_deg)
            c, s = math.cos(rad), math.sin(rad)
            fx, fy = c * fx - s * fy, s * fx + c * fy
        wx, wy, _wz = self._quat_rotate_vector(w, x, y, z, fx, fy, fz)
        heading = math.degrees(math.atan2(wx, wy))
        if heading < 0.0:
            heading += 360.0
        return heading

    def _apply_yaw_invert(self, heading_deg: float) -> float:
        """
        Mirror E↔W while keeping N/S: (360 - heading) % 360.

        Unchanged at 0°/180° — verify with bow toward E or W, not N.
        """
        if self._yaw_invert_active():
            return (360.0 - heading_deg) % 360.0
        return heading_deg
    
    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        
        if yaw_deg < 0:
            yaw_deg += 360.0
        
        return roll_deg, pitch_deg, yaw_deg
    
    def imu_callback(self, msg: Imu):
        """Process IMU message: extract heading, gyro, accel."""
        current_time = time.time()

        if self._prev_imu_time is not None:
            gap_s = current_time - self._prev_imu_time
            if gap_s > self._HEALTH_GAP_S:
                self._instability_cooldown_until = (
                    current_time + self._INSTABILITY_COOLDOWN_S
                )
                self._healthy_candidate_since = None

        self._prev_imu_time = current_time
        self.last_imu_time = current_time
        if self._healthy_candidate_since is None:
            self._healthy_candidate_since = current_time

        # Fresh /imu after recovery — reset so logs stay readable
        self.recovery_attempt_count = 0
        self.last_recovery_attempt_time = 0.0
        
        qw, qx, qy, qz = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        )
        # Bow heading from fusion quaternion + physical mount
        heading = self._heading_from_rotation_vector(qw, qx, qy, qz)
        heading = self._apply_yaw_invert(heading)
        heading_compass = (heading + self.yaw_offset_deg) % 360.0
        
        # /compass: compass convention (0=N, 90=E). /pose: math yaw for TF/controller (same as sim bridge).
        compass_msg = Vector3(x=0.0, y=0.0, z=heading_compass)
        self.pub_compass.publish(compass_msg)
        pose_msg = Vector3(
            x=0.0,
            y=0.0,
            z=(450.0 - heading_compass) % 360.0,
        )
        self.pub_pose.publish(pose_msg)
        
        # Publish gyro/accel in base_link (REP-103). Do not apply plane_align here —
        # that rotation is sensor-XY only and breaks roll/pitch after axis remap.
        gx, gy, gz = self._sensor_gyro_to_base_link(
            math.degrees(msg.angular_velocity.x),
            math.degrees(msg.angular_velocity.y),
            math.degrees(msg.angular_velocity.z),
        )
        self.pub_gyro.publish(Vector3(x=gx, y=gy, z=gz))

        ax, ay, az = (
            msg.linear_acceleration.x / 9.81,
            msg.linear_acceleration.y / 9.81,
            msg.linear_acceleration.z / 9.81,
        )
        bx, by, bz = self._sensor_accel_to_base_link(ax, ay, az)
        self.pub_accel.publish(Vector3(x=bx, y=by, z=bz))
    
    def mag_callback(self, msg: MagneticField):
        """Process magnetic field message (currently unused)."""
        pass
    
    def _imu_stream_looks_healthy(self, current_time: float) -> bool:
        """True only when /imu is fresh, stable long enough, and not in post-gap cooldown."""
        if self.last_imu_time is None:
            return False
        if (current_time - self.last_imu_time) > self._HEALTH_STALE_S:
            return False
        if current_time < self._instability_cooldown_until:
            return False
        if self._healthy_candidate_since is None:
            return False
        return (current_time - self._healthy_candidate_since) >= self._HEALTH_STABLE_S

    def _check_health(self):
        """Health check: sustained /imu required; restart unit if /imu dies long enough."""
        current_time = time.time()

        if self._imu_stream_looks_healthy(current_time):
            if not self.health_status:
                self._publish_health_status(True)
        else:
            if self.health_status:
                self._publish_health_status(False)
            if self.last_imu_time is None or (
                current_time - self.last_imu_time
            ) > self._HEALTH_STALE_S:
                self._healthy_candidate_since = None

        # Heartbeat /imu_health every timer tick so volatile late subscribers get state.
        # Heading streams on each /imu; health was only emitted on transitions, so nodes
        # that start later (web dashboard, restarts) could show UNKNOWN with live bearing.
        hb = Bool()
        hb.data = self.health_status
        self.pub_health.publish(hb)

        # Automatic recovery was previously never invoked (dead code). Restart the full
        # argo_bno085.service (launcher + C++ + bridge) when /imu has been gone/stale
        # long enough; throttle to avoid flapping. Requires passwordless sudo for
        # systemctl if the service user has no TTY.
        if current_time - self._bridge_start_time < self._STARTUP_GRACE_S:
            return
        if self.last_imu_time is None:
            imu_stale_s = None
        else:
            imu_stale_s = current_time - self.last_imu_time
        if imu_stale_s is not None and imu_stale_s < self._RESTART_IF_STALE_S:
            return
        # Never received /imu: wait until (startup grace + settle) before restarting a wedged driver
        if imu_stale_s is None:
            min_uptime = self._STARTUP_GRACE_S + self._RESTART_IF_STALE_S
            if current_time - self._bridge_start_time < min_uptime:
                return
            self._attempt_recovery(current_time)
            return
        self._attempt_recovery(current_time)
    
    def _publish_health_status(self, is_healthy: bool):
        """Publish health status update (logs on transitions; heartbeat uses _check_health)."""
        self.health_status = is_healthy
        health_msg = Bool()
        health_msg.data = is_healthy
        self.pub_health.publish(health_msg)
        
        if is_healthy:
            self.get_logger().info("BNO085 health: HEALTHY")
        else:
            self.get_logger().warn("BNO085 health: UNHEALTHY")
    
    def _handle_health_request(self, request, response):
        """Handle health status service request."""
        try:
            response.success = True
            response.message = json.dumps({
                'healthy': self.health_status,
                'details': f"BNO085 bridge health: {'HEALTHY' if self.health_status else 'UNHEALTHY'}",
                'timestamp': time.time(),
                'node_name': 'bno085_bridge'
            })
        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {e}"
        
        return response
    
    def _switch_to_recovery_mode(self):
        """Switch to low-frequency recovery mode (checking every 3 seconds)."""
        if not self.in_recovery_mode:
            self.in_recovery_mode = True
            self.health_check_timer.destroy()
            self.health_check_timer = self.create_timer(3.0, self._check_health)
            self.get_logger().info("Switched to recovery mode (3s check interval)")
    
    def _switch_to_normal_mode(self):
        """Switch back to normal health check frequency (1 second)."""
        if self.in_recovery_mode:
            self.in_recovery_mode = False
            self.health_check_timer.destroy()
            self.health_check_timer = self.create_timer(1.0, self._check_health)
            self.get_logger().info("Switched back to normal mode (1s check interval)")
    
    def _attempt_recovery(self, current_time):
        """Recover by restarting the whole argo_bno085 unit (systemd); kills this bridge too."""
        time_since_last_attempt = current_time - self.last_recovery_attempt_time
        if time_since_last_attempt < self._MIN_RESTART_INTERVAL_S:
            return

        self.recovery_attempt_count += 1
        self.last_recovery_attempt_time = current_time

        if self.last_imu_time is None:
            stale_msg = "never received /imu from driver"
        else:
            stale_msg = f"no /imu for {current_time - self.last_imu_time:.1f}s"

        time_since_last_log = current_time - self.last_unreachable_log_time
        if time_since_last_log >= 60.0 or self.last_unreachable_log_time == 0.0:
            self.get_logger().error(
                f"BNO085 IMU failing ({stale_msg}); "
                f"recovery attempt #{self.recovery_attempt_count} — restarting argo_bno085.service"
            )
            self.last_unreachable_log_time = current_time

        self.get_logger().info(
            f"Recovery #{self.recovery_attempt_count}: "
            f"sudo systemctl restart argo_bno085.service"
        )
        
        try:
            # Try to restart the C++ driver service using systemctl
            result = subprocess.run(
                ['sudo', 'systemctl', 'restart', 'argo_bno085.service'],
                capture_output=True,
                text=True,
                timeout=10
            )
            
            if result.returncode == 0:
                self.get_logger().info("argo_bno085.service restart command successful - waiting for data...")
            else:
                self.get_logger().warn(f"argo_bno085.service restart command failed: {result.stderr}")
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("Driver restart command timed out")
        except Exception as e:
            self.get_logger().error(f"Failed to restart driver: {e}")
    
    def _debug_parameters(self):
        """Debug function to check driver parameters."""
        try:
            cmd1 = (f'source /opt/ros/humble/setup.bash && '
                    f'source {self.argo_dir}/nodes/argo_bno08x_driver_workspace/install/setup.bash && '
                    f'ros2 param get /bno08x_ros publish.imu.rate')
            result = subprocess.run([
                'bash', '-c', cmd1
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info(f"DEBUG: Driver IMU rate parameter: {result.stdout.strip()}")
            else:
                self.get_logger().warn(f"DEBUG: Failed to get IMU rate parameter: {result.stderr}")
                
            # Also check magnetic field rate
            cmd2 = (f'source /opt/ros/humble/setup.bash && '
                    f'source {self.argo_dir}/nodes/argo_bno08x_driver_workspace/install/setup.bash && '
                    f'ros2 param get /bno08x_ros publish.magnetic_field.rate')
            result2 = subprocess.run([
                'bash', '-c', cmd2
            ], capture_output=True, text=True, timeout=5)
            
            if result2.returncode == 0:
                self.get_logger().info(f"DEBUG: Driver magnetic field rate parameter: {result2.stdout.strip()}")
            else:
                self.get_logger().warn(f"DEBUG: Failed to get magnetic field rate parameter: {result2.stderr}")
                
        except Exception as e:
            self.get_logger().error(f"DEBUG: Error checking parameters: {e}")


# ============================================================================
# CALIBRATION / VERIFY: Shared trackers (CLI + web dashboard)
# ============================================================================

ACCURACY_NAMES = ['UNRELIABLE', 'LOW', 'MEDIUM', 'HIGH']
TARGET_MAG_SPAN_UT = 50.0
TARGET_MAG_BAR_UT = 60.0


def cov_to_accuracy(cov: float) -> int:
    """Convert fusion covariance to accuracy level (0-3)."""
    if cov <= 0:
        return 0
    if cov < 0.01:
        return 3
    if cov < 0.05:
        return 2
    if cov < 0.1:
        return 1
    return 0


def _forward_axis_vector(mount_forward_axis: str):
    """Unit bow vector in BNO085 sensor frame (matches BNO085Bridge)."""
    axis = str(mount_forward_axis).strip().lower()
    vectors = {
        'x': (1.0, 0.0, 0.0),
        'y': (0.0, 1.0, 0.0),
        'neg_x': (-1.0, 0.0, 0.0),
        'neg_y': (0.0, -1.0, 0.0),
        '-x': (-1.0, 0.0, 0.0),
        '-y': (0.0, -1.0, 0.0),
    }
    return vectors.get(axis, (0.0, 1.0, 0.0))


def _quat_rotate_vector(w, x, y, z, vx, vy, vz):
    """Rotate vector v by unit quaternion q (Hamilton, active rotation)."""
    qx, qy, qz = float(x), float(y), float(z)
    qw = float(w)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def compass_heading_from_quaternion(
    w,
    x,
    y,
    z,
    mount_forward_axis: str = 'y',
    mount_yaw_deg: float = 0.0,
    yaw_offset_deg: float = 0.0,
    yaw_invert: bool = False,
) -> float:
    """
    Compass heading (0=N, 90=E, CW) from fusion quaternion — same math as BNO085Bridge.
    Use for low-latency verify UI directly from /imu (no /compass hop).
    """
    fx, fy, fz = _forward_axis_vector(mount_forward_axis)
    if mount_yaw_deg != 0.0:
        rad = math.radians(mount_yaw_deg)
        c, s = math.cos(rad), math.sin(rad)
        fx, fy = c * fx - s * fy, s * fx + c * fy
    wx, wy, _wz = _quat_rotate_vector(w, x, y, z, fx, fy, fz)
    heading = math.degrees(math.atan2(wx, wy))
    if heading < 0.0:
        heading += 360.0
    if yaw_invert:
        heading = (360.0 - heading) % 360.0
    return (heading + yaw_offset_deg) % 360.0


def _parse_yaml_bool(value, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def load_imu_heading_config(yaml_path=None) -> dict:
    """Load imu mount/trim keys from nodes/argo.yaml."""
    try:
        path = Path(yaml_path) if yaml_path else Path(__file__).resolve().parent / 'argo.yaml'
        if not path.exists():
            return {}
        with open(path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        root = cfg.get('/**', {}).get('ros__parameters', {})
        imu = root.get('imu', {}) or {}
        return {
            'mount_forward_axis': str(imu.get('mount_forward_axis', 'y')),
            'mount_yaw_deg': float(imu.get('mount_yaw_deg', 0.0)),
            'yaw_offset_deg': float(imu.get('yaw_offset_deg', 0.0)),
            'yaw_invert': _parse_yaml_bool(imu.get('yaw_invert'), False),
        }
    except Exception:
        return {}


def quaternion_to_euler_deg(w, x, y, z):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    if yaw_deg < 0:
        yaw_deg += 360.0
    return roll_deg, pitch_deg, yaw_deg


class ImuCalibrationTracker:
    """Non-ROS magnetometer calibration state (shared by CLI and web dashboard)."""

    def __init__(self, duration_s: int = 60, start_time: float = None):
        self.duration_s = int(duration_s)
        self.start_time = start_time if start_time is not None else time.time()
        self.mag_accuracy = self.accel_accuracy = self.gyro_accuracy = 0
        self.imu_sample_count = self.mag_sample_count = 0
        self.orientation_changes = 0
        self.last_orientation = None
        self.motion_detected = False
        self.mag_min_x = self.mag_min_y = self.mag_min_z = float('inf')
        self.mag_max_x = self.mag_max_y = self.mag_max_z = float('-inf')

    def on_imu(self, msg: Imu) -> None:
        self.imu_sample_count += 1
        self.mag_accuracy = cov_to_accuracy(msg.orientation_covariance[0])
        self.accel_accuracy = cov_to_accuracy(msg.linear_acceleration_covariance[0])
        self.gyro_accuracy = cov_to_accuracy(msg.angular_velocity_covariance[0])
        q = msg.orientation
        if self.last_orientation:
            dq = math.sqrt(sum((a - b) ** 2 for a, b in zip(
                [q.w, q.x, q.y, q.z], self.last_orientation)))
            if dq > 0.1:
                self.orientation_changes += 1
                self.motion_detected = True
        self.last_orientation = [q.w, q.x, q.y, q.z]

    def on_mag(self, msg: MagneticField) -> None:
        self.mag_sample_count += 1
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y
        mz = msg.magnetic_field.z
        self.mag_min_x = min(self.mag_min_x, mx)
        self.mag_max_x = max(self.mag_max_x, mx)
        self.mag_min_y = min(self.mag_min_y, my)
        self.mag_max_y = max(self.mag_max_y, my)
        self.mag_min_z = min(self.mag_min_z, mz)
        self.mag_max_z = max(self.mag_max_z, mz)

    def elapsed_s(self) -> float:
        return time.time() - self.start_time

    def remaining_s(self) -> float:
        return max(0.0, self.duration_s - self.elapsed_s())

    def is_timer_complete(self) -> bool:
        return self.remaining_s() <= 0

    def _mag_spans(self):
        if self.mag_min_x == float('inf'):
            return None
        return {
            'x': self.mag_max_x - self.mag_min_x,
            'y': self.mag_max_y - self.mag_min_y,
            'z': self.mag_max_z - self.mag_min_z,
        }

    def coverage_info(self) -> dict:
        """Coverage tier and average percent for mag ellipsoid sampling."""
        spans = self._mag_spans()
        if not spans:
            return {
                'percent': 0.0,
                'label': 'NO_DATA',
                'display': 'NO DATA (0%)',
            }
        coverage_x = min(100.0, (spans['x'] / TARGET_MAG_SPAN_UT) * 100.0)
        coverage_y = min(100.0, (spans['y'] / TARGET_MAG_SPAN_UT) * 100.0)
        coverage_z = min(100.0, (spans['z'] / TARGET_MAG_SPAN_UT) * 100.0)
        avg = (coverage_x + coverage_y + coverage_z) / 3.0
        if avg >= 80:
            label, display = 'EXCELLENT', f'EXCELLENT ({avg:.0f}%)'
        elif avg >= 60:
            label, display = 'GOOD', f'GOOD ({avg:.0f}%)'
        elif avg >= 40:
            label, display = 'FAIR', f'FAIR ({avg:.0f}%) — keep moving'
        else:
            label, display = 'POOR', f'POOR ({avg:.0f}%) — more movement needed'
        return {
            'percent': avg,
            'label': label,
            'display': display,
            'per_axis_percent': {'x': coverage_x, 'y': coverage_y, 'z': coverage_z},
        }

    def guidance(self) -> str:
        spans = self._mag_spans()
        if not spans:
            return 'Waiting for magnetometer data from /magnetic_field'
        if self.mag_accuracy >= 3:
            if self.accel_accuracy >= 2 and self.gyro_accuracy >= 2:
                return 'All sensors calibrated — safe to finish'
            return 'Magnetometer complete — continue briefly for accel/gyro'
        if spans['x'] < 30 or spans['y'] < 30 or spans['z'] < 30:
            return 'Insufficient coverage — rotate through more orientations (target ~40–60 µT per axis)'
        if self.motion_detected:
            return 'Motion detected — continue figure-8 through all axes'
        return 'Keep moving — figure-8 pattern through roll, pitch, and yaw'

    def consume_motion_flag(self) -> bool:
        """Return and clear one-shot motion hint (for terminal print_status)."""
        if self.motion_detected:
            self.motion_detected = False
            return True
        return False

    def snapshot(self) -> dict:
        cov = self.coverage_info()
        spans = self._mag_spans()
        mag_ranges = None
        if spans:
            mag_ranges = {
                'x': {'min': self.mag_min_x, 'max': self.mag_max_x, 'span': spans['x']},
                'y': {'min': self.mag_min_y, 'max': self.mag_max_y, 'span': spans['y']},
                'z': {'min': self.mag_min_z, 'max': self.mag_max_z, 'span': spans['z']},
            }
        return {
            'elapsed_s': round(self.elapsed_s(), 1),
            'remaining_s': round(self.remaining_s(), 1),
            'duration_s': self.duration_s,
            'accuracy': {
                'magnetometer': {
                    'level': self.mag_accuracy,
                    'name': ACCURACY_NAMES[self.mag_accuracy],
                },
                'accelerometer': {
                    'level': self.accel_accuracy,
                    'name': ACCURACY_NAMES[self.accel_accuracy],
                },
                'gyroscope': {
                    'level': self.gyro_accuracy,
                    'name': ACCURACY_NAMES[self.gyro_accuracy],
                },
            },
            'mag_ranges_uT': mag_ranges,
            'coverage': cov,
            'guidance': self.guidance(),
            'samples': {
                'imu': self.imu_sample_count,
                'magnetometer': self.mag_sample_count,
                'orientation_changes': self.orientation_changes,
            },
        }

    def success_level(self) -> str:
        if self.mag_accuracy >= 3 and self.accel_accuracy >= 2 and self.gyro_accuracy >= 2:
            return 'EXCELLENT'
        if self.mag_accuracy >= 2:
            return 'ACCEPTABLE'
        return 'INSUFFICIENT'

    def final_report(self, interrupted: bool = False) -> dict:
        snap = self.snapshot()
        return {
            'interrupted': interrupted,
            'duration_s': round(self.elapsed_s(), 1),
            'success_level': self.success_level(),
            'accuracy': snap['accuracy'],
            'mag_ranges_uT': snap['mag_ranges_uT'],
            'coverage': snap['coverage'],
            'samples': snap['samples'],
            'storage_note': (
                'Calibration is saved automatically to the BNO085 internal flash '
                'and persists across power cycles.'
            ),
        }


class ImuVerifySnapshot:
    """Non-ROS verify state; compass heading should come from /compass (bridge)."""

    def __init__(self, duration_s: int = 0, start_time: float = None):
        self.duration_s = int(duration_s)
        self.start_time = start_time if start_time is not None else time.time()
        self.sample_count = 0
        self.compass_heading = None
        self.compass_history = collections.deque(maxlen=50)
        self.has_imu = False
        self.accel_g = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gyro_dps = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.mag_uT = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.has_mag = False
        self.roll_deg = self.pitch_deg = 0.0
        self.quaternion = {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.quaternion_unit_ok = False
        self.gravity_ok = False
        self.accel_mag_mps2 = 0.0

    def on_imu(self, msg: Imu) -> None:
        self.sample_count += 1
        self.has_imu = True
        q = msg.orientation
        self.quaternion = {'w': q.w, 'x': q.x, 'y': q.y, 'z': q.z}
        self.quaternion_unit_ok = abs(math.sqrt(q.w ** 2 + q.x ** 2 + q.y ** 2 + q.z ** 2) - 1.0) < 0.01
        self.roll_deg, self.pitch_deg, _yaw = quaternion_to_euler_deg(q.w, q.x, q.y, q.z)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.accel_g = {'x': ax / 9.81, 'y': ay / 9.81, 'z': az / 9.81}
        self.accel_mag_mps2 = math.sqrt(ax * ax + ay * ay + az * az)
        self.gravity_ok = abs(self.accel_mag_mps2 - 9.81) < 2.0
        self.gyro_dps = {
            'x': math.degrees(msg.angular_velocity.x),
            'y': math.degrees(msg.angular_velocity.y),
            'z': math.degrees(msg.angular_velocity.z),
        }

    def on_mag(self, msg: MagneticField) -> None:
        self.has_mag = True
        self.mag_uT = {
            'x': msg.magnetic_field.x,
            'y': msg.magnetic_field.y,
            'z': msg.magnetic_field.z,
        }

    def on_compass(self, heading_deg: float) -> None:
        self.compass_heading = float(heading_deg) % 360.0
        self.compass_history.append(self.compass_heading)

    def elapsed_s(self) -> float:
        return time.time() - self.start_time

    def remaining_s(self) -> float:
        if self.duration_s <= 0:
            return None
        return max(0.0, self.duration_s - self.elapsed_s())

    def is_timer_complete(self) -> bool:
        if self.duration_s <= 0:
            return False
        return self.remaining_s() <= 0

    def heading_stability_deg(self) -> Optional[float]:
        n = len(self.compass_history)
        if n < 2:
            return None
        rads = [math.radians(v) for v in self.compass_history]
        sin_sum = sum(math.sin(r) for r in rads)
        cos_sum = sum(math.cos(r) for r in rads)
        r = math.hypot(sin_sum / n, cos_sum / n)
        if r < 1e-9:
            return None
        circ_var = max(0.0, 1.0 - r)
        return math.degrees(math.sqrt(circ_var))

    def snapshot(self) -> dict:
        return {
            'elapsed_s': round(self.elapsed_s(), 1),
            'remaining_s': (
                round(self.remaining_s(), 1) if self.remaining_s() is not None else None
            ),
            'duration_s': self.duration_s,
            'sample_count': self.sample_count,
            'compass_heading': self.compass_heading,
            'compass_samples': len(self.compass_history),
            'heading_stability_deg': self.heading_stability_deg(),
            'has_imu': self.has_imu,
            'has_mag': self.has_mag,
            'accel_g': self.accel_g,
            'gyro_dps': self.gyro_dps,
            'mag_uT': self.mag_uT,
            'roll_deg': round(self.roll_deg, 1),
            'pitch_deg': round(self.pitch_deg, 1),
            'quaternion': self.quaternion,
            'quaternion_unit_ok': self.quaternion_unit_ok,
            'gravity_ok': self.gravity_ok,
        }


def print_calibration_header():
    """Instructions printed for interactive CLI calibration."""
    print("\n" + "=" * 80)
    print("BNO085 INTERACTIVE CALIBRATION TOOL")
    print("=" * 80)
    print("\nCALIBRATION APPROACH:")
    print("The BNO085 uses unified sensor fusion - all sensors calibrate together.")
    print("\nINSTRUCTIONS:")
    print("1. MAGNETOMETER (Most Important):")
    print("   - Move sensor in figure-8 pattern through ALL orientations")
    print("   - Rotate through roll, pitch, and yaw axes")
    print("   - Watch the magnetometer ranges expand as you move")
    print("   - Target: Cover full 3D space for best calibration")
    print("\n2. ACCELEROMETER:")
    print("   - Auto-calibrates during motion")
    print("   - Benefits from varied orientations")
    print("\n3. GYROSCOPE:")
    print("   - Auto-calibrates during still and motion periods")
    print("\nVISUAL FEEDBACK:")
    print("   - Magnetometer ranges will show min/max values and progress bars")
    print("   - Coverage Quality indicator: POOR → FAIR → GOOD → EXCELLENT")
    print("   - Sensor Accuracy bars: UNRELIABLE → LOW → MEDIUM → HIGH")
    print("\nTARGET: All sensors reach HIGH accuracy (green bars)")
    print("TIP: Press Ctrl+C when satisfied with calibration quality")
    print("=" * 80 + "\n")


def print_calibration_report(report: dict) -> str:
    """Print final calibration report; returns success_level."""
    print("\n" + "=" * 80)
    if report.get('interrupted'):
        print("CALIBRATION INTERRUPTED BY USER")
    else:
        print("CALIBRATION COMPLETE")
    print("=" * 80)
    print(f"\nDuration: {report['duration_s']:.1f}s")
    acc = report['accuracy']
    print("\nFinal Sensor Accuracy:")
    print(f"  Magnetometer:  {acc['magnetometer']['name']}")
    print(f"  Accelerometer: {acc['accelerometer']['name']}")
    print(f"  Gyroscope:     {acc['gyroscope']['name']}")
    print("\nMagnetometer Coverage:")
    mag = report.get('mag_ranges_uT')
    if mag:
        for axis in ('x', 'y', 'z'):
            r = mag[axis]
            print(
                f"  {axis.upper()} Range: [{r['min']:+6.1f} .. {r['max']:+6.1f}] µT "
                f"(span: {r['span']:5.1f} µT)"
            )
        print(f"  Coverage Quality: {report['coverage']['display']}")
    else:
        print("  No magnetometer data collected")
    samples = report['samples']
    print(f"\nSample Statistics:")
    print(f"  IMU samples: {samples['imu']}")
    print(f"  Magnetometer samples: {samples['magnetometer']}")
    print(f"  Orientation changes: {samples['orientation_changes']}")
    print(f"\nCalibration Storage:")
    print(f"  {report['storage_note']}")
    level = report['success_level']
    if level == 'EXCELLENT':
        print("\nSUCCESS! Sensor is well-calibrated and ready for use")
    elif level == 'ACCEPTABLE':
        print("\nPARTIAL SUCCESS: Acceptable calibration achieved")
    else:
        print("\nINCOMPLETE: Magnetometer calibration insufficient")
    print("=" * 80)
    return level


# ============================================================================
# CALIBRATION MODE: Interactive sensor calibration
# ============================================================================

class BNO085Calibrator(Node):
    """Interactive calibration tool with real-time guidance and visual feedback."""

    def __init__(self, duration=30, save_interval=30, interactive=True, no_prompt=False):
        super().__init__('bno085_calibrator')
        self.tracker = ImuCalibrationTracker(duration)
        self.save_interval = save_interval
        self.interactive = interactive

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/magnetic_field', self.mag_callback, 10)

        if interactive:
            self.status_timer = self.create_timer(2.0, self.print_status)
            if save_interval > 0:
                self.save_timer = self.create_timer(float(save_interval), self.auto_save)
            print_calibration_header()
            if not no_prompt:
                try:
                    input("Press ENTER to start calibration (or Ctrl+C to cancel)...")
                    print("\nStarting calibration...\n")
                except KeyboardInterrupt:
                    print("\nCalibration cancelled by user")
                    rclpy.shutdown()
                    raise SystemExit(0)
            self.tracker.start_time = time.time()

        self.get_logger().info(f"Calibration started: {duration}s duration")
        self.get_logger().info("Press Ctrl+C at any time to finish calibration early")

    def imu_callback(self, msg: Imu):
        self.tracker.on_imu(msg)

    def mag_callback(self, msg: MagneticField):
        self.tracker.on_mag(msg)
        if self.tracker.mag_sample_count <= 5:
            mx = msg.magnetic_field.x
            my = msg.magnetic_field.y
            mz = msg.magnetic_field.z
            self.get_logger().info(
                f"Mag sample #{self.tracker.mag_sample_count}: "
                f"X={mx:.2f}, Y={my:.2f}, Z={mz:.2f} µT"
            )

    def _terminal_bar(self, acc):
        return ["░░░░", "▓░░░", "▓▓░░", "▓▓▓░", "▓▓▓▓"][max(0, min(3, int(acc)))]

    def _range_bar(self, span, width=50):
        filled = int(min(1.0, span / TARGET_MAG_BAR_UT) * width)
        bar = '█' * filled + '░' * (width - filled)
        return f"|{bar}|"

    def print_status(self):
        t = self.tracker
        elapsed = t.elapsed_s()
        remaining = t.remaining_s()
        snap = t.snapshot()

        print("\033[2J\033[H", end='')
        print("=" * 80)
        print(f"BNO085 CALIBRATION - {datetime.now().strftime('%H:%M:%S')}")
        print("=" * 80)
        print(f"\nTIME: {elapsed:.1f}s / {t.duration_s}s (Ctrl+C to finish early)")

        acc = snap['accuracy']
        print("\nSENSOR ACCURACY:")
        print(f"   Magnetometer:  {self._terminal_bar(acc['magnetometer']['level'])} "
              f"{acc['magnetometer']['name']}")
        print(f"   Accelerometer: {self._terminal_bar(acc['accelerometer']['level'])} "
              f"{acc['accelerometer']['name']}")
        print(f"   Gyroscope:     {self._terminal_bar(acc['gyroscope']['level'])} "
              f"{acc['gyroscope']['name']}")

        print("\nMAGNETOMETER RANGES (µT):")
        try:
            import shutil
            term_width = shutil.get_terminal_size().columns
        except Exception:
            term_width = 80
        bar_width = min(50, term_width - 35)

        mag = snap['mag_ranges_uT']
        if mag:
            for axis in ('x', 'y', 'z'):
                r = mag[axis]
                print(
                    f"   {axis.upper()}: [{r['min']:+6.1f} .. {r['max']:+6.1f}] "
                    f"{self._range_bar(r['span'], bar_width)} span={r['span']:5.1f}"
                )
            print(f"   Coverage Quality: {snap['coverage']['display']}")
            print(f"\n{snap['guidance']}")
            t.consume_motion_flag()
        else:
            print("   Waiting for magnetometer data...")

        samples = snap['samples']
        print(
            f"\nSAMPLES: IMU={samples['imu']}, Mag={samples['magnetometer']}, "
            f"Movements={samples['orientation_changes']}"
        )
        print("=" * 80 + "\n")

        if remaining <= 0:
            raise SystemExit("calibration_complete")

    def auto_save(self):
        print("Calibration auto-saved by sensor")

    def final_report(self, interrupted=False):
        report = self.tracker.final_report(interrupted=interrupted)
        if self.interactive:
            return print_calibration_report(report)
        return report['success_level']


# ============================================================================
# VERIFICATION MODE: Interactive ASCII display for sensor testing
# ============================================================================

def get_terminal_size():
    """Get terminal size, fallback to 80x24 if detection fails."""
    try:
        import shutil
        return shutil.get_terminal_size()
    except:
        return (80, 24)

def clear_screen_and_home():
    """Clear screen and move cursor to top-left."""
    try:
        sys.stdout.write('\x1b[2J')    # clear screen
        sys.stdout.write('\x1b[H')     # home cursor
        sys.stdout.flush()
    except:
        pass

class RotationVectorVerifier(Node):
    """Interactive ASCII display for BNO085 sensor testing."""

    def __init__(self, duration=10, interactive=True, no_prompt=False):
        """
        Args:
            duration (int): Seconds to run (0 = until interrupted).
            interactive: Terminal UI when True.
            no_prompt: Skip ENTER/auto-start prompt (web/automation).
        """
        super().__init__('rotation_vector_verifier')
        self.snapshot_tracker = ImuVerifySnapshot(duration)
        self.duration = duration
        self.interactive = interactive
        self._vis_initialized = False
        self.display_started = not interactive or no_prompt

        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.mag_sub = self.create_subscription(
            MagneticField, '/magnetic_field', self.mag_callback, 10)
        self.compass_sub = self.create_subscription(
            Vector3, '/compass', self.compass_callback, 10)

        if interactive:
            self._init_ascii_vis()
            print("\n" + "=" * 80)
            print("BNO085 INTERACTIVE SENSOR DISPLAY")
            print("=" * 80)
            print("\nReal-time sensor data visualization:")
            print("  • Accelerometer (g), gyroscope (deg/s), magnetometer (µT)")
            print("  • Compass heading from /compass (0°=N, 90°=E)")
            print("\nWaiting for sensor data...")
            print("=" * 80 + "\n")
            if not no_prompt:
                self._prompt_with_timeout()
            else:
                self.snapshot_tracker.start_time = time.time()
            self.display_timer = self.create_timer(0.1, self.update_display)
    
    def _prompt_with_timeout(self, timeout=10):
        """Prompt user with automatic timeout after specified seconds."""
        import select
        import sys
        
        print(f"Press ENTER to start display immediately (or wait {timeout}s for auto-start)...")
        
        # Check if stdin is available and is a terminal
        if not sys.stdin.isatty():
            # Not a terminal (e.g., running as service), skip prompt
            print(f"Auto-starting in {timeout}s...")
            time.sleep(timeout)
            print("\n🚀 Starting display...\n")
            self.display_started = True
            return
        
        try:
            # Use select to wait for input with timeout (Unix/Linux only)
            start_time = time.time()
            ready, _, _ = select.select([sys.stdin], [], [], timeout)
            
            if ready:
                # User pressed a key
                sys.stdin.readline()  # Consume the input
                print("\n🚀 Starting display...\n")
            else:
                # Timeout reached
                elapsed = time.time() - start_time
                print(f"\n⏱️  Auto-starting after {elapsed:.1f}s...\n")
            
            self.display_started = True
            
        except Exception as e:
            # Fallback if select doesn't work
            print(f"Auto-starting in {timeout}s...")
            time.sleep(timeout)
            print("\n🚀 Starting display...\n")
            self.display_started = True
    
    def _init_ascii_vis(self):
        """Initialize ASCII visualization."""
        if self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')    # reset attributes
            sys.stdout.write('\x1b[?25l')  # hide cursor
            clear_screen_and_home()
            self._vis_initialized = True
        except Exception as e:
            self._vis_initialized = False
    
    def _teardown_ascii_vis(self):
        """Clean up ASCII visualization."""
        if not self._vis_initialized:
            return
        try:
            sys.stdout.write('\x1b[0m')
            clear_screen_and_home()
            sys.stdout.write('\x1b[?25h')  # show cursor
            sys.stdout.flush()
        except Exception:
            pass
        self._vis_initialized = False
    
    def _signed_bar(self, value: float, limit: float, width: int = 50) -> str:
        """Create a signed bar visualization centered at zero."""
        width = max(10, width)
        mid = width // 2
        # clip value to limit
        val = max(-limit, min(limit, value))
        pos = int(round((val / limit) * mid))
        left = [' '] * mid
        right = [' '] * (width - mid - 1)
        if pos < 0:
            fill = mid + pos  # fill up to this index (exclusive)
            for i in range(fill, mid):
                left[i] = '-'
        elif pos > 0:
            for i in range(0, pos):
                if i < len(right):
                    right[i] = '+'
        return '[' + ''.join(left) + '|' + ''.join(right) + ']'
    
    def _render_compass(self, heading_deg: float, width: int = 40, height: int = 20) -> list:
        """Render compass heading as a point on a circle with N at top."""
        width = max(20, width)
        height = max(10, height)
        
        cx = width // 2
        cy = height // 2
        radius = min(cx - 3, cy - 2)
        
        # Create grid
        grid = [[' ' for _ in range(width)] for _ in range(height)]
        
        # Draw circle (octagon approximation)
        for angle in range(0, 360, 5):
            rad = math.radians(angle)
            x = cx + int(round(radius * math.sin(rad)))
            y = cy - int(round(radius * math.cos(rad)))
            if 0 <= y < height and 0 <= x < width:
                grid[y][x] = '.'
        
        # Mark cardinal directions
        if cy - radius >= 0:
            grid[cy - radius][cx] = 'N'
        if cx + radius < width:
            grid[cy][cx + radius] = 'E'
        if cy + radius < height:
            grid[cy + radius][cx] = 'S'
        if cx - radius >= 0:
            grid[cy][cx - radius] = 'W'
        
        # Draw center
        grid[cy][cx] = '+'
        
        # Draw heading indicator
        theta = math.radians(heading_deg)
        hx = cx + int(round(radius * math.sin(theta)))
        hy = cy - int(round(radius * math.cos(theta)))
        if 0 <= hy < height and 0 <= hx < width:
            grid[hy][hx] = 'O'
        
        # Draw line from center to heading point
        steps = int(radius)
        for i in range(1, steps):
            frac = i / float(steps)
            lx = cx + int(round(frac * radius * math.sin(theta)))
            ly = cy - int(round(frac * radius * math.cos(theta)))
            if 0 <= ly < height and 0 <= lx < width:
                if grid[ly][lx] == ' ':
                    grid[ly][lx] = '*'
        
        return [''.join(row) for row in grid]
    
    def imu_callback(self, msg: Imu):
        self.snapshot_tracker.on_imu(msg)

    def compass_callback(self, msg: Vector3):
        if msg.z is not None:
            self.snapshot_tracker.on_compass(msg.z)

    def mag_callback(self, msg: MagneticField):
        self.snapshot_tracker.on_mag(msg)
    
    def update_display(self):
        if not self.display_started or not self.snapshot_tracker.has_imu:
            return
        try:
            snap = self.snapshot_tracker.snapshot()
            clear_screen_and_home()
            term_width, term_height = get_terminal_size()
            bar_width = term_width - 20
            a_lim, g_lim, m_lim = 2.0, 500.0, 100.0
            ag = snap['accel_g']
            gd = snap['gyro_dps']
            mg = snap['mag_uT']
            heading = snap['compass_heading']
            if heading is None:
                heading = 0.0
            q = snap['quaternion']
            sensor_lines = [
                "=== BNO085 Sensor Data (Interactive Display) ===",
                "",
                "Accelerometer:",
                f"Ax {ag['x']:+7.3f} g   " + self._signed_bar(ag['x'], a_lim, bar_width),
                f"Ay {ag['y']:+7.3f} g   " + self._signed_bar(ag['y'], a_lim, bar_width),
                f"Az {ag['z']:+7.3f} g   " + self._signed_bar(ag['z'], a_lim, bar_width),
                "",
                "Gyroscope:",
                f"Gx {gd['x']:+7.1f} dps " + self._signed_bar(gd['x'], g_lim, bar_width),
                f"Gy {gd['y']:+7.1f} dps " + self._signed_bar(gd['y'], g_lim, bar_width),
                f"Gz {gd['z']:+7.1f} dps " + self._signed_bar(gd['z'], g_lim, bar_width),
                "",
                "Magnetometer:",
                f"Mx {mg['x']:+7.1f} uT  " + self._signed_bar(mg['x'], m_lim, bar_width),
                f"My {mg['y']:+7.1f} uT  " + self._signed_bar(mg['y'], m_lim, bar_width),
                f"Mz {mg['z']:+7.1f} uT  " + self._signed_bar(mg['z'], m_lim, bar_width),
                "",
                f"=== Compass (/compass): {heading:6.1f} deg (0=N, 90=E) ===",
                "",
                "=== Quaternion ===",
                f"Qw {q['w']:+7.3f}  Qx {q['x']:+7.3f}  Qy {q['y']:+7.3f}  Qz {q['z']:+7.3f}",
                f"Unit quat: {'PASS' if snap['quaternion_unit_ok'] else 'FAIL'}",
                "",
                f"Roll {snap['roll_deg']:+7.1f}  Pitch {snap['pitch_deg']:+7.1f}",
                f"Gravity: {'PASS' if snap['gravity_ok'] else 'FAIL'}",
                "",
            ]
            available_height = term_height - len(sensor_lines) - 3
            compass_lines = self._render_compass(
                heading, width=term_width, height=max(5, available_height))
            elapsed = snap['elapsed_s']
            rem = snap['remaining_s']
            rem_s = f"{rem:.1f}" if rem is not None else "inf"
            status_line = (
                f"Samples: {snap['sample_count']} | Elapsed: {elapsed:.1f}s | "
                f"Remaining: {rem_s}"
            )
            for ln in sensor_lines + compass_lines + ["", status_line, "Ctrl-C to exit"]:
                sys.stdout.write(ln + '\n')
            sys.stdout.flush()
            if self.snapshot_tracker.is_timer_complete():
                self._teardown_ascii_vis()
                print(f"\nVerification complete after {elapsed:.1f}s")
                raise SystemExit("verification_complete")
        except SystemExit:
            raise
        except Exception:
            pass


# ============================================================================
# STATUS MODE: Check system health
# ============================================================================

def cmd_status():
    """Check BNO085 system status."""
    ros_env = 'source /opt/ros/humble/setup.bash'
    ros_cmd = ['bash', '-c']

    def _run_ros(command: str, timeout: float):
        return subprocess.run(
            ros_cmd + [f'{ros_env} && {command}'],
            capture_output=True,
            text=True,
            timeout=timeout,
        )

    print("\n📊 BNO085 SYSTEM STATUS")
    print("=" * 60)

    nodes = ''
    try:
        result = _run_ros('ros2 node list', 10)
        nodes = result.stdout
        driver_ok = 'bno08x_ros' in nodes or 'bno08x_driver' in nodes
        bridge_ok = 'bno085_bridge' in nodes
        print(f"\n{'✅' if driver_ok else '❌'} BNO08x C++ driver (bno08x_ros)")
        print(f"{'✅' if bridge_ok else '❌'} BNO085 bridge")
    except subprocess.TimeoutExpired:
        print("\n❌ ROS2 node list timed out")
    except Exception:
        print("\n❌ Cannot check ROS2 nodes")

    topic_names = set()
    try:
        result = _run_ros('ros2 topic list', 10)
        topic_names = set(result.stdout.splitlines())
    except subprocess.TimeoutExpired:
        pass
    except Exception:
        pass

    print("\nTopics:")
    for topic in ['/imu', '/magnetic_field', '/compass', '/pose', '/accel', '/gyro', '/imu_health']:
        print(f"  {'✅' if topic in topic_names else '❌'} {topic}")

    # echo --once can take several seconds on a loaded graph; avoid ros2 topic info (often >2s here)
    try:
        result = _run_ros('ros2 topic echo /imu_health --once', 10)
        if result.returncode == 0 and 'data: true' in result.stdout.lower():
            print("\n💚 Health Status: HEALTHY")
        elif result.returncode == 0 and 'data: false' in result.stdout.lower():
            print("\n❤️  Health Status: UNHEALTHY")
        else:
            print("\n⚠️  Health Status: UNKNOWN")
    except subprocess.TimeoutExpired:
        print("\n⚠️  Health Status: timed out waiting for /imu_health")
    except Exception:
        print("\n⚠️  Health Status: Cannot check")

    try:
        result = _run_ros('ros2 topic echo /compass --once', 10)
        match = re.search(r'z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)', result.stdout)
        if result.returncode == 0 and match:
            print(f"🧭 Compass sample: {float(match.group(1)):.1f}° (0=N, 90=E, from /compass)")
    except (subprocess.TimeoutExpired, ValueError):
        pass
    except Exception:
        pass
    
    # Check I2C
    print("\nHardware:")
    try:
        result = subprocess.run(['i2cdetect', '-y', '0'], capture_output=True, text=True, timeout=2)
        if '4a' in result.stdout or 'UU' in result.stdout:
            print("  ✅ BNO085 on I2C bus 0 at 0x4a")
        else:
            print("  ❌ BNO085 not detected")
    except:
        print("  ⚠️  Cannot check I2C")
    
    print("\n" + "=" * 60 + "\n")


# ============================================================================
# MAIN CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='BNO085 IMU Management Tool',
        epilog=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('command', nargs='?', default='bridge',
                        choices=['bridge', 'calibrate', 'verify', 'status'],
                        help='Command to execute (default: bridge)')
    parser.add_argument('--duration', type=int, default=60,
                        help='Duration in seconds for calibrate/verify (default: 60)')
    parser.add_argument('--save-interval', type=int, default=30,
                        help='Calibration auto-save interval (default: 30)')
    parser.add_argument('--no-prompt', action='store_true',
                        help='Skip ENTER prompt (automation / web subprocess)')
    
    # Enable bash completion for command-line arguments
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    
    if args.command == 'status':
        cmd_status()
    elif args.command in ['bridge', 'calibrate', 'verify']:
        rclpy.init()
        node = None
        try:
            if args.command == 'bridge':
                node = BNO085Bridge()
            elif args.command == 'calibrate':
                node = BNO085Calibrator(
                    args.duration, args.save_interval, no_prompt=args.no_prompt)
            else:  # verify
                node = RotationVectorVerifier(args.duration, no_prompt=args.no_prompt)
            
            rclpy.spin(node)
        except (KeyboardInterrupt, SystemExit) as e:
            # Handle calibration completion (both interrupted and natural)
            if args.command == 'calibrate' and node:
                # Determine if interrupted or completed
                interrupted = isinstance(e, KeyboardInterrupt)
                success_level = node.final_report(interrupted=interrupted)
                # Clean up current node
                node.destroy_node()
                node = None
                
                # Offer to run verify
                print("\nWould you like to verify the calibration results?")
                try:
                    response = input("Run verification display? (Y/n): ").strip().lower()
                    if response in ['', 'y', 'yes']:
                        print("\n🔄 Starting verification display...Ctrl-C to exit\n")
                        time.sleep(3)
                        
                        
                        # Start verify node
                        verify_node = RotationVectorVerifier(duration=0, no_prompt=True)
                        try:
                            rclpy.spin(verify_node)
                        except KeyboardInterrupt:
                            print("\n✅ Verification stopped")
                        finally:
                            if hasattr(verify_node, '_teardown_ascii_vis'):
                                verify_node._teardown_ascii_vis()
                            verify_node.destroy_node()
                except (KeyboardInterrupt, EOFError):
                    print("\n✅ Calibration complete")
            elif args.command == 'verify' and node:
                if isinstance(e, KeyboardInterrupt):
                    print("\n✅ Stopped")
                node.destroy_node()
                node = None
            else:
                print("\n✅ Stopped")
        finally:
            # Clean up ASCII display if it was initialized
            if node and hasattr(node, '_teardown_ascii_vis'):
                node._teardown_ascii_vis()
            if rclpy.ok():
                rclpy.shutdown()
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
