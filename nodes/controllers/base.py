#!/usr/bin/env python3

# 1) Standard library imports
import os
import sys
import time
import math
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List, Tuple

# 2) Third-party / ROS imports
from geometry_msgs.msg import Vector3

# 3) Path modifications (for local support utilities)
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'support'))

# 4) Local/custom imports deferred in methods where needed to avoid heavy ROS side-effects on import


def signed_angle_difference_degrees(angle1_deg: float, angle2_deg: float) -> float:
    """
    Computes the signed difference between two angles in degrees,
    returning a result in the range [-180, 180].
    """
    diff_deg = angle1_deg - angle2_deg
    return (diff_deg + 180.0) % 360.0 - 180.0


# Max normalized sail command from wind trim at close-hauled (-) and broad reach (+).
SAIL_WIND_EXTREME_LIMIT = 0.7


def relative_wind_angle_to_sail_cmd(wind_angle_deg: float) -> float:
    """
    Map relative wind angle to normalized sail command (-1 in, +1 out).

    wind_angle_deg: degrees CW from bow, where wind comes FROM (anem convention).
    Folds port-side angles (180-360) so beam reach on either tack maps to neutral sail.
    Close-hauled and broad reach are capped at ±SAIL_WIND_EXTREME_LIMIT (default 0.7).
    """
    angle_from_bow = float(wind_angle_deg) % 360.0
    if angle_from_bow > 180.0:
        angle_from_bow = 360.0 - angle_from_bow
    sail_cmd = (angle_from_bow - 90.0) / 90.0 * SAIL_WIND_EXTREME_LIMIT
    return max(-SAIL_WIND_EXTREME_LIMIT, min(SAIL_WIND_EXTREME_LIMIT, sail_cmd))


@dataclass
class BoatState:
    """Complete state representation of the boat from all sensors."""
    # Time
    timestamp: float = 0.0

    # Navigation
    compass_heading: Optional[float] = None  # degrees (0-360) - fast updates (~10 Hz), low noise, reliable when moving
    gps_cog: Optional[float] = None          # course over ground, degrees true (updates at ~0.2 Hz)
    gps_sog: Optional[float] = None          # speed over ground, knots (updates at ~0.2 Hz - CRITICAL: may be stale)
    gps_velocity: Optional[Vector3] = None   # x=north, y=east, z=speed (updates at ~0.2 Hz)
    # Note: GPS position (/fix) updates at 1 Hz, but GPS velocity/COG/SOG updates at ~0.2 Hz (1/5 Hz)
    # Controller provides fallback speed estimation from position changes when GPS SOG is stale
    # 
    # Future sensor fusion:
    # - fused_heading: Compass-GPS fusion (compass fast/low-noise, GPS absolute reference)
    # - absolute_wind_direction: Calculated from relative wind + heading (centralized in controller.py)

    # IMU
    accel: Optional[Vector3] = None          # accelerometer, g units
    gyro: Optional[Vector3] = None           # gyroscope, deg/s
    compass_raw: Optional[Vector3] = None    # magnetometer, µT

    # Wind
    wind_speed: Optional[float] = None       # m/s
    wind_angle: Optional[float] = None       # See controller for absolute/relative semantics
    wind_temp: Optional[float] = None        # celsius

    # Radio/Human input (from rudder_sail_radio.py)
    radio_rudder: Optional[float] = None     # -1:+1 left:right
    radio_sail: Optional[float] = None       # -1:+1 in:out
    human_controlled: bool = True            # control mode

    # Controller state
    target_heading: Optional[float] = None   # degrees

    # Battery/Water monitoring
    battery_voltage: Optional[float] = None
    battery_remaining_pct: Optional[float] = None
    battery_low_alert: bool = False
    saltwater_alert: bool = False
    humidity_alert: bool = False

    # Connectivity monitoring (for RTH)
    shore_connected: bool = False
    last_shore_contact: Optional[float] = None
    remote_command: Optional[str] = None

    # Home position tracking (for return-to-home)
    home_latitude: Optional[float] = None
    home_longitude: Optional[float] = None
    current_latitude: Optional[float] = None
    current_longitude: Optional[float] = None
    return_to_home_active: bool = False

    # Geofence tracking (for patrol controller)
    geofence_polygon: Optional[List[Tuple[float, float]]] = None
    distance_to_boundary: Optional[float] = None
    predicted_boundary_crossing_time: Optional[float] = None

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

        lat1 = math.radians(self.current_latitude)
        lon1 = math.radians(self.current_longitude)
        lat2 = math.radians(self.home_latitude)
        lon2 = math.radians(self.home_longitude)

        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0
        return bearing_deg

    def get_distance_to_home(self) -> Optional[float]:
        """Calculate distance to home position in nautical miles."""
        if (self.home_latitude is None or self.home_longitude is None or
            self.current_latitude is None or self.current_longitude is None):
            return None

        lat1 = math.radians(self.current_latitude)
        lon1 = math.radians(self.current_longitude)
        lat2 = math.radians(self.home_latitude)
        lon2 = math.radians(self.home_longitude)

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        distance_nm = 3440.065 * c
        return distance_nm

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for data logging."""
        data = asdict(self)
        for key, value in data.items():
            if hasattr(value, 'x'):
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


class BaseController:
    """Abstract-ish base class for all controllers."""

    def __init__(self, config: Dict[str, Any], logger=None, parent_node=None):
        self.config = config
        # Subclasses should set self.name in their own __init__ methods
        self.name = None
        self.logger = logger
        self.parent_node = parent_node
        self._captains_log_pub = None
        self._controller_state_pub = None
        self._release_servos_pub = None
        self._last_release_servos_state = None
        self._last_release_servos_publish_time = 0.0
        self.RELEASE_SERVOS_PUBLISH_INTERVAL = 1.0  # Publish at 1Hz maximum

    def generate_control(self, state: BoatState) -> ControlCommand:
        raise NotImplementedError

    def reset(self):
        """Reset controller state (called when switching to this controller).
        
        By default, publishes release_servos=False to ensure normal PWM operation
        when switching to non-human controllers. HumanController overrides this.
        """
        # Publish False immediately (force=True) to ensure normal PWM operation
        # This bypasses throttling since controller switch is a critical state change
        self.publish_release_servos(False, force=True)

    def update_config(self, config: Dict[str, Any]):
        """Update controller configuration."""
        self.config.update(config)
    def clip_rudder(self, cmd_rudder: float) -> float:
        """Clip rudder to ±1; rudder_sail_radio maps ±1 to servo_rudder_*_pw in argo.yaml."""
        return max(-1.0, min(1.0, float(cmd_rudder)))

    def _get_captains_log_publisher(self):
        """Get or create the captain's log publisher."""
        if self.parent_node and self._captains_log_pub is None:
            from std_msgs.msg import String
            self._captains_log_pub = self.parent_node.create_publisher(
                String, '/controller/captains_log', 10)
        return self._captains_log_pub

    def _get_controller_state_publisher(self):
        """Get or create the controller state publisher."""
        if self.parent_node and self._controller_state_pub is None:
            from std_msgs.msg import String
            self._controller_state_pub = self.parent_node.create_publisher(
                String, '/controller/state', 10)
        return self._controller_state_pub

    def _get_release_servos_publisher(self):
        """Get or create the release_servos publisher."""
        if self.parent_node and self._release_servos_pub is None:
            from std_msgs.msg import Bool
            self._release_servos_pub = self.parent_node.create_publisher(
                Bool, '/release_servos', 10)
        return self._release_servos_pub

    def publish_release_servos(self, release: bool, force: bool = False):
        """Publish release_servos topic to signal servo release state.
        
        Throttled to 1Hz maximum to reduce overhead. Publishes immediately if:
        - force=True (e.g., on controller switch)
        - State changed from last published value
        - At least 1 second has passed since last publication
        
        Args:
            release: True to release servos (high impedance), False for normal PWM
            force: If True, publish immediately regardless of throttling
        """
        current_time = time.time()
        state_changed = (self._last_release_servos_state is None or 
                        self._last_release_servos_state != release)
        time_elapsed = (current_time - self._last_release_servos_publish_time) >= self.RELEASE_SERVOS_PUBLISH_INTERVAL
        
        # Publish if forced, state changed, or enough time has passed
        if force or state_changed or time_elapsed:
            pub = self._get_release_servos_publisher()
            if pub and self.parent_node:
                from std_msgs.msg import Bool
                msg = Bool(data=release)
                try:
                    # Try direct publish first (more reliable)
                    pub.publish(msg)
                    self._last_release_servos_state = release
                    self._last_release_servos_publish_time = current_time
                except Exception as e:
                    # Fallback to safe_publish if direct publish fails
                    from safe_publish import safe_publish
                    success = safe_publish(pub, msg, self.parent_node)
                    if not success and self.logger:
                        self.logger.warn(f"Failed to publish release_servos={release}: {e}")
                    else:
                        self._last_release_servos_state = release
                        self._last_release_servos_publish_time = current_time
            elif self.logger:
                self.logger.warn(f"Cannot publish release_servos: pub={pub is not None}, parent_node={self.parent_node is not None}")

    def log_entry(self, message: str, level: str = "INFO"):
        """
        Publish a captain's log entry. Skips logging when paused unless ERROR.
        """
        if level != "ERROR":
            if self.parent_node and hasattr(self.parent_node, 'is_paused') and self.parent_node.is_paused():
                return

        pub = self._get_captains_log_publisher()
        if pub and self.parent_node:
            from std_msgs.msg import String
            from safe_publish import safe_publish
            log_msg = String(data=f"[{level}] {message}")
            safe_publish(pub, log_msg, self.parent_node)

        if self.logger:
            if self.parent_node and hasattr(self.parent_node, 'is_paused') and self.parent_node.is_paused() and level != "ERROR":
                return
            if level == "WARN":
                self.logger.warn(message)
            elif level == "ERROR":
                self.logger.error(message)
            else:
                self.logger.info(message)

    def publish_state(self, state_name: str, state_value: str = ""):
        """Publish controller state for visualization."""
        pub = self._get_controller_state_publisher()
        if pub and self.parent_node:
            from std_msgs.msg import String
            from safe_publish import safe_publish
            state_msg = String(data=f"{state_name}:{state_value}" if state_value else state_name)
            safe_publish(pub, state_msg, self.parent_node)

    def log_periodic_state(self, state: BoatState, interval_sec: float = 30.0):
        """Base implementation for periodic state logging."""
        if not hasattr(self, '_last_periodic_log_time'):
            self._last_periodic_log_time = 0.0
        current_time = time.time()
        if current_time - self._last_periodic_log_time < interval_sec:
            return
        state_parts = []
        # Always include controller name to avoid confusion
        state_parts.append(f"Controller: {self.name}")
        if state.compass_heading is not None:
            state_parts.append(f"heading={state.compass_heading:.1f}°")
        if state.gps_sog is not None:
            state_parts.append(f"speed={state.gps_sog:.1f}kt")
        if state.wind_speed is not None and state.wind_angle is not None:
            state_parts.append(f"wind={state.wind_speed:.1f}m/s @ {state.wind_angle:.1f}°")
        if state.current_latitude is not None and state.current_longitude is not None:
            state_parts.append(f"pos=({state.current_latitude:.6f}, {state.current_longitude:.6f})")
        if state_parts:
            self.log_entry(f"State: {', '.join(state_parts)}", level="INFO")
        self._last_periodic_log_time = current_time
