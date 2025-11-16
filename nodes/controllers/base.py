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
        self.name = self.__class__.__name__
        self.logger = logger
        self.parent_node = parent_node
        self._captains_log_pub = None
        self._controller_state_pub = None

    def generate_control(self, state: BoatState) -> ControlCommand:
        raise NotImplementedError

    def reset(self):
        """Reset controller state (called when switching to this controller)."""
        pass

    def update_config(self, config: Dict[str, Any]):
        """Update controller configuration."""
        self.config.update(config)

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


