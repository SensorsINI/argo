#!/usr/bin/env python3
"""
Argo Boat 3D Visualization Node
===============================

Purpose
-------
Publishes standard visualization markers for rendering the Argo sailboat state
in RViz and the Foxglove 3D panel. Intended for operator situational awareness
during development and testing. Does not affect control logic.

What is visualized
------------------
- Boat hull and mast as basic geometry
- Rudder position indicator
- Sail position indicator
- Wind vector arrow
- GPS velocity vector
- Boat heading arrow
- Roll/pitch indicators derived from accelerometer data

Published Topics
----------------
- /visualization_marker (visualization_msgs/Marker): Individual markers
- /visualization_marker_array (visualization_msgs/MarkerArray): All markers together

Marker Persistence and Inspection
----------------------------------
Markers have lifetime=0 (persistent), but they disappear when this node terminates.
To inspect markers after simulation ends:
1. Use Ctrl+Z (SIGSTOP) to pause simulation before stopping
2. Record with ros2 bag and playback for post-analysis
3. Use Foxglove's recording feature to capture and replay sessions

The heading trail uses incremental publishing (only new markers) for bandwidth
efficiency, reducing network usage by ~500x compared to republishing all markers.

Subscribed Topics
-----------------
- /pose (geometry_msgs/Vector3): Boat heading (z-component)
- /accel (geometry_msgs/Vector3): IMU accelerometer for roll/pitch
- /rudder_sail_servo (geometry_msgs/Vector3): Final executed rudder and sail commands (preferred)
- /rudder_sail_radio (geometry_msgs/Vector3): Raw keyboard/radio input (fallback for simulation)
- /rudder_sail_cmd (geometry_msgs/Vector3): Autonomous rudder and sail commands (fallback)
- /anem_speed_angle_temp (geometry_msgs/Vector3): Wind data
- /gps_velocity (geometry_msgs/Vector3): GPS velocity vector
- /fix (sensor_msgs/NavSatFix): GPS position

Coordinate Frames, Marker Implementation, and Axis Conventions
--------------------------------------------------------------
All visualization markers created by this node are published using the 'base_link' frame, which is the boat's local coordinate frame. The base_link frame is transformed to the world ENU (East-North-Up) coordinate system via transforms published by the argo_transform_publisher node. This is especially important for integration with visualization tools such as Foxglove Studio and RViz.

**Coordinate Frame Hierarchy:**
The transform tree establishes the relationship between frames:
- **map**: Fixed world frame at GPS origin (ENU convention)
  └── **odom**: Odometry frame (currently same as map)
      └── **base_link**: Boat center frame (this node publishes markers here)
          ├── gps_link (GPS antenna position)
          ├── compass_link (IMU/magnetometer position)
          ├── wind_sensor_link (anemometer position)
          └── rudder_link (rudder position)

**ENU Frame (map/odom) in ROS and Foxglove:**
- **X axis (East):** Positive X points to the **east**.
- **Y axis (North):** Positive Y points to the **north**.
- **Z axis (Up):** Positive Z points **upward**.

**base_link Frame (Boat-Centric Coordinate System):**
The base_link frame is a local coordinate frame attached to the boat's center. All boat visualization markers are defined in this frame, which automatically moves and rotates with the boat as transforms are updated.

**base_link axes (ROS standard convention):**
- **X axis (Forward/Aft):** Positive X points **forward** (toward bow), negative X points **aft** (toward stern).
- **Y axis (Port/Starboard):** Positive Y points to **port** (left side), negative Y points to **starboard** (right side).
- **Z axis (Up/Down):** Positive Z points **upward**, negative Z points **downward**.

**Relationship between base_link and ENU:**
- The base_link frame rotates relative to ENU based on the boat's heading (yaw).
- When the boat points north (heading = 0°), base_link +X aligns with ENU +Y (north).
- When the boat points east (heading = 90°), base_link +X aligns with ENU +X (east).
- base_link +Y always points to port (left side of boat) regardless of heading.
- base_link +Z always points up (same as ENU +Z).

**Visualization Markers in base_link:**
All boat markers are defined in base_link coordinates, which means they move and rotate with the boat automatically:
- The **hull marker** (triangle) has:
  - Tip at bow: (hull_half_length, 0, 0) - forward (+X direction)
  - Base at stern: (-hull_half_length, ±hull_half_width, 0) - aft (-X direction)
- The **mast marker** (cylinder) extends from (0, 0, 0) to (0, 0, mast_height).
- The **rudder marker** (triangle) is located at stern: x=-0.325 (stern), y=0 (centerline), z below hull.
- The **rudder arrow** is positioned at stern center: (-0.325, 0, 0).
- The **sail marker** (triangle) extends from mast top (0, 0, mast_top) to stern area, rotated based on sail position.
- The **wind vector marker** is positioned above the mast (0, 0, above_mast), with arrow direction based on wind angle relative to boat.
- The **velocity vector marker** starts at boat center (0, 0, 0), points in direction of movement.
- The **heading arrow** points in the boat's forward direction (+X axis in base_link).

**Practical Notes:**
- In Foxglove, when viewing in "base_link" frame, all boat markers appear stationary (they're attached to the boat).
- When viewing in "map" or "ENU frame", the entire boat (with all markers) moves and rotates as transforms update.
- The boat's position and orientation in the world (map frame) come from GPS and compass data via the transform publisher.
- All marker positions are relative to the boat center (base_link origin), making visualization independent of boat location.

**Coordinate Conversion Example:**
If the boat is at GPS position (latitude, longitude) and heading 45° (northeast):
- A point at base_link (0, 0.5, 0) is 0.5m forward of boat center
- In ENU/map frame, this point is rotated 45° and translated from boat's GPS position
- The transform publisher handles this conversion automatically via /tf transforms

"""

import rclpy
from geometry_msgs.msg import Vector3, PoseStamped, Point
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, ColorRGBA, Header, Float64, String
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import numpy as np
import sys
import os
import argparse
import argcomplete
import copy
from collections import deque
from dataclasses import dataclass

# Import ArgoBaseNode
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode

# Default update rate (Hz) - can be overridden by parameter
DEFAULT_UPDATE_RATE = 10.0

@dataclass
class HeadingTrailEntry:
    """Snapshot of boat position and heading for trail visualization."""
    x: float
    y: float
    yaw_rad: float
    compass_deg: float
    human_controlled: bool = False  # Track control mode for this trail point
    controller_state: str = ""  # Track controller state (e.g., 'tacking', 'jibing', 'broad_reach')


class ArgoBoatVisualization(ArgoBaseNode):
    # Common text size for all marker text labels (in meters, before visualization scale)
    TEXT_MARKER_SIZE = 0.05
    
    def __init__(self, debug_mode=False):
        super().__init__('argo_boat_visualization')
        
        # Declare parameters
        self.declare_parameter('visualization_scale', 3.0)
        self.visualization_scale = self.get_parameter('visualization_scale').get_parameter_value().double_value
        
        # Read update rate from shared simulation parameters (argo.yaml)
        # Uses simulation.publish_rate to match transform publisher and simulator rate
        self.declare_parameter('simulation.publish_rate', DEFAULT_UPDATE_RATE)
        self.update_rate = self.get_parameter('simulation.publish_rate').get_parameter_value().double_value
        if self.update_rate <= 0:
            self.update_rate = DEFAULT_UPDATE_RATE
        
        # Historical heading trail configuration
        self.declare_parameter('simulation.heading_trail_limit', 20)
        initial_trail_limit = max(
            0,
            int(self.get_parameter('simulation.heading_trail_limit').get_parameter_value().integer_value)
        )
        self.heading_trail_limit = initial_trail_limit
        
        # No-go zone angle for sail positioning (read from controller parameters)
        self.declare_parameter('controller_node.no_go_zone_angle', 45.0)
        self.no_go_zone_angle = self.get_parameter('controller_node.no_go_zone_angle').get_parameter_value().double_value
        # Use unlimited deque - markers will expire via lifetime instead of deque limit
        self.heading_trail = deque()
        self.heading_trail_spacing_m = 3.0  # Minimum spacing between trail markers
        self.heading_trail_heading_threshold_deg = 12.0  # Heading delta to force a new marker
        self.heading_trail_time_spacing_s = 5.0  # Minimum time between markers
        self.heading_trail_last_time_s = None
        # Unique marker ID counter (increments for each new marker, ensuring unique IDs)
        self._trail_marker_id_counter = 0
        
        # Log scale setting for debugging
        if self.visualization_scale != 1.0:
            self.get_logger().info(f"Visualization scale set to {self.visualization_scale}x")
        
        # Log update rate
        self.get_logger().info(f"Visualization update rate: {self.update_rate:.1f} Hz")
        
        # Clock time for timestamp preservation during re-recording
        # Subscribe to /clock topic to get simulated time from bag playback
        # Use BEST_EFFORT reliability to match ros2 bag play --clock QoS
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.sim_time = None
        self.create_subscription(
            Clock, '/clock', 
            self.clock_callback, 
            clock_qos
        )
        
        # Add parameter callback to handle runtime parameter changes (e.g., from Foxglove)
        self.add_on_set_parameters_callback(self._on_parameter_change)
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        # Debug publisher for calculated wind direction (for debugging wind vector drift)
        self.pub_debug_wind = self.create_publisher(Float64, '/visualization/debug/wind_direction_calculated', 10)
        
        # Subscribers
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Vector3, '/accel', self.accel_callback, 10)
        # Subscribe to control topics (priority order: servo > radio > cmd)
        # /rudder_sail_servo: Final executed commands (best, if rudder_sail_radio.py is running)
        self.create_subscription(Vector3, '/rudder_sail_servo', self.control_callback, 10)
        # /rudder_sail_radio: Raw keyboard/radio input (good fallback for simulation)
        self.create_subscription(Vector3, '/rudder_sail_radio', self.control_callback, 10)
        # /rudder_sail_cmd: Autonomous commands (lowest priority fallback)
        self.create_subscription(Vector3, '/rudder_sail_cmd', self.control_callback, 10)
        self.create_subscription(Vector3, '/anem_speed_angle_temp', self.wind_callback, 10)
        # Subscribe to true wind direction for accurate relative wind calculation
        self.create_subscription(Float64, '/simulator/true_wind_direction', self.true_wind_callback, 10)
        self.create_subscription(Bool, '/simulator/tacking', self.tacking_status_callback, 10)
        self.create_subscription(Vector3, '/gps_velocity', self.velocity_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        # Subscribe to true GPS position for noise visualization
        self.create_subscription(NavSatFix, '/fix_true', self.gps_true_callback, 10)
        
        # Subscribe to human control status and controller state
        self.create_subscription(Bool, '/human_controlled', self.human_controlled_callback, 10)
        self.create_subscription(String, '/controller/state', self.controller_state_callback, 10)
        
        # Subscribe to sailing area markers (QoS must match sailing_area_publisher)
        from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
        sailing_marker_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        self.sailing_boundaries = []
        self.sailing_waypoints = []
        self.sailing_hazards = []
        self.create_subscription(MarkerArray, '/sailing_boundaries', self.sailing_boundaries_callback, sailing_marker_qos)
        self.create_subscription(MarkerArray, '/sailing_waypoints', self.sailing_waypoints_callback, sailing_marker_qos)
        self.create_subscription(MarkerArray, '/sailing_hazards', self.sailing_hazards_callback, sailing_marker_qos)
        
        # State variables
        self.boat_heading = 0.0  # degrees
        self.boat_roll = 0.0     # degrees
        self.boat_pitch = 0.0    # degrees
        self.rudder_cmd = 0.0    # normalized -1 to +1
        self.sail_cmd = 0.0      # normalized -1 to +1
        self.wind_speed = 0.0    # m/s
        self.wind_angle = 0.0    # degrees relative to boat
        self.wind_temp = 0.0     # celsius
        self.true_wind_direction = None  # degrees (compass convention) - for recalculating relative wind
        self.gps_velocity_north = 0.0  # knots
        self.gps_velocity_east = 0.0   # knots
        self.gps_velocity_speed = 0.0  # knots
        self.boat_speed = 0.0  # m/s (derived from GPS speed)
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        # True GPS position (for noise visualization)
        self.gps_lat_true = None
        self.gps_lon_true = None
        self.boat_pos_x_true = None
        self.boat_pos_y_true = None
        self._last_visual_sail_side = 1.0
        self.base_lat = None
        self.base_lon = None
        self.boat_pos_x = 0.0
        self.boat_pos_y = 0.0
        self.overlay_markers = []
        self.is_tacking = False
        self._tacking_marker_active = False
        self.human_controlled = False  # Track if boat is under human control
        self.controller_state = ""  # Track controller state (e.g., 'tacking', 'jibing', 'broad_reach')
        
        # Track how many trail markers have been published (for efficiency)
        # This is the index in the deque of the last published marker
        self._published_trail_count = 0
        
        # TF2 buffer and listener for getting boat position from transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Timer for publishing markers
        self.timer = self.create_timer(1.0/self.update_rate, self.publish_markers)
    
        # Initialize health status - healthy when publishing successfully
        self.set_healthy("Boat visualization initialized")
        self.publish_success_count = 0
        self.publish_failure_count = 0
        
        self.get_logger().info("Argo boat visualization started")
    
    def clock_callback(self, msg):
        """Update simulated time from /clock topic for timestamp preservation"""
        # Store the clock time - this comes from ros2 bag play --clock
        # and represents the original bag's timestamps
        # msg.clock is a builtin_interfaces/Time object
        self.sim_time = msg.clock
    
    def get_current_time(self):
        """Get current time for message headers, using /clock if available (for re-recording)"""
        # If /clock topic is available (during bag playback), use that time
        # This preserves original timestamps even when playing at high speed
        if self.sim_time is not None:
            return self.sim_time
        # Otherwise, use node's clock (normal operation)
        return self.get_clock().now().to_msg()
    
    def sailing_boundaries_callback(self, msg):
        """Store sailing boundary markers for inclusion in visualization."""
        self.sailing_boundaries = msg.markers
        if len(msg.markers) > 0:
            self.get_logger().debug(f"✅ Received {len(msg.markers)} sailing boundary markers")
            # Log first marker's coordinates for debugging
            if len(msg.markers) > 0 and len(msg.markers[0].points) > 0:
                first_point = msg.markers[0].points[0]
                self.get_logger().debug(f"First boundary point: x={first_point.x:.2f}, y={first_point.y:.2f}, z={first_point.z:.2f}")
        else:
            self.get_logger().warn("Received empty sailing_boundaries message")
    
    def sailing_waypoints_callback(self, msg):
        """Store sailing waypoint markers for inclusion in visualization."""
        self.sailing_waypoints = msg.markers
        if len(msg.markers) > 0:
            self.get_logger().debug(f"Received {len(msg.markers)} sailing waypoint markers")
    
    def sailing_hazards_callback(self, msg):
        """Store sailing hazard markers for inclusion in visualization."""
        self.sailing_hazards = msg.markers
        if len(msg.markers) > 0:
            self.get_logger().debug(f"Received {len(msg.markers)} sailing hazard markers")
    
    def pose_callback(self, msg):
        """Update boat heading from pose topic"""
        heading_math = float(msg.z) % 360.0
        # Convert mathematical (counter-clockwise, 0° = East) heading to compass convention (clockwise from North)
        self.boat_heading = (450.0 - heading_math) % 360.0
    
    def accel_callback(self, msg):
        """Estimate roll and pitch from accelerometer data"""
        ax, ay, az = msg.x, msg.y, msg.z
        
        # Calculate roll and pitch from accelerometer
        self.boat_roll = math.degrees(math.atan2(ay, az))
        self.boat_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    
    def _on_parameter_change(self, parameters):
        """Handle runtime parameter changes (called when parameters are set via ros2 param set or Foxglove)"""
        from rcl_interfaces.msg import SetParametersResult
        
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            if param.name == 'visualization_scale':
                old_scale = self.visualization_scale
                self.visualization_scale = param.get_parameter_value().double_value
                self.get_logger().info(
                    f"Visualization scale changed from {old_scale}x to {self.visualization_scale}x "
                    f"(change via ros2 param set or Foxglove)"
                )
            elif param.name == 'simulation.publish_rate':
                old_rate = self.update_rate
                new_rate = param.get_parameter_value().double_value
                if new_rate > 0:
                    self.update_rate = new_rate
                    # Cancel and recreate timer with new rate
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0/self.update_rate, self.publish_markers)
                    self.get_logger().info(
                        f"Visualization update rate changed from {old_rate:.1f} Hz to {self.update_rate:.1f} Hz "
                        f"(change via ros2 param set or Foxglove)"
                    )
                else:
                    result.successful = False
                    result.reason = f"Invalid publish_rate: {new_rate} (must be > 0)"
            elif param.name == 'simulation.heading_trail_limit':
                new_limit = int(param.get_parameter_value().integer_value)
                if new_limit < 0:
                    result.successful = False
                    result.reason = f"Invalid heading_trail_limit: {new_limit} (must be >= 0)"
                else:
                    if new_limit != self.heading_trail_limit:
                        self.get_logger().info(
                            f"Heading trail limit changed from {self.heading_trail_limit} to {new_limit} "
                            f"(change via ros2 param set or Foxglove)"
                        )
                        self.heading_trail_limit = new_limit
                        # Note: We don't clear the deque or reset published count
                        # Old markers will expire naturally via their lifetime
                        # New markers will use the updated lifetime calculation
            else:
                # Allow other parameters (don't fail on unknown parameters)
                pass
        
        return result
    
    def control_callback(self, msg):
        """Update rudder and sail command positions"""
        self.rudder_cmd = msg.x  # -1 to +1
        self.sail_cmd = msg.y    # -1 to +1
    
    def wind_callback(self, msg):
        """Update wind sensor data"""
        self.wind_speed = msg.x      # m/s
        self.wind_angle = msg.y      # degrees relative to boat
        self.wind_temp = msg.z       # celsius
    
    def true_wind_callback(self, msg):
        """Update true wind direction (absolute, compass convention)"""
        self.true_wind_direction = msg.data

    def tacking_status_callback(self, msg: Bool):
        """Track whether the simulator reports a tack in progress."""
        self.is_tacking = bool(msg.data)
    
    def human_controlled_callback(self, msg: Bool):
        """Track whether the boat is under human control."""
        self.human_controlled = bool(msg.data)
    
    def controller_state_callback(self, msg: String):
        """Track controller state (e.g., 'tacking', 'jibing', 'broad_reach')."""
        self.controller_state = msg.data
    
    def _update_boat_position_from_gps(self):
        """Convert current GPS lat/lon to local map XY offsets."""
        if self.base_lat is None or self.base_lon is None:
            return
        try:
            R = 6378137.0  # Earth radius in meters
            d_lat = math.radians(self.gps_lat - self.base_lat)
            d_lon = math.radians(self.gps_lon - self.base_lon)
            self.boat_pos_y = d_lat * R
            self.boat_pos_x = d_lon * R * math.cos(math.radians(self.base_lat))
        except Exception as exc:
            self.get_logger().warn(f"Failed to update boat XY from GPS: {exc}")
    
    def _get_boat_position_from_tf(self):
        """Get boat position in map frame from TF transforms.
        
        Returns:
            tuple: (x, y) position in map frame, or falls back to GPS-based position if transform unavailable
        """
        try:
            # Use current time for transform lookup (works with /clock during playback)
            # Convert builtin_interfaces/Time to rclpy.time.Time
            time_msg = self.get_current_time()
            if self.sim_time is not None:
                # During playback, use sim_time directly
                current_time = rclpy.time.Time(seconds=time_msg.sec, nanoseconds=time_msg.nanosec)
            else:
                # During live operation, use node's clock
                current_time = self.get_clock().now()
            
            # Look up transform from map to base_link
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                current_time,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            # Extract translation (boat position in map frame)
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            # Debug: Log position periodically to diagnose offset issues
            if hasattr(self, '_tf_debug_counter'):
                self._tf_debug_counter += 1
            else:
                self._tf_debug_counter = 0
            if self._tf_debug_counter % 100 == 0:  # Every 10 seconds at 10Hz
                self.get_logger().debug(f"TF boat position: x={x:.2f}, y={y:.2f}, GPS fallback: x={self.boat_pos_x:.2f}, y={self.boat_pos_y:.2f}")
            return (x, y)
        except (TransformException, Exception) as ex:
            # Transform not available - fall back to GPS-based position
            if hasattr(self, '_tf_fallback_counter'):
                self._tf_fallback_counter += 1
            else:
                self._tf_fallback_counter = 0
            if self._tf_fallback_counter % 100 == 0:  # Log fallback periodically
                self.get_logger().debug(f"TF lookup failed, using GPS fallback: {ex}")
            return (self.boat_pos_x, self.boat_pos_y)

    def _update_heading_trail(self):
        """Store the current boat position and heading for historical visualization."""
        if self.heading_trail_limit == 0:
            self.heading_trail_last_time_s = None
            return
        if self.base_lat is None or self.base_lon is None:
            return
        pos_x = self.boat_pos_x
        pos_y = self.boat_pos_y
        if pos_x is None or pos_y is None:
            return
        if any(map(math.isnan, (pos_x, pos_y, self.boat_heading))):
            return

        yaw_rad = math.radians((90.0 - self.boat_heading) % 360.0)
        current_time_s = self.get_clock().now().nanoseconds / 1e9
        entry = HeadingTrailEntry(
            x=float(pos_x),
            y=float(pos_y),
            yaw_rad=yaw_rad,
            compass_deg=float(self.boat_heading),
            human_controlled=self.human_controlled,
            controller_state=self.controller_state
        )

        if not self.heading_trail:
            self.heading_trail.append(entry)
            self.heading_trail_last_time_s = current_time_s
            return

        last_entry = self.heading_trail[-1]
        dx = entry.x - last_entry.x
        dy = entry.y - last_entry.y
        dist = math.hypot(dx, dy)
        heading_delta = self._heading_delta(entry.compass_deg, last_entry.compass_deg)
        time_delta = current_time_s - self.heading_trail_last_time_s if self.heading_trail_last_time_s is not None else float('inf')
        
        # Check if control mode or controller state changed
        control_mode_changed = entry.human_controlled != last_entry.human_controlled
        controller_state_changed = entry.controller_state != last_entry.controller_state

        if (
            dist >= self.heading_trail_spacing_m
            or heading_delta >= self.heading_trail_heading_threshold_deg
            or time_delta >= self.heading_trail_time_spacing_s
            or control_mode_changed
            or controller_state_changed
        ):
            self.heading_trail.append(entry)
            self.heading_trail_last_time_s = current_time_s

    def _calculate_marker_lifetime(self):
        """Calculate marker lifetime based on heading_trail_limit and spacing parameters.
        
        Returns:
            Tuple of (seconds, nanoseconds) for marker lifetime.
            If heading_trail_limit is 0, returns (0, 0) for persistent markers.
        """
        if self.heading_trail_limit == 0:
            return (0, 0)  # Persistent markers
        
        # Calculate lifetime to show approximately heading_trail_limit markers
        # Use the time spacing as the basis, with a conservative multiplier
        # If markers are created every heading_trail_time_spacing_s seconds,
        # and we want heading_trail_limit markers visible, lifetime should be:
        # heading_trail_limit * heading_trail_time_spacing_s
        base_lifetime_sec = self.heading_trail_limit * self.heading_trail_time_spacing_s
        # Add a small buffer (20%) to account for variations in spacing
        lifetime_sec = int(base_lifetime_sec * 1.2)
        # Calculate fractional seconds as nanoseconds
        lifetime_nsec = int((base_lifetime_sec * 1.2 - lifetime_sec) * 1e9)
        
        return (lifetime_sec, lifetime_nsec)
    
    def _create_heading_trail_markers(self, start_idx=0):
        """Create markers representing the historical heading trace.
        
        Args:
            start_idx: Index to start creating markers from (for efficiency, only create new markers)
        """
        markers = []
        if self.heading_trail_limit == 0 or not self.heading_trail:
            return markers

        base_id = 400
        text_base_id = 10000
        arrow_length = 0.35 * self.visualization_scale
        arrow_width = 0.02 * self.visualization_scale
        arrow_height = 0.015 * self.visualization_scale
        
        # Calculate marker lifetime
        lifetime_sec, lifetime_nsec = self._calculate_marker_lifetime()
        
        # Get previous entry for state change detection
        prev_entry = None
        if start_idx > 0 and start_idx <= len(self.heading_trail):
            prev_entry = self.heading_trail[start_idx - 1]

        for idx in range(start_idx, len(self.heading_trail)):
            entry = self.heading_trail[idx]
            
            # Use unique marker ID from counter (not based on deque index)
            unique_marker_id = self._trail_marker_id_counter
            self._trail_marker_id_counter += 1
            # Wrap around at 100000 to avoid ID overflow (markers expire via lifetime anyway)
            if self._trail_marker_id_counter >= 100000:
                self._trail_marker_id_counter = 0
            
            marker = Marker()
            marker.header = Header()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_current_time()

            marker.ns = "argo_heading_trail"
            marker.id = base_id + unique_marker_id
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.frame_locked = True

            marker.pose.position.x = entry.x
            marker.pose.position.y = entry.y
            marker.pose.position.z = 0.05 * self.visualization_scale

            marker.pose.orientation.w = math.cos(entry.yaw_rad * 0.5)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = math.sin(entry.yaw_rad * 0.5)

            marker.scale.x = arrow_length
            marker.scale.y = arrow_width
            marker.scale.z = arrow_height

            # Color based on control mode and controller state at the time of this trail entry
            marker_color = None
            label_text = ""
            
            if entry.human_controlled:
                # Human control - red
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
                marker_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                label_text = "HUMAN"
            else:
                # Autonomous control - color based on controller state
                state = entry.controller_state.lower()
                if 'tacking' in state:
                    # Tacking - green
                    marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.5)
                    marker_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                    label_text = "TACKING"
                elif 'jibing' in state or 'jibe' in state:
                    # Jibing - orange/yellow
                    marker.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.5)
                    marker_color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=1.0)
                    label_text = "JIBING"
                elif 'return_to_home' in state:
                    # Return to home - purple/magenta
                    marker.color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=0.5)
                    marker_color = ColorRGBA(r=0.8, g=0.0, b=0.8, a=1.0)
                    label_text = "RTH"
                elif 'broad_reach' in state:
                    # Broad reach - cyan/blue
                    marker.color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.5)
                    marker_color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=1.0)
                    label_text = "BROAD REACH"
                elif 'proportional' in state:
                    # Proportional - cyan/blue
                    marker.color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.5)
                    marker_color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=1.0)
                    label_text = "PROPORTIONAL"
                elif 'wind_aware' in state:
                    # Wind aware - cyan/blue
                    marker.color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=0.5)
                    marker_color = ColorRGBA(r=0.2, g=0.2, b=1.0, a=1.0)
                    label_text = "WIND AWARE"
                else:
                    # Unknown state - lighter blue/gray
                    marker.color = ColorRGBA(r=0.5, g=0.5, b=0.8, a=0.5)
                    marker_color = ColorRGBA(r=0.5, g=0.5, b=0.8, a=1.0)
                    label_text = entry.controller_state.upper() if entry.controller_state else "AUTO"
            
            # Set marker lifetime (markers will expire naturally)
            marker.lifetime.sec = lifetime_sec
            marker.lifetime.nanosec = lifetime_nsec

            markers.append(marker)
            
            # Check if this is a state change (first marker or state different from previous)
            is_state_change = False
            if prev_entry is None:
                # First marker - always add label
                is_state_change = True
            elif (entry.human_controlled != prev_entry.human_controlled or
                  entry.controller_state != prev_entry.controller_state):
                # State changed - add label
                is_state_change = True
            
            if is_state_change and label_text:
                # Create text label marker
                text_marker = Marker()
                text_marker.header = Header()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = self.get_current_time()
                
                text_marker.ns = "argo_heading_trail_labels"
                text_marker.id = text_base_id + unique_marker_id
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.frame_locked = True
                
                # Position above the arrow
                text_marker.pose.position.x = entry.x
                text_marker.pose.position.y = entry.y
                text_marker.pose.position.z = 0.15 * self.visualization_scale  # Above the arrow
                
                # Text content
                text_marker.text = label_text
                
                # Scale (text size) - same as boat labels
                text_marker.scale.z = self.TEXT_MARKER_SIZE * self.visualization_scale
                
                # Color matching the arrow (but fully opaque for text)
                text_marker.color = marker_color
                
                # Set label lifetime to match arrow marker
                text_marker.lifetime.sec = lifetime_sec
                text_marker.lifetime.nanosec = lifetime_nsec
                
                markers.append(text_marker)
            
            prev_entry = entry

        return markers

    @staticmethod
    def _heading_delta(a_deg, b_deg):
        """Return the smallest absolute difference between two compass headings."""
        diff = (a_deg - b_deg + 180.0) % 360.0 - 180.0
        return abs(diff)

    def velocity_callback(self, msg):
        """Update GPS velocity data"""
        self.gps_velocity_north = msg.x  # knots north
        self.gps_velocity_east = msg.y   # knots east
        self.gps_velocity_speed = msg.z  # total speed knots
        # Convert knots to meters per second for display
        self.boat_speed = float(self.gps_velocity_speed) * 0.514444
    
    def gps_callback(self, msg):
        """Update GPS position (noisy position)"""
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.gps_lat = msg.latitude
            self.gps_lon = msg.longitude
            if self.base_lat is None or self.base_lon is None:
                self.base_lat = msg.latitude
                self.base_lon = msg.longitude
            self._update_boat_position_from_gps()
    
    def gps_true_callback(self, msg):
        """Update true GPS position (for noise visualization)"""
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.gps_lat_true = msg.latitude
            self.gps_lon_true = msg.longitude
            # Update true position in local coordinates
            if self.base_lat is not None and self.base_lon is not None:
                try:
                    R = 6378137.0  # Earth radius in meters
                    d_lat = math.radians(self.gps_lat_true - self.base_lat)
                    d_lon = math.radians(self.gps_lon_true - self.base_lon)
                    self.boat_pos_y_true = d_lat * R
                    self.boat_pos_x_true = d_lon * R * math.cos(math.radians(self.base_lat))
                except Exception as exc:
                    self.get_logger().warn(f"Failed to update true boat XY from GPS: {exc}")
            else:
                # base_lat/base_lon not set yet - use GPS callback to set them
                if self.base_lat is None or self.base_lon is None:
                    self.base_lat = msg.latitude
                    self.base_lon = msg.longitude
                    self.boat_pos_y_true = 0.0
                    self.boat_pos_x_true = 0.0
                    self.get_logger().info(f"Set base location from true GPS: lat={self.base_lat:.6f}, lon={self.base_lon:.6f}")
    
    def create_boat_hull_marker(self):
        """Create a simple boat hull marker as a triangle with tip at bow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker follows boat position
        marker.header.stamp = self.get_current_time()
        
        marker.id = 1
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Hull dimensions - apply visualization scale
        # base_link coordinate system (ROS standard): +X = forward (bow), +Y = left (port), +Z = up
        hull_length = 0.65 * self.visualization_scale  # Dragonforce 65 hull length
        hull_half_length = hull_length / 2.0
        hull_half_width = 0.065 * self.visualization_scale  # Half hull width (0.13/2)
        hull_z = 0.0  # At water surface
        
        # Create triangle vertices: tip at bow, base at stern
        # Vertex 1: Tip at bow (forward, centerline)
        vertex1 = Point()
        vertex1.x = hull_half_length  # Bow (forward, +X direction)
        vertex1.y = 0.0  # Centerline
        vertex1.z = hull_z  # Water surface
        
        # Vertex 2: Stern port side (aft, port)
        vertex2 = Point()
        vertex2.x = -hull_half_length  # Stern (aft, -X direction)
        vertex2.y = -hull_half_width  # Port side (left, -Y direction)
        vertex2.z = hull_z  # Water surface
        
        # Vertex 3: Stern starboard side (aft, starboard)
        vertex3 = Point()
        vertex3.x = -hull_half_length  # Stern (aft, -X direction)
        vertex3.y = hull_half_width  # Starboard side (right, +Y direction)
        vertex3.z = hull_z  # Water surface
        
        # Create triangle using three vertices
        marker.points = [vertex1, vertex2, vertex3]
        
        # Scale not used for TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (blue for hull)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.6)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_mast_marker(self):
        """Create boat mast marker"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_current_time()
        
        marker.id = 2
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at GPS location - apply visualization scale to height
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25 * self.visualization_scale  # Half mast height
        
        # Orientation (mast is vertical regardless of boat roll/pitch for simplicity)
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Scale - apply visualization scale
        marker.scale.x = 0.01 * self.visualization_scale  # mast diameter
        marker.scale.y = 0.01 * self.visualization_scale
        marker.scale.z = 0.915 * self.visualization_scale   # mast height
        
        # Color (brown for mast)
        marker.color = ColorRGBA(r=0.6, g=0.3, b=0.0, a=1.0)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_rudder_indicator_marker(self):
        """Create rudder position indicator as gray rectangular fin below stern"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_current_time()
        
        marker.id = 3
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Rudder position: at stern, below hull
        # base_link coordinate system (ROS standard): +X = forward (bow), +Y = left (port), +Z = up
        stern_x = -0.325 * self.visualization_scale  # Half hull length back (0.65/2)
        hull_bottom_z = -0.075 * self.visualization_scale  # Bottom of hull (half of 0.15m height)
        
        # Rudder angle relative to boat centerline
        # Positive rudder_cmd = right turn = rudder deflects to starboard (negative Y, right side)
        rudder_angle_deg = self.rudder_cmd * 30.0  # Max 30 degrees
        rudder_angle_rad = math.radians(rudder_angle_deg)
        
        rudder_height = 0.3 * self.visualization_scale  # 30cm tall (vertical)
        rudder_chord = 0.10 * self.visualization_scale  # 10cm chord length (front to back)

        base_start_x = stern_x + rudder_chord  # Forward of stern by chord length
        pivot_top = Point()
        pivot_top.x = base_start_x
        pivot_top.y = 0.0
        pivot_top.z = hull_bottom_z

        pivot_bottom = Point()
        pivot_bottom.x = base_start_x
        pivot_bottom.y = 0.0
        pivot_bottom.z = hull_bottom_z - rudder_height

        cos_angle = math.cos(rudder_angle_rad)
        sin_angle = math.sin(rudder_angle_rad)
        rotated_dx = -rudder_chord * cos_angle
        rotated_dy = -rudder_chord * sin_angle

        trailing_top = Point()
        trailing_top.x = base_start_x + rotated_dx
        trailing_top.y = rotated_dy
        trailing_top.z = hull_bottom_z

        trailing_bottom = Point()
        trailing_bottom.x = base_start_x + rotated_dx
        trailing_bottom.y = rotated_dy
        trailing_bottom.z = hull_bottom_z - rudder_height

        marker.points = [
            pivot_top, trailing_top, trailing_bottom,
            pivot_top, trailing_bottom, pivot_bottom,
        ]

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.9)
        # Add overlay arrow for top-down visibility
        overlay = Marker()
        overlay.header = marker.header
        overlay.id = 103
        overlay.type = Marker.ARROW
        overlay.action = Marker.ADD
        overlay.ns = "argo_boat_overlay"
        overlay.pose.position.x = base_start_x
        overlay.pose.position.y = 0.0
        overlay.pose.position.z = hull_bottom_z
        # Flip rudder arrow to point backwards (add 180 deg = pi rad to angle)
        flipped_angle = rudder_angle_rad + math.pi
        overlay.pose.orientation.w = math.cos(flipped_angle * 0.5)
        overlay.pose.orientation.x = 0.0
        overlay.pose.orientation.y = 0.0
        overlay.pose.orientation.z = math.sin(flipped_angle * 0.5)
        overlay.scale.x = rudder_height
        overlay.scale.y = 0.005 * self.visualization_scale
        overlay.scale.z = 0.0 # no head needed
        overlay.color = marker.color
        self.overlay_markers.append(overlay)

        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_rudder_arrow_marker(self):
        """Create rudder arrow indicator: fixed at 90° to stern, length proportional to rudder angle
        
        The arrow points in the direction the boat will turn when moving forwards:
        - Right rudder (positive rudder_cmd, rudder angled to port) → boat turns right → arrow points +X (starboard)
        - Left rudder (negative rudder_cmd, rudder angled to starboard) → boat turns left → arrow points -X (port)
        - Arrow is always perpendicular to the boat centerline (90° to stern)
        - Arrow length is proportional to |rudder_cmd|, so it shrinks to zero when rudder is neutral
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_current_time()
        
        marker.id = 8  # Separate ID for rudder arrow
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at stern of boat (base_link frame, ROS standard):
        #   +X = forward/bow
        #   -X = aft/stern
        #   +Y = left/port
        #   -Y = right/starboard
        stern_x = -0.325 * self.visualization_scale  # Half hull length back (0.65/2)
        marker.pose.position.x = stern_x  # At stern position (aft, -X)
        marker.pose.position.y = 0.0  # Centerline (center of stern)
        marker.pose.position.z = 0.0  # At waterline level
        
        # Rudder command: -1.0 (full left) to +1.0 (full right)
        # Positive rudder_cmd = right turn (rudder angled to port, boat turns starboard)
        # Negative rudder_cmd = left turn (rudder angled to starboard, boat turns port)
        
        # Arrow direction: fixed at 90° to stern (perpendicular to boat centerline)
        # ROS2 ARROW markers point in +X direction of their local frame by default
        # In base_link: +X = forward, +Y = left, so perpendicular to centerline is along ±Y
        # - For positive rudder_cmd: arrow points starboard (-Y direction in base_link)
        # - For negative rudder_cmd: arrow points port (+Y direction in base_link)
        if self.rudder_cmd > 0:
            # Right turn: arrow points starboard (-Y direction in base_link)
            # Rotate 90° around Z: arrow's default +X aligns with base_link -Y (starboard)
            # 270° rotation: w=cos(135°)= -0.707, z=sin(135°)= 0.707 (but normalized)
            # Actually: 270° = -90°, so w=cos(-45°)=0.707, z=sin(-45°)=-0.707
            marker.pose.orientation.w = 0.7071067811865476  # cos(-90°/2) = cos(-45°)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = -0.7071067811865476  # sin(-90°/2) = sin(-45°)
        elif self.rudder_cmd < 0:
            # Left turn: arrow points port (+Y direction in base_link)
            # Rotate 90° around Z: arrow's default +X aligns with base_link +Y (port)
            # 90° rotation: w=cos(45°)=0.707, z=sin(45°)=0.707
            marker.pose.orientation.w = 0.7071067811865476  # cos(90°/2) = cos(45°)
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.7071067811865476  # sin(90°/2) = sin(45°)
        else:
            # Neutral: arrow length will be zero, direction doesn't matter
            # Default to starboard direction
            marker.pose.orientation.w = 0.7071067811865476
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = -0.7071067811865476
        
        # Arrow length: proportional to |rudder_cmd|
        # Base length when rudder is at full deflection (|rudder_cmd| = 1.0)
        base_length = 0.3 * self.visualization_scale
        arrow_length = base_length * abs(self.rudder_cmd)
        
        # Scale - length proportional to rudder angle, width fixed
        marker.scale.x = arrow_length  # arrow length (proportional to |rudder_cmd|)
        marker.scale.y = 0.02 * self.visualization_scale  # arrow width (fixed)
        marker.scale.z = 0.02 * self.visualization_scale  # arrow height (fixed)
        
        # Color (red for rudder arrow)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_sail_indicator_marker(self):
        """Create sail position indicator as a white triangle panel on the downwind side"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_current_time()
        
        marker.id = 4
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Mast dimensions
        mast_top_z = 0.915 * self.visualization_scale  # Top of mast (0.915m tall)
        mast_bottom_z = 0.0  # Bottom of mast at hull center
        
        # Hull dimensions
        # base_link coordinate system (ROS standard): +X = forward (bow), +Y = left (port), +Z = up
        hull_length = 0.65 * self.visualization_scale
        hull_half_length = hull_length / 2.0
        hull_top_z = 0.075 * self.visualization_scale  # Half hull height (0.15/2)
        stern_x = -hull_half_length  # Back of hull (stern is at negative X)
        
        # Determine sail side based on fully extended sail angle relative to wind direction
        # Sail should be on the side where the fully extended sail aligns best with wind direction
        # If boat is in stays (no-go zone), sail should align with wind vector
        
        # Get wind direction (where wind goes, downwind)
        if self.true_wind_direction is not None:
            # Use true wind direction (compass convention) - more accurate
            # Calculate where wind goes (downwind direction) in compass frame
            absolute_wind_to = (self.true_wind_direction + 180.0) % 360.0
            # Convert to relative to boat heading
            relative_wind_to_deg = (absolute_wind_to - self.boat_heading) % 360.0
            if relative_wind_to_deg > 180.0:
                relative_wind_to_deg -= 360.0  # Normalize to -180 to +180
        else:
            # Fallback: use wind_angle from sensor
            # wind_angle: where wind comes from relative to boat
            wind_angle_norm = self.wind_angle % 360.0
            if wind_angle_norm > 180.0:
                relative_wind_from = wind_angle_norm - 360.0
            else:
                relative_wind_from = wind_angle_norm
            # Calculate where wind goes (downwind direction)
            relative_wind_to_deg = relative_wind_from + 180.0
        
        # Check if boat is in stays (no-go zone)
        # No-go zone is read from argo.yaml parameter (typically 45° on either side of where wind comes from)
        relative_wind_from_deg = relative_wind_to_deg - 180.0
        if relative_wind_from_deg > 180.0:
            relative_wind_from_deg -= 360.0
        elif relative_wind_from_deg < -180.0:
            relative_wind_from_deg += 360.0
        
        is_in_stays = abs(relative_wind_from_deg) < self.no_go_zone_angle
        
        # Determine sail side based on where wind is going (downwind side)
        # The sail should be on the side where the wind is going, not where it's coming from
        # In base_link: +Y = port (left), -Y = starboard (right)
        # 
        # Note: The angle convention needs to match the coordinate system
        # If wind is going to port (left/+Y), the angle should be positive (90°)
        # If wind is going to starboard (right/-Y), the angle should be negative (-90°)
        # But we need to verify this matches the actual wind_angle convention
        
        # Special cases: wind going directly forward or aft - use last known side
        if abs(relative_wind_to_deg) < 5.0 or abs(abs(relative_wind_to_deg) - 180.0) < 5.0:
            sail_side = self._last_visual_sail_side
        elif relative_wind_to_deg > 0:
            # Wind going to port side → sail goes to port (+Y) [INVERTED from previous]
            sail_side = 1.0
        else:  # relative_wind_to_deg < 0
            # Wind going to starboard side → sail goes to starboard (-Y) [INVERTED from previous]
            sail_side = -1.0
        
        # Store for next time (for special cases)
        self._last_visual_sail_side = sail_side
        
        # Sail angle parameters
        max_sail_angle_deg = 60.0
        max_sail_angle_rad = math.radians(max_sail_angle_deg)
        base_aft_rad = math.pi  # 180° = aft direction (-X)
        
        # Special case: if in stays, align sail with wind vector
        if is_in_stays:
            # In stays: sail should point in the direction wind is going (align with wind vector)
            # Calculate sail angle to align with wind direction
            relative_wind_to_rad = math.radians(relative_wind_to_deg)
            # Sail angle should match wind direction (where wind goes)
            total_sail_angle_rad = relative_wind_to_rad
            # Determine sail side based on which side the wind is going to (matching normal case)
            if relative_wind_to_deg > 0:
                sail_side = 1.0  # Wind going to port, sail on port [INVERTED]
            else:
                sail_side = -1.0  # Wind going to starboard, sail on starboard [INVERTED]
        else:
            # Normal sailing: use sail trim based on command
            self._last_visual_sail_side = sail_side
            
            # Sail trim angle from sail command (-1 = sheeted in, +1 = fully eased)
            sheet_fraction = max(0.0, min(1.0, 0.5 * (self.sail_cmd + 1.0)))
            
            # Sail orientation: start centered (straight aft) and ease toward the downwind side
            # When sheet_fraction=0 (cmd = -1) → straight aft (π rad)
            # When sheet_fraction=1 (cmd = +1) → π ± max_sail_angle depending on side
            total_sail_angle_rad = base_aft_rad + sail_side * sheet_fraction * max_sail_angle_rad
        
        boom_length = hull_half_length

        vertex1 = Point()
        vertex1.x = 0.0
        vertex1.y = 0.0
        vertex1.z = mast_top_z

        vertex2 = Point()
        vertex2.x = boom_length * math.cos(total_sail_angle_rad)
        vertex2.y = boom_length * math.sin(total_sail_angle_rad)
        vertex2.z = hull_top_z + 0.10 * self.visualization_scale

        vertex3 = Point()
        vertex3.x = 0.0
        vertex3.y = 0.0
        vertex3.z =  hull_top_z +  0.1 * self.visualization_scale

        marker.points = [vertex1, vertex2, vertex3]

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8) # transparent gray
        # Add overlay arrow for top-down visibility
        overlay = Marker()
        overlay.header = marker.header
        overlay.id = 104
        overlay.type = Marker.ARROW
        overlay.action = Marker.ADD
        overlay.ns = "argo_boat_overlay"
        overlay.pose.position.x = 0.0
        overlay.pose.position.y = 0.0
        overlay.pose.position.z = hull_top_z + 0.1 * self.visualization_scale
        overlay.pose.orientation.w = math.cos(total_sail_angle_rad * 0.5)
        overlay.pose.orientation.x = 0.0
        overlay.pose.orientation.y = 0.0
        overlay.pose.orientation.z = math.sin(total_sail_angle_rad * 0.5)
        overlay.scale.x = boom_length
        overlay.scale.y = 0.02 * self.visualization_scale
        overlay.scale.z = 0.0 # flat along water
        overlay.color = marker.color # saame as sail
        self.overlay_markers.append(overlay)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_wind_vector_marker(self):
        """Create wind vector arrow showing absolute wind direction (where wind goes on the map)
        
        Wind direction convention:
        - wind_angle from sensor: "where wind comes from" relative to boat (0° = from front, 90° = from starboard)
        - boat_heading: boat's heading in compass convention (0° = North, 90° = East, 180° = South, 270° = West)
        - We want to show: "where wind goes" in absolute terms (on the map)
        
        Calculation:
        - Step 1: Calculate absolute wind direction (where wind comes from in compass frame)
          absolute_wind_from_compass = (boat_heading + wind_angle) % 360
        - Step 2: Convert "wind comes from" to "wind goes toward" (add 180°)
          absolute_wind_to_compass = (absolute_wind_from_compass + 180) % 360
        - Step 3: Convert back to relative direction for base_link frame visualization
          relative_wind_to = (absolute_wind_to_compass - boat_heading) % 360
          
        Note: The wind_angle from sensor is already relative to boat and in the correct coordinate system
        for this calculation (where 0° = from front of boat).
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"  # Use map frame so arrow shows absolute wind direction
        marker.header.stamp = self.get_current_time()
        
        marker.id = 5
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position above boat in map frame - get from TF transforms (works during playback)
        # Falls back to GPS-based position if TF not available
        boat_x, boat_y = self._get_boat_position_from_tf()
        marker.pose.position.x = boat_x
        marker.pose.position.y = boat_y
        marker.pose.position.z = 1.0 * self.visualization_scale  # Just above mast
        
        # Convert wind direction from "relative to boat, where wind comes from" 
        # to "relative to boat, where wind goes" (for visualization)
        # wind_angle: where wind comes from relative to boat (0° = from front/bow, 90° = from starboard)
        # boat_heading: compass heading (0° = North, 90° = East)
        
        # Step 1: Calculate absolute wind direction (where wind comes from in compass frame)
        # The relative wind_angle from sensor is in simulator convention, but boat_heading is now in compass convention.
        # We can either convert wind_angle OR recalculate from true wind direction (more accurate).
        if self.true_wind_direction is not None:
            # Recalculate relative wind from true wind and boat heading (both in compass convention)
            # Relative wind = (true_wind - boat_heading), normalized to -180 to +180
            relative_wind_compass = (self.true_wind_direction - self.boat_heading) % 360.0
            if relative_wind_compass > 180.0:
                relative_wind_compass -= 360.0
            absolute_wind_from = self.true_wind_direction
        else:
            # Fallback: use relative wind angle from sensor (may have coordinate system mismatch)
            # Note: This assumes wind_angle is in the same convention as boat_heading
            absolute_wind_from = (self.boat_heading + self.wind_angle) % 360.0
        
        # Publish calculated absolute wind direction for debugging
        debug_msg = Float64(data=absolute_wind_from)
        self.pub_debug_wind.publish(debug_msg)
        
        # Step 2: Convert "wind comes from" to "wind goes toward" (add 180°)
        # If wind comes from 0° (north), it goes toward 180° (south)
        absolute_wind_to = (absolute_wind_from + 180.0) % 360.0
        
        # Step 3: Convert to map frame yaw (0° = East, increasing CCW)
        # Compass 0° = North, so yaw = (90 - compass)
        yaw_deg = (90.0 - absolute_wind_to) % 360.0
        wind_angle_rad = math.radians(yaw_deg)
        marker.pose.orientation.w = math.cos(wind_angle_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(wind_angle_rad * 0.5)
        
        # Scale based on wind speed (max 2m arrow length, min 0.1m for visibility) - apply visualization scale
        wind_scale = min(max(self.wind_speed * 0.5, 0.1), 2.0)  # Scale factor with minimum
        marker.scale.x = wind_scale * self.visualization_scale
        marker.scale.y = 0.05 * self.visualization_scale
        marker.scale.z = 0.05 * self.visualization_scale
        
        # Color (green for wind, intensity based on speed, but always visible)
        wind_intensity = min(max(self.wind_speed / 10.0, 0.3), 1.0)  # Normalize to 0.3-1.0 for visibility
        marker.color = ColorRGBA(r=0.0, g=wind_intensity, b=0.0, a=0.2)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_velocity_vector_marker(self):
        """Create GPS velocity vector arrow
        
        GPS velocity is in ENU frame (map frame):
        - velocity_north: component toward north (positive Y in ENU)
        - velocity_east: component toward east (positive X in ENU)
        
        We need to convert from ENU absolute direction to base_link relative direction.
        base_link: +Y = forward (bow), +X = starboard
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_current_time()
        
        marker.id = 6
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at boat center - apply visualization scale
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1 * self.visualization_scale  # Slightly above hull
        
        # Calculate absolute velocity heading in ENU frame (map frame)
        # atan2(east, north) gives angle from north (0° = north, 90° = east)
        absolute_velocity_heading = math.degrees(math.atan2(self.gps_velocity_east, self.gps_velocity_north))
        # Normalize to 0-360 range
        if absolute_velocity_heading < 0:
            absolute_velocity_heading += 360.0
        
        # Convert from absolute (map frame) to relative (base_link frame)
        # base_link +X = forward = boat heading direction (ROS standard)
        relative_velocity_heading = (absolute_velocity_heading - self.boat_heading) % 360.0
        # Normalize to -180 to +180 range
        if relative_velocity_heading > 180.0:
            relative_velocity_heading -= 360.0
        
        # Convert to radians for quaternion calculation
        velocity_heading_rad = math.radians(relative_velocity_heading)
        
        marker.pose.orientation.w = math.cos(velocity_heading_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(velocity_heading_rad * 0.5)
        
        # Scale based on speed (max 1m arrow length) - apply visualization scale
        velocity_scale = min(self.gps_velocity_speed * 0.1, 1.0)  # Scale factor
        marker.scale.x = velocity_scale * self.visualization_scale
        marker.scale.y = 0.03 * self.visualization_scale
        marker.scale.z = 0.03 * self.visualization_scale
        
        # Color (yellow for velocity)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.5)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_heading_arrow_marker(self):
        """Create boat heading direction arrow
        
        base_link coordinate system (ROS standard):
        - +X = forward (bow direction)
        - +Y = left (port side)
        - +Z = up
        The arrow should point in +X direction (forward/bow)
        ROS2 ARROW markers default to pointing in +X direction, so identity quaternion works
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_current_time()
        
        marker.id = 7
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at bow of boat (in base_link frame, +X is forward) - apply visualization scale
        marker.pose.position.x = 0.35 * self.visualization_scale  # Front of boat (bow, +X direction)
        marker.pose.position.y = 0.0  # Centerline
        marker.pose.position.z = 0.1 * self.visualization_scale
        
        # Heading direction arrow points forward (+X direction in base_link)
        # ROS2 ARROW markers default to pointing in +X direction, so identity quaternion works
        marker.pose.orientation.w = 1.0  # Identity quaternion (points in +X direction)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Scale - apply visualization scale
        marker.scale.x = self.gps_velocity_speed * 0.2 * self.visualization_scale  # arrow length
        marker.scale.y = 0.02 * self.visualization_scale # arrow width
        marker.scale.z = 0.01 * self.visualization_scale
        
        # Color (transparent red for human control, cyan for autonomous)
        if self.human_controlled:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
        else:
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_rudder_label_marker(self):
        """Create optional text label for rudder indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_current_time()
        
        marker.id = 10  # Separate ID range for labels (10-16)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"  # Separate namespace for labels (can be toggled independently)
        
        # Position near rudder indicator (slightly offset) - apply visualization scale
        marker.pose.position.x = -0.3 * self.visualization_scale  # At stern
        marker.pose.position.y = 0.15 * self.visualization_scale  # Offset to the side
        marker.pose.position.z = -0.1 * self.visualization_scale  # Slightly below water
        
        # Text content with value
        rudder_angle_deg = self.rudder_cmd * 30.0  # Max 30 degrees
        marker.text = f"Rudder: {rudder_angle_deg:.1f}°"
        
        # Scale (text size) - apply visualization scale
        marker.scale.z = self.TEXT_MARKER_SIZE * self.visualization_scale
        
        # Color matching rudder indicator (red)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_sail_label_marker(self):
        """Create optional text label for sail indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_current_time()
        
        marker.id = 11
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"
        
        # Position near sail indicator (at mast) - apply visualization scale
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.2 * self.visualization_scale  # Offset to the side
        marker.pose.position.z = 0.5 * self.visualization_scale  # Above sail
        
        # Text content with value
        sail_angle_deg = self.sail_cmd * 45.0  # Max 45 degrees
        marker.text = f"Sail: {sail_angle_deg:.1f}°"
        
        # Scale - apply visualization scale
        marker.scale.z = self.TEXT_MARKER_SIZE * self.visualization_scale
        
        # Color matching sail indicator (white, but darker for visibility)
        marker.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_wind_label_marker(self):
        """Create optional text label for wind vector, positioned at the middle of the wind arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"  # Use same frame as wind arrow
        marker.header.stamp = self.get_current_time()
        
        marker.id = 12
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"
        
        # Calculate wind direction for positioning (same calculation as wind arrow)
        if self.true_wind_direction is not None:
            absolute_wind_from = self.true_wind_direction
        else:
            absolute_wind_from = (self.boat_heading + self.wind_angle) % 360.0
        
        # Convert "wind comes from" to "wind goes toward" (add 180°)
        absolute_wind_to = (absolute_wind_from + 180.0) % 360.0
        
        # Calculate arrow length (same as wind arrow marker)
        wind_scale = min(max(self.wind_speed * 0.5, 0.1), 2.0)
        arrow_length = wind_scale * self.visualization_scale
        
        # Position at middle of wind arrow: start position + half arrow length in wind direction
        # Wind arrow starts at boat position, points in absolute_wind_to direction
        # Convert absolute_wind_to (compass: 0°=North) to map frame (0°=East, CCW)
        wind_yaw_deg = (90.0 - absolute_wind_to) % 360.0
        wind_yaw_rad = math.radians(wind_yaw_deg)
        
        # Get boat position from TF (works during playback)
        boat_x, boat_y = self._get_boat_position_from_tf()
        
        # Calculate midpoint offset (half arrow length in wind direction)
        midpoint_offset = arrow_length * 0.5
        marker.pose.position.x = boat_x + midpoint_offset * math.cos(wind_yaw_rad)
        marker.pose.position.y = boat_y + midpoint_offset * math.sin(wind_yaw_rad)
        marker.pose.position.z = 1.0 * self.visualization_scale  # Same height as wind arrow start
        
        # Text content with values
        marker.text = f"Wind: {self.wind_speed:.1f} m/s @ {self.wind_angle:.1f}°"
        
        # Scale - apply visualization scale
        marker.scale.z = self.TEXT_MARKER_SIZE * self.visualization_scale
        
        # Color matching wind vector (green)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_heading_label_marker(self):
        """Create optional text label for heading indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_current_time()
        
        marker.id = 13
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"
        
        # Position near heading arrow (at bow) - apply visualization scale
        marker.pose.position.x = 0.5 * self.visualization_scale  # Forward of bow
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2 * self.visualization_scale  # Above heading arrow
        
        # Text content with value
        marker.text = f"{self.boat_heading:.1f}° {self.boat_speed:.1f} m/s"
        
        # Scale - apply visualization scale
        marker.scale.z = self.TEXT_MARKER_SIZE * self.visualization_scale
        
        # Color matching heading arrow (cyan)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker

    def create_controller_status_marker(self):
        """Create text banner showing controller state and maneuver.
        
        Parses controller_state which can be:
        - Just a state: "tacking", "jibing", "toward_middle", etc.
        - State with maneuver: "Toward Middle (SAILING)", "Turning Around (TACKING)", etc.
        
        Displays abbreviated state and maneuver above the boat.
        """
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_current_time()

        marker.id = 14
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.0 * self.visualization_scale

        # Parse controller_state to extract goal state and maneuver
        # Format can be: "Toward Middle (SAILING)" or "tacking" or "jibing"
        state_text = self.controller_state.strip() if self.controller_state else ""
        
        # Define abbreviations for states and maneuvers
        state_abbrev = {
            'toward_middle': 'TO-MID',
            'toward middle': 'TO-MID',
            'launching': 'LAUNCH',
            'crossing_through': 'CROSS',
            'crossing through': 'CROSS',
            'turning_around': 'TURN',
            'turning around': 'TURN',
            'tacking_upwind': 'UPWIND',
            'tacking upwind': 'UPWIND',
            'proportional': 'PROP',
            'wind_aware': 'WIND',
            'return_to_home': 'RTH',
            'patrol': 'PATROL',
            'crosser': 'CROSS',
            'human': 'HUMAN',
        }
        
        maneuver_abbrev = {
            'sailing': '⛵',
            'tacking': '↻TACK',
            'jibing': '↺JIBE',
            'tack': '↻TACK',
            'jibe': '↺JIBE',
        }
        
        # Try to parse "State (MANEUVER)" format
        if '(' in state_text and ')' in state_text:
            # Extract state and maneuver
            parts = state_text.split('(')
            goal_state = parts[0].strip().lower()
            maneuver = parts[1].replace(')', '').strip().lower()
            
            # Get abbreviations
            goal_abbrev = state_abbrev.get(goal_state, goal_state[:6].upper())
            maneuver_abbrev_text = maneuver_abbrev.get(maneuver, maneuver[:4].upper())
            
            marker.text = f"{goal_abbrev}\n{maneuver_abbrev_text}"
        else:
            # Simple state name - check if it's a maneuver or state
            state_lower = state_text.lower()
            if state_lower in ['tacking', 'jibing', 'sailing', 'tack', 'jibe']:
                # It's a maneuver
                marker.text = maneuver_abbrev.get(state_lower, state_text[:6].upper())
            else:
                # It's a state
                marker.text = state_abbrev.get(state_lower, state_text[:6].upper())

        marker.scale.z = self.TEXT_MARKER_SIZE * self.visualization_scale

        # Color based on maneuver type
        state_lower = state_text.lower()
        if 'tacking' in state_lower or 'tack' in state_lower:
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.85)  # Green for tacking
        elif 'jibing' in state_lower or 'jibe' in state_lower:
            marker.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.85)  # Orange for jibing
        else:
            marker.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.75)  # Blue for normal sailing

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker
    
    def create_gps_noise_marker(self):
        """Create a transparent line marker showing GPS position noise (from true to noisy position).
        
        Returns:
            Marker or None: Line marker connecting true and noisy GPS positions, or None if not available
        """
        # Only show if we have both true and noisy positions
        if (self.boat_pos_x_true is None or self.boat_pos_y_true is None or
            self.boat_pos_x is None or self.boat_pos_y is None):
            return None
        
        # Calculate distance between true and noisy positions
        dx = self.boat_pos_x - self.boat_pos_x_true
        dy = self.boat_pos_y - self.boat_pos_y_true
        distance = math.sqrt(dx**2 + dy**2)
        
        # Log first time we create a marker (for debugging)
        if not hasattr(self, '_gps_noise_marker_created'):
            self._gps_noise_marker_created = True
            self.get_logger().info(
                f"GPS noise marker created: distance={distance:.2f}m, "
                f"true=({self.boat_pos_x_true:.2f}, {self.boat_pos_y_true:.2f}), "
                f"noisy=({self.boat_pos_x:.2f}, {self.boat_pos_y:.2f})"
            )
        
        # Only show marker if there's significant noise (more than 0.01m to catch small noise)
        # Lower threshold helps visualize even small GPS errors
        if distance < 0.01:
            return None
        
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"  # Use map frame for GPS positions
        marker.header.stamp = self.get_current_time()
        
        marker.id = 15
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.ns = "gps_noise"
        
        # Create line from true position to noisy position
        true_point = Point()
        true_point.x = self.boat_pos_x_true
        true_point.y = self.boat_pos_y_true
        true_point.z = 0.1  # Slightly above ground for visibility
        
        noisy_point = Point()
        noisy_point.x = self.boat_pos_x
        noisy_point.y = self.boat_pos_y
        noisy_point.z = 0.1  # Slightly above ground for visibility
        
        marker.points = [true_point, noisy_point]
        
        # Very transparent red line to show noise
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)  # Very transparent red
        
        # Thin line
        marker.scale.x = 0.05  # Line width in meters
        
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def publish_markers(self):
        """Publish all visualization markers"""
        try:
            marker_array = MarkerArray()
            
            self._update_heading_trail()
            
            # Note: We no longer need to check for deque trimming or clear markers
            # Markers expire naturally via their lifetime, and the deque is unlimited

            marker_array.markers.append(self.create_boat_hull_marker())
            marker_array.markers.append(self.create_mast_marker())
            marker_array.markers.append(self.create_rudder_indicator_marker())  # Gray triangle
            marker_array.markers.append(self.create_rudder_arrow_marker())  # Red arrow
            marker_array.markers.append(self.create_sail_indicator_marker())  # White triangle
            marker_array.markers.append(self.create_wind_vector_marker())
            # marker_array.markers.append(self.create_velocity_vector_marker())
            marker_array.markers.append(self.create_heading_arrow_marker())

            # Add NEW historical heading markers only (efficiency optimization)
            # Markers have finite lifetimes and expire naturally, so we only need to publish new ones
            if self.heading_trail_limit > 0 and self.heading_trail:
                num_trail_markers = len(self.heading_trail)
                if num_trail_markers > self._published_trail_count:
                    # Only create markers for new entries
                    new_markers = self._create_heading_trail_markers(start_idx=self._published_trail_count)
                    marker_array.markers.extend(new_markers)
                    self._published_trail_count = num_trail_markers

            # Add overlay markers (e.g., top-down indicators) and clear buffer
            if self.overlay_markers:
                marker_array.markers.extend(self.overlay_markers)
                self.overlay_markers = []
            
            # Add optional text labels (in separate namespace for independent toggling)
            marker_array.markers.append(self.create_rudder_label_marker())
            marker_array.markers.append(self.create_sail_label_marker())
            marker_array.markers.append(self.create_wind_label_marker())
            marker_array.markers.append(self.create_heading_label_marker())
            
            # Show controller status marker when controller is active
            # Controller state comes from /controller/state topic published by controller.py
            if self.controller_state and self.controller_state.strip():
                marker_array.markers.append(self.create_controller_status_marker())
                self._tacking_marker_active = True
            elif self._tacking_marker_active:
                # Delete marker if controller state is no longer available
                delete_marker = Marker()
                delete_marker.header = Header()
                delete_marker.header.frame_id = "base_link"
                delete_marker.header.stamp = self.get_current_time()
                delete_marker.ns = "argo_boat_labels"
                delete_marker.id = 14
                delete_marker.action = Marker.DELETE
                marker_array.markers.append(delete_marker)
                self._tacking_marker_active = False
            
            # Add GPS noise visualization (transparent line from true to noisy position)
            # Always try to create the marker - it will return None if conditions aren't met
            noise_marker = self.create_gps_noise_marker()
            if noise_marker:
                marker_array.markers.append(noise_marker)
            
            # Add sailing area markers (boundaries, waypoints, hazards) for 3D visualization
            # Note: These come from sailing_area_publisher and may be empty initially
            # Copy markers to avoid ID conflicts and ensure they're properly included
            if self.sailing_boundaries:
                for marker in self.sailing_boundaries:
                    # Create a copy to avoid modifying the original
                    marker_copy = copy.deepcopy(marker)
                    # Offset marker IDs to avoid conflicts (boat markers use 1-7)
                    marker_copy.id += 100
                    # Ensure frame_id is set correctly
                    if not marker_copy.header.frame_id:
                        marker_copy.header.frame_id = "map"
                    # Ensure markers are visible - verify scale is set
                    if marker_copy.scale.x == 0.0:
                        marker_copy.scale.x = 1.0  # Ensure line has width
                    marker_array.markers.append(marker_copy)
            if self.sailing_waypoints:
                for marker in self.sailing_waypoints:
                    marker_copy = copy.deepcopy(marker)
                    marker_copy.id += 200  # Offset waypoint IDs
                    if not marker_copy.header.frame_id:
                        marker_copy.header.frame_id = "map"
                    marker_array.markers.append(marker_copy)
            if self.sailing_hazards:
                for marker in self.sailing_hazards:
                    marker_copy = copy.deepcopy(marker)
                    marker_copy.id += 300  # Offset hazard IDs
                    if not marker_copy.header.frame_id:
                        marker_copy.header.frame_id = "map"
                    marker_array.markers.append(marker_copy)
            
            # Log marker counts periodically for debugging
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
            # Log marker counts periodically (300 ticks ≈ 150s at 2Hz; was wrongly labeled "1s @ 10Hz")
            if self._debug_counter > 0 and self._debug_counter % 300 == 0:
                boundary_count = len(self.sailing_boundaries)
                waypoint_count = len(self.sailing_waypoints)
                hazard_count = len(self.sailing_hazards)
                total_sailing = boundary_count + waypoint_count + hazard_count
                self.get_logger().debug(f"Publishing {len(marker_array.markers)} total markers "
                                     f"({boundary_count} boundaries, {waypoint_count} waypoints, {hazard_count} hazards)")
                if total_sailing == 0:
                    self.get_logger().warn("No sailing area markers received - check sailing_area_publisher is running")
            
            # Publish marker array (this is the primary publication method)
            self.marker_array_pub.publish(marker_array)
            
            # Note: Individual marker publishing removed to prevent duplicates in Foxglove
            # Foxglove should subscribe to /visualization_marker_array, not /visualization_marker
            # Individual markers were only for debugging and caused duplicate visualization
            
            # Update health status - successful publishing
            self.publish_success_count += 1
            if self.publish_success_count % 10 == 0:  # Update health every 10 successful publishes
                # Log at debug level to reduce chatter (health status still tracked internally)
                self.get_logger().debug(f"Health status: HEALTHY - Publishing visualization markers successfully (count: {self.publish_success_count})")
                # Update internal health status without triggering INFO log
                # Only update if health actually changed to avoid unnecessary logging
                if not self.health_status:
                    self.set_healthy(f"Publishing visualization markers successfully (count: {self.publish_success_count})")
                else:
                    # Just update details silently
                    self.health_details = f"Publishing visualization markers successfully (count: {self.publish_success_count})"
                
        except Exception as e:
            # Update health status - publishing failure
            self.publish_failure_count += 1
            self.set_unhealthy(f"Visualization publishing failed: {e}")
            self.get_logger().error(f"Error publishing visualization markers: {e}")

def main(args=None):
    parser = ArgoBaseNode.create_standard_parser(
        'Argo Boat 3D Visualization Node',
        epilog="""
This ROS2 node publishes standard visualization markers for rendering the Argo sailboat state
in RViz and the Foxglove 3D panel. Intended for operator situational awareness during 
development and testing. Does not affect control logic.

What is visualized:
- Boat hull and mast as basic geometry
- Rudder position indicator
- Sail position indicator
- Wind vector arrow
- GPS velocity vector
- Boat heading arrow
- Roll/pitch indicators derived from accelerometer data

TOPICS:
  Publishes:
    /visualization_marker (visualization_msgs/Marker): Individual markers
    /visualization_marker_array (visualization_msgs/MarkerArray): All markers together
    /argo_boat_visualization_health: Bool - Node health status (ArgoBaseNode)

  Subscribes:
    /pose (geometry_msgs/Vector3): Boat heading (z-component)
    /accel (geometry_msgs/Vector3): IMU accelerometer for roll/pitch
    /rudder_sail_servo (geometry_msgs/Vector3): Final executed rudder and sail commands (preferred)
    /rudder_sail_radio (geometry_msgs/Vector3): Raw keyboard/radio input (fallback for simulation)
    /rudder_sail_cmd (geometry_msgs/Vector3): Autonomous rudder and sail commands (fallback)
    /anem_speed_angle_temp (geometry_msgs/Vector3): Wind data
    /gps_velocity (geometry_msgs/Vector3): GPS velocity vector
    /fix (sensor_msgs/NavSatFix): GPS position

SERVICES:
  /argo_boat_visualization/health: Trigger - Health status service endpoint

HEALTH CRITERIA:
  - Healthy when successfully publishing visualization markers
  - Unhealthy when marker publishing fails

Coordinate Frame:
  All markers are published in the 'map' frame for consistent 3D visualization.
        """
    )
    
    try:
        ArgoBaseNode.run_node(ArgoBoatVisualization, args, parser)
    except Exception as e:
        print(f"CRITICAL: Failed to initialize Boat Visualization node: {e}")
        print("CRITICAL: Check ROS2 environment and visualization dependencies.")
        sys.exit(1)

if __name__ == '__main__':
    main()
