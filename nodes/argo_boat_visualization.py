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

**base_link axes (as implemented in this visualization):**
- **X axis (Starboard/Port):** Positive X points to **starboard** (right side), negative X points to **port** (left side).
- **Y axis (Forward/Aft):** Positive Y points **forward** (toward bow), negative Y points **aft** (toward stern).
- **Z axis (Up/Down):** Positive Z points **upward**, negative Z points **downward**.

**Relationship between base_link and ENU:**
- The base_link frame rotates relative to ENU based on the boat's heading (yaw).
- When the boat points north (heading = 0°), base_link +Y aligns with ENU +Y (north).
- When the boat points east (heading = 90°), base_link +Y aligns with ENU +X (east).
- base_link +X always points to starboard (right side of boat) regardless of heading.
- base_link +Z always points up (same as ENU +Z).

**Visualization Markers in base_link:**
All boat markers are defined in base_link coordinates, which means they move and rotate with the boat automatically:
- The **hull marker** (cube) is centered at (0, 0, 0) with:
  - scale.x = hull width (starboard/port)
  - scale.y = hull length (forward/aft)
  - scale.z = hull height (up/down)
- The **mast marker** (cylinder) extends from (0, 0, 0) to (0, 0, mast_height).
- The **rudder marker** (triangle) is located at stern: x=0 (centerline), y=-0.325 (stern), z below hull.
- The **rudder arrow** is positioned at stern center: (0, -0.325, 0).
- The **sail marker** (triangle) extends from mast top (0, 0, mast_top) to stern area, rotated based on sail position.
- The **wind vector marker** is positioned above the mast (0, 0, above_mast), with arrow direction based on wind angle relative to boat.
- The **velocity vector marker** starts at boat center (0, 0, 0), points in direction of movement.
- The **heading arrow** points in the boat's forward direction (+Y axis in base_link).

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
from std_msgs.msg import ColorRGBA, Header
import math
import numpy as np
import sys
import os
import argparse
import argcomplete
import copy

# Import ArgoBaseNode
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode

UPDATE_RATE = 1  # Hz (match transform publisher rate for smooth visualization)

class ArgoBoatVisualization(ArgoBaseNode):
    def __init__(self, debug_mode=False):
        super().__init__('argo_boat_visualization')
        
        # Declare parameters
        self.declare_parameter('visualization_scale', 3.0)
        self.visualization_scale = self.get_parameter('visualization_scale').get_parameter_value().double_value
        
        # Log scale setting for debugging
        if self.visualization_scale != 1.0:
            self.get_logger().info(f"Visualization scale set to {self.visualization_scale}x")
        
        # Add parameter callback to handle runtime parameter changes (e.g., from Foxglove)
        self.add_on_set_parameters_callback(self._on_parameter_change)
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
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
        self.create_subscription(Vector3, '/gps_velocity', self.velocity_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        
        # Subscribe to sailing area markers for 3D visualization
        # Use TRANSIENT_LOCAL QoS to receive last published message even if subscribing late
        from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
        transient_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.sailing_boundaries = []
        self.sailing_waypoints = []
        self.sailing_hazards = []
        self.create_subscription(MarkerArray, '/sailing_boundaries', self.sailing_boundaries_callback, transient_qos)
        self.create_subscription(MarkerArray, '/sailing_waypoints', self.sailing_waypoints_callback, transient_qos)
        self.create_subscription(MarkerArray, '/sailing_hazards', self.sailing_hazards_callback, transient_qos)
        
        # State variables
        self.boat_heading = 0.0  # degrees
        self.boat_roll = 0.0     # degrees
        self.boat_pitch = 0.0    # degrees
        self.rudder_cmd = 0.0    # normalized -1 to +1
        self.sail_cmd = 0.0      # normalized -1 to +1
        self.wind_speed = 0.0    # m/s
        self.wind_angle = 0.0    # degrees relative to boat
        self.wind_temp = 0.0     # celsius
        self.gps_velocity_north = 0.0  # knots
        self.gps_velocity_east = 0.0   # knots
        self.gps_velocity_speed = 0.0  # knots
        self.gps_lat = 0.0
        self.gps_lon = 0.0
        
        # Timer for publishing markers
        self.timer = self.create_timer(1/UPDATE_RATE, self.publish_markers)  #  Hz
        
        # Initialize health status - healthy when publishing successfully
        self.set_healthy("Boat visualization initialized")
        self.publish_success_count = 0
        self.publish_failure_count = 0
        
        self.get_logger().info("Argo boat visualization started")
    
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
        self.boat_heading = msg.z
    
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
            else:
                result.successful = False
                result.reason = f"Unknown parameter: {param.name}"
        
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
    
    def velocity_callback(self, msg):
        """Update GPS velocity data"""
        self.gps_velocity_north = msg.x  # knots north
        self.gps_velocity_east = msg.y   # knots east
        self.gps_velocity_speed = msg.z  # total speed knots
    
    def gps_callback(self, msg):
        """Update GPS position"""
        if not math.isnan(msg.latitude) and not math.isnan(msg.longitude):
            self.gps_lat = msg.latitude
            self.gps_lon = msg.longitude
    
    def create_boat_hull_marker(self):
        """Create a simple boat hull marker as a triangle with tip at bow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 1
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Hull dimensions - apply visualization scale
        # base_link coordinate system: +X = starboard, +Y = forward, +Z = up
        hull_length = 0.65 * self.visualization_scale  # Dragonforce 65 hull length
        hull_half_length = hull_length / 2.0
        hull_half_width = 0.065 * self.visualization_scale  # Half hull width (0.13/2)
        hull_z = 0.0  # At water surface
        
        # Create triangle vertices: tip at bow, base at stern
        # Vertex 1: Tip at bow (forward, centerline)
        vertex1 = Point()
        vertex1.x = 0.0  # Centerline
        vertex1.y = hull_half_length  # Bow (forward)
        vertex1.z = hull_z  # Water surface
        
        # Vertex 2: Stern port side (aft, port)
        vertex2 = Point()
        vertex2.x = -hull_half_width  # Port side
        vertex2.y = -hull_half_length  # Stern (aft)
        vertex2.z = hull_z  # Water surface
        
        # Vertex 3: Stern starboard side (aft, starboard)
        vertex3 = Point()
        vertex3.x = hull_half_width  # Starboard side
        vertex3.y = -hull_half_length  # Stern (aft)
        vertex3.z = hull_z  # Water surface
        
        # Create triangle using three vertices
        marker.points = [vertex1, vertex2, vertex3]
        
        # Scale not used for TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (blue for hull)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_mast_marker(self):
        """Create boat mast marker"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
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
        """Create rudder position indicator as gray triangle below stern"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 3
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Rudder position: at stern, below hull
        # base_link coordinate system: +X = starboard, +Y = forward, +Z = up
        stern_y = -0.325 * self.visualization_scale  # Half hull length back (0.65/2)
        hull_bottom_z = -0.075 * self.visualization_scale  # Bottom of hull (half of 0.15m height)
        
        # Rudder angle relative to boat centerline
        # Positive rudder_cmd = right turn = rudder deflects to starboard (positive X)
        rudder_angle_deg = self.rudder_cmd * 30.0  # Max 30 degrees
        rudder_angle_rad = math.radians(rudder_angle_deg)
        
        # Rudder triangle dimensions
        # Rudder is a vertical fin that extends downward from hull
        rudder_height = 0.3 * self.visualization_scale  # 30cm tall (vertical)
        rudder_base_length = 0.10 * self.visualization_scale  # 10cm base length along hull
        
        # Rudder base edge: along hull (forward-aft direction), from inside hull to stern
        # Base starts forward of stern, ends at stern
        # Tip extends downward from stern end of base and rotates left/right based on rudder angle
        
        # Vertex 1: Forward end of rudder base (inside hull, at bottom)
        base_start_y = stern_y + rudder_base_length  # Forward of stern by base_length
        vertex1 = Point()
        vertex1.x = 0.0  # At centerline
        vertex1.y = base_start_y  # Forward end of base
        vertex1.z = hull_bottom_z  # At hull bottom
        
        # Vertex 2: Stern end of rudder base (at stern, at bottom)
        vertex2 = Point()
        vertex2.x = 0.0  # At centerline
        vertex2.y = stern_y  # At stern y position
        vertex2.z = hull_bottom_z  # At hull bottom
        
        # Vertex 3: Tip of rudder (below stern end of base, rotated by rudder angle)
        # Rudder rotates around vertical Z axis (horizontal rotation in XY plane)
        # Pivot point: (x=0, y=stern_y, z=hull_bottom_z) - the stern end of the base (vertex2)
        # The rudder is a vertical fin. When angle=0, it extends straight back (in -Y direction)
        # When rotated, the tip rotates horizontally around the pivot point in XY plane
        # When angle = 0: tip is at (x=0, y=stern_y-rudder_height, z=hull_bottom_z-rudder_height) - straight back and down
        # Positive angle: tip rotates to starboard (positive X) and slightly forward (positive Y)
        # Negative angle: tip rotates to port (negative X) and slightly forward (positive Y)
        vertex3 = Point()
        # Rotate tip position around pivot in XY plane
        # Initial direction when angle=0: pointing in -Y direction (straight back)
        # Rotate this direction by rudder_angle_rad
        vertex3.x = 0.0 + rudder_height * math.sin(rudder_angle_rad)  # Rotates starboard/port
        vertex3.y = stern_y - rudder_height * math.cos(rudder_angle_rad)  # Extends back when angle=0, rotates forward
        vertex3.z = hull_bottom_z - rudder_height  # Always extends straight down (vertical, Z axis)
        
        # Create triangle using three vertices
        marker.points = [vertex1, vertex2, vertex3]
        
        # Scale not used for TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (gray for rudder)
        marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.9)
        
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
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 8  # Separate ID for rudder arrow
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at stern of boat (base_link frame):
        #   +X = starboard/right
        #   -X = port/left
        #   +Y = bow/forward
        #   -Y = stern/backward
        stern_y = -0.325 * self.visualization_scale  # Half hull length back (0.65/2)
        marker.pose.position.x = 0.0  # Centerline (center of stern)
        marker.pose.position.y = stern_y  # At stern position
        marker.pose.position.z = 0.0  # At waterline level
        
        # Rudder command: -1.0 (full left) to +1.0 (full right)
        # Positive rudder_cmd = right turn (rudder angled to port, boat turns starboard)
        # Negative rudder_cmd = left turn (rudder angled to starboard, boat turns port)
        
        # Arrow direction: fixed at 90° to stern (perpendicular to boat centerline)
        # ROS2 ARROW markers point in +X direction of their local frame by default
        # To point perpendicular to hull (along ±X in base_link), we need:
        # - For positive rudder_cmd: arrow points +X (starboard) → identity orientation (no rotation)
        # - For negative rudder_cmd: arrow points -X (port) → 180° rotation around Z
        if self.rudder_cmd > 0:
            # Right turn: arrow points starboard (+X direction in base_link)
            # Identity orientation: arrow's default +X aligns with base_link +X
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
        elif self.rudder_cmd < 0:
            # Left turn: arrow points port (-X direction in base_link)
            # 180° rotation around Z: arrow's default +X aligns with base_link -X
            marker.pose.orientation.w = 0.0  # cos(180°/2) = cos(90°) = 0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 1.0  # sin(180°/2) = sin(90°) = 1
        else:
            # Neutral: arrow length will be zero, direction doesn't matter
            # Default to starboard direction (identity)
            marker.pose.orientation.w = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
        
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
        """Create sail position indicator as white triangle on downwind side"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 4
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Mast dimensions
        mast_top_z = 0.915 * self.visualization_scale  # Top of mast (0.915m tall)
        mast_bottom_z = 0.0  # Bottom of mast at hull center
        mast_mid_z = 0.1 * self.visualization_scale  # 10cm from bottom
        
        # Hull dimensions
        # base_link coordinate system: +X = starboard, +Y = forward, +Z = up
        hull_length = 0.65 * self.visualization_scale
        hull_half_length = hull_length / 2.0
        hull_top_z = 0.075 * self.visualization_scale  # Half hull height (0.15/2)
        stern_y = -hull_half_length  # Back of hull (stern is at negative Y)
        
        # Determine downwind side from wind direction
        # wind_angle: 0° = from front, 90° = from starboard (right/+y), -90° = from port (left/-y)
        # Sail goes to downwind side (opposite of where wind comes from)
        
        if abs(self.wind_angle) < 0.1:  # Wind from front/back
            sail_side = 0.0  # No side preference when wind is head-on/from-back
        else:
            sail_side = 1.0 if self.wind_angle > 0 else -1.0  # Wind blows sail to side same as wind direction, since wind direction is anemometer angle relative to boat heading
        
        # Sail trim angle from sail command
        # sail_cmd: -1 = fully out, 0 = half in, +1 = fully in (along hull)
        # Note: Hardware convention from rudder_sail_radio.py is -1 = pulled in, +1 = let out
        # So we negate sail_cmd to match visualization expectation
        # When sail_cmd = 0: sail should be halfway pulled in (not aligned with hull)
        sail_angle_deg = -self.sail_cmd * 45.0  # Negate to match visualization convention, max 45 degrees
        sail_angle_rad = math.radians(sail_angle_deg)
        
        # Sail base angle: aligned with hull length (aft direction = 180° or π radians)
        # When sail_cmd = 0 (halfway pulled in): sail angle is 0, so base angle determines position
        # From this base, sail rotates to downwind side and with trim angle
        base_aft_rad = math.pi  # 180° = aft direction (toward stern)
        
        # Rotation to downwind side: perpendicular component (90° from base)
        # When wind comes from starboard (wind_angle > 0): sail goes to port (-y), so perpendicular = +90°
        # When wind comes from port (wind_angle < 0): sail goes to starboard (+y), so perpendicular = -90°
        # sail_side is -1.0 for port, +1.0 for starboard
        # But we need: port side = +90° from aft = π + π/2 = 3π/2, starboard = π - π/2 = π/2
        # So perpendicular should be: -π/2 * sail_side
        perpendicular_component_rad = -(math.pi / 2.0) * sail_side  # -90° for port, +90° for starboard
        
        # Total sail angle: aft direction + perpendicular rotation + trim angle
        # When sail_cmd = 0 (halfway pulled in): sail_angle_rad = 0, so total = base + perpendicular
        # When sail_cmd = 0 and wind head-on (sail_side = 0): total = π (straight aft)
        # When sail_cmd = 0 and wind from starboard (sail_side = -1): total = π + π/2 = 3π/2 (aft and port)
        total_sail_angle_rad = base_aft_rad + perpendicular_component_rad + sail_angle_rad
        
        # Vertex 1: Top of mast (at boat center)
        vertex1 = Point()
        vertex1.x = 0.0
        vertex1.y = 0.0
        vertex1.z = mast_top_z
        
        # Vertex 2: 10cm above back hull (at stern), position depends on sail angle
        # When sail_cmd = 0: should be at stern (x=0 on centerline, stern_y), aligned with hull
        # When sail rotates: x offset based on sail angle from mast center
        
        # Calculate vertex2 position using parametric line equation:
        # Sail extends from mast (0,0) at angle total_sail_angle_rad
        # Parametric: x = t * cos(θ), y = t * sin(θ)
        # When y = stern_y: t = stern_y / sin(θ)
        # Then: x = stern_y * cos(θ) / sin(θ) = stern_y * cot(θ)
        # Handle edge case when sin(θ) ≈ 0 (sail perpendicular to hull)
        
        vertex2 = Point()
        vertex2.y = stern_y  # Always at stern y position
        
        # Calculate x position using parametric intersection
        # Sail extends from mast (0,0) at angle total_sail_angle_rad
        # In base_link: 0° = +X (starboard), 90° = +Y (forward), 180° = -X (port), 270° = -Y (aft)
        # total_sail_angle_rad uses math.pi (180°) as base (aft direction)
        # Parametric: x = t * cos(θ), y = t * sin(θ) where θ is measured from +X axis
        # We want vertex2 at stern (y = stern_y), so solve for x
        cos_angle = math.cos(total_sail_angle_rad)
        sin_angle = math.sin(total_sail_angle_rad)
        
        if abs(sin_angle) > 0.001:  # Normal case: not perpendicular to Y axis
            # When y = stern_y: t = stern_y / sin(θ)
            # Then: x = stern_y * cos(θ) / sin(θ) = stern_y * cot(θ)
            vertex2.x = stern_y * cos_angle / sin_angle
        else:
            # Edge case: sail is perpendicular to Y axis (sin ≈ 0)
            # When perpendicular, x is determined by sign of cos and distance to stern
            # If cos > 0: sail extends to starboard, x should be large positive
            # If cos < 0: sail extends to port, x should be large negative
            # But we want vertex2 at stern y, so use a reasonable offset
            max_x_offset = abs(stern_y) * 2.0  # Allow sail to extend 2x hull length sideways
            vertex2.x = max_x_offset if cos_angle > 0 else -max_x_offset
        
        vertex2.z = hull_top_z + 0.10 * self.visualization_scale  # 10cm above hull top
        
        # Vertex 3: 10cm from bottom of mast (at boat center)
        vertex3 = Point()
        vertex3.x = 0.0
        vertex3.y = 0.0
        vertex3.z = mast_mid_z  # 10cm from bottom (mast bottom is at 0)
        
        # Create triangle using three vertices
        marker.points = [vertex1, vertex2, vertex3]
        
        # Scale not used for TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (white for sail)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_wind_vector_marker(self):
        """Create wind vector arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 5
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position above boat - apply visualization scale
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.1 * self.visualization_scale  # Above mast
        
        # Wind direction relative to boat (wind_angle is already relative to boat heading)
        wind_angle_rad = math.radians(self.wind_angle)
        
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
        marker.color = ColorRGBA(r=0.0, g=wind_intensity, b=0.0, a=1.0)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_velocity_vector_marker(self):
        """Create GPS velocity vector arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 6
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at boat center - apply visualization scale
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1 * self.visualization_scale  # Slightly above hull
        
        # Velocity direction
        velocity_heading = math.atan2(self.gps_velocity_east, self.gps_velocity_north)
        
        marker.pose.orientation.w = math.cos(velocity_heading * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(velocity_heading * 0.5)
        
        # Scale based on speed (max 1m arrow length) - apply visualization scale
        velocity_scale = min(self.gps_velocity_speed * 0.1, 1.0)  # Scale factor
        marker.scale.x = velocity_scale * self.visualization_scale
        marker.scale.y = 0.03 * self.visualization_scale
        marker.scale.z = 0.03 * self.visualization_scale
        
        # Color (yellow for velocity)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_heading_arrow_marker(self):
        """Create boat heading direction arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 7
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at bow of boat (in base_link frame, +x is forward) - apply visualization scale
        marker.pose.position.x = 0.35 * self.visualization_scale  # Front of boat
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1 * self.visualization_scale
        
        # Heading direction arrow points forward (+x direction in base_link)
        marker.pose.orientation.w = 1.0  # Identity quaternion (points in +x direction)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Scale - apply visualization scale
        marker.scale.x = 0.2 * self.visualization_scale  # arrow length
        marker.scale.y = 0.03 * self.visualization_scale # arrow width
        marker.scale.z = 0.03 * self.visualization_scale
        
        # Color (cyan for heading)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_rudder_label_marker(self):
        """Create optional text label for rudder indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 10  # Separate ID range for labels (10-16)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"  # Separate namespace for labels (can be toggled independently)
        
        # Position near rudder indicator (slightly offset) - apply visualization scale
        marker.pose.position.x = -0.3 * self.visualization_scale  # At stern
        marker.pose.position.y = 0.15 * self.visualization_scale  # Offset to the side
        marker.pose.position.z = 0.1 * self.visualization_scale  # Slightly above
        
        # Text content with value
        rudder_angle_deg = self.rudder_cmd * 30.0  # Max 30 degrees
        marker.text = f"Rudder: {rudder_angle_deg:.1f}°"
        
        # Scale (text size) - apply visualization scale
        marker.scale.z = 0.1 * self.visualization_scale  # Text height in meters
        
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
        marker.header.stamp = self.get_clock().now().to_msg()
        
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
        marker.scale.z = 0.15 * self.visualization_scale
        
        # Color matching sail indicator (white, but darker for visibility)
        marker.color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_wind_label_marker(self):
        """Create optional text label for wind vector"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 12
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"
        
        # Position near wind vector (above boat) - apply visualization scale
        marker.pose.position.x = 0.3 * self.visualization_scale  # Offset forward
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.2 * self.visualization_scale  # Above wind arrow
        
        # Text content with values
        marker.text = f"Wind: {self.wind_speed:.1f} m/s @ {self.wind_angle:.1f}°"
        
        # Scale - apply visualization scale
        marker.scale.z = 0.1 * self.visualization_scale
        
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
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 13
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.ns = "argo_boat_labels"
        
        # Position near heading arrow (at bow) - apply visualization scale
        marker.pose.position.x = 0.5 * self.visualization_scale  # Forward of bow
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2 * self.visualization_scale  # Above heading arrow
        
        # Text content with value
        marker.text = f"Heading: {self.boat_heading:.1f}°"
        
        # Scale - apply visualization scale
        marker.scale.z = 0.1 * self.visualization_scale
        
        # Color matching heading arrow (cyan)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        
        # Lifetime
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def publish_markers(self):
        """Publish all visualization markers"""
        try:
            marker_array = MarkerArray()
            
            # Add boat visualization markers
            marker_array.markers.append(self.create_boat_hull_marker())
            marker_array.markers.append(self.create_mast_marker())
            marker_array.markers.append(self.create_rudder_indicator_marker())  # Gray triangle
            marker_array.markers.append(self.create_rudder_arrow_marker())  # Red arrow
            marker_array.markers.append(self.create_sail_indicator_marker())  # White triangle
            marker_array.markers.append(self.create_wind_vector_marker())
            marker_array.markers.append(self.create_velocity_vector_marker())
            marker_array.markers.append(self.create_heading_arrow_marker())
            
            # Add optional text labels (in separate namespace for independent toggling)
            marker_array.markers.append(self.create_rudder_label_marker())
            marker_array.markers.append(self.create_sail_label_marker())
            marker_array.markers.append(self.create_wind_label_marker())
            marker_array.markers.append(self.create_heading_label_marker())
            
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
            
            # Log marker counts periodically for better debugging
            if self._debug_counter % 300 == 0:  # Every 1 second at 10Hz
                boundary_count = len(self.sailing_boundaries)
                waypoint_count = len(self.sailing_waypoints)
                hazard_count = len(self.sailing_hazards)
                total_sailing = boundary_count + waypoint_count + hazard_count
                self.get_logger().debug(f"Publishing {len(marker_array.markers)} total markers "
                                     f"({boundary_count} boundaries, {waypoint_count} waypoints, {hazard_count} hazards)")
                if total_sailing == 0:
                    self.get_logger().warn("No sailing area markers received - check sailing_area_publisher is running")
            
            # Publish marker array
            self.marker_array_pub.publish(marker_array)
            
            # Also publish individual markers for debugging
            for marker in marker_array.markers:
                self.marker_pub.publish(marker)
            
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
