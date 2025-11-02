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

Coordinate Frame
----------------
All markers are published in the 'map' frame for consistent 3D visualization.
"""

import rclpy
from geometry_msgs.msg import Vector3, PoseStamped
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

UPDATE_RATE = 1  # Hz

class ArgoBoatVisualization(ArgoBaseNode):
    def __init__(self, debug_mode=False):
        super().__init__('argo_boat_visualization')
        
        # Declare parameters
        self.declare_parameter('visualization_scale', 1.0)
        self.visualization_scale = self.get_parameter('visualization_scale').get_parameter_value().double_value
        
        # Log scale setting for debugging
        if self.visualization_scale != 1.0:
            self.get_logger().info(f"Visualization scale set to {self.visualization_scale}x")
        
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
            self.get_logger().info(f"✅ Received {len(msg.markers)} sailing boundary markers")
            # Log first marker's coordinates for debugging
            if len(msg.markers) > 0 and len(msg.markers[0].points) > 0:
                first_point = msg.markers[0].points[0]
                self.get_logger().info(f"First boundary point: x={first_point.x:.2f}, y={first_point.y:.2f}, z={first_point.z:.2f}")
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
        """Create a simple boat hull marker"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        # Set unique identifier for this marker so visualization can update/delete it by ID
        marker.id = 1

        # Specify the marker type as a cube to represent the boat hull's simple rectangular shape
        marker.type = Marker.CUBE

        # Set action to ADD so this marker is added or updated in the visualization
        marker.action = Marker.ADD

        # Place this marker in the "argo_boat" namespace so it doesn't conflict with other boat markers
        marker.ns = "argo_boat"
        
        # Position at GPS location (simplified - in real implementation would convert lat/lon to map coordinates)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Orientation from heading, roll, pitch
        heading_rad = math.radians(self.boat_heading)
        roll_rad = math.radians(self.boat_roll)
        pitch_rad = math.radians(self.boat_pitch)
        
        # Create rotation quaternion
        cy = math.cos(heading_rad * 0.5)
        sy = math.sin(heading_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        marker.pose.orientation.w = cr * cp * cy + sr * sp * sy
        marker.pose.orientation.x = sr * cp * cy - cr * sp * sy
        marker.pose.orientation.y = cr * sp * cy + sr * cp * sy
        marker.pose.orientation.z = cr * cp * sy - sr * sp * cy
        
        # Scale (boat dimensions in meters) - apply visualization scale
        marker.scale.x = 0.65 * self.visualization_scale  # Dragonforce 65 hull length
        marker.scale.y = 0.13 * self.visualization_scale  # hull width
        marker.scale.z = 0.05 * self.visualization_scale  # hull height
        
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
        marker.scale.z = 0.5 * self.visualization_scale   # mast height
        
        # Color (brown for mast)
        marker.color = ColorRGBA(r=0.6, g=0.3, b=0.0, a=1.0)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_rudder_indicator_marker(self):
        """Create rudder position indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 3
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at stern of boat (in base_link frame, +x is forward) - apply visualization scale
        marker.pose.position.x = -0.3 * self.visualization_scale  # Behind boat center (stern)
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Orientation: rudder deflection relative to boat frame (boat heading is already in base_link transform)
        rudder_angle_deg = self.rudder_cmd * 30.0  # Max 30 degrees
        rudder_angle_rad = math.radians(rudder_angle_deg)
        
        marker.pose.orientation.w = math.cos(rudder_angle_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(rudder_angle_rad * 0.5)
        
        # Scale - apply visualization scale
        marker.scale.x = 0.2 * self.visualization_scale  # arrow length
        marker.scale.y = 0.02 * self.visualization_scale # arrow width
        marker.scale.z = 0.02 * self.visualization_scale
        
        # Color (red for rudder)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        # Lifetime - infinite so marker persists
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_sail_indicator_marker(self):
        """Create sail position indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "base_link"  # Use base_link so marker moves with boat
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 4
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.ns = "argo_boat"
        
        # Position at mast (already at boat center, just raise up) - apply visualization scale
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.4 * self.visualization_scale  # Up the mast
        
        # Orientation: sail trim angle relative to boat frame (boat heading is already in base_link transform)
        # Sail is perpendicular to boat (+y is port side) plus trim angle
        sail_angle_deg = self.sail_cmd * 45.0  # Max 45 degrees
        sail_angle_rad = math.radians(sail_angle_deg)
        
        # Sail extends perpendicular to boat (90 degrees from +x forward) plus trim
        total_angle_rad = math.pi / 2.0 + sail_angle_rad
        
        marker.pose.orientation.w = math.cos(total_angle_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = math.sin(total_angle_rad * 0.5)
        marker.pose.orientation.z = 0.0
        
        # Scale - apply visualization scale
        marker.scale.x = 0.3 * self.visualization_scale  # sail length
        marker.scale.y = 0.05 * self.visualization_scale # sail width
        marker.scale.z = 0.01 * self.visualization_scale
        
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
        marker.pose.position.z = 1.0 * self.visualization_scale  # Above mast
        
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
        marker.scale.z = 0.15 * self.visualization_scale  # Text height in meters
        
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
        marker.scale.z = 0.15 * self.visualization_scale
        
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
        marker.scale.z = 0.15 * self.visualization_scale
        
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
            marker_array.markers.append(self.create_rudder_indicator_marker())
            marker_array.markers.append(self.create_sail_indicator_marker())
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
            
            # Log marker counts every 10 seconds for better debugging
            if self._debug_counter % 10 == 0:  # Every 10 seconds at 1Hz
                boundary_count = len(self.sailing_boundaries)
                waypoint_count = len(self.sailing_waypoints)
                hazard_count = len(self.sailing_hazards)
                total_sailing = boundary_count + waypoint_count + hazard_count
                self.get_logger().info(f"Publishing {len(marker_array.markers)} total markers "
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
                self.set_healthy(f"Publishing visualization markers successfully (count: {self.publish_success_count})")
                
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
