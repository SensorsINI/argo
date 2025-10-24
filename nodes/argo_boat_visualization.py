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
- /rudder_sail_cmd (geometry_msgs/Vector3): Rudder and sail commands
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

# Import ArgoBaseNode
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode

UPDATE_RATE = 1  # Hz

class ArgoBoatVisualization(ArgoBaseNode):
    def __init__(self, debug_mode=False):
        super().__init__('argo_boat_visualization')
        
        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Subscribers
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Vector3, '/accel', self.accel_callback, 10)
        self.create_subscription(Vector3, '/rudder_sail_cmd', self.control_callback, 10)
        self.create_subscription(Vector3, '/anem_speed_angle_temp', self.wind_callback, 10)
        self.create_subscription(Vector3, '/gps_velocity', self.velocity_callback, 10)
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        
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
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 1
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
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
        
        # Scale (boat dimensions in meters)
        marker.scale.x = 0.65  # Dragonforce 65 hull length
        marker.scale.y = 0.13  # hull width
        marker.scale.z = 0.05  # hull height
        
        # Color (blue for hull)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)
        
        return marker
    
    def create_mast_marker(self):
        """Create boat mast marker"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 2
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position at GPS location
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25  # Half mast height
        
        # Orientation (mast is vertical regardless of boat roll/pitch for simplicity)
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        
        # Scale
        marker.scale.x = 0.01  # mast diameter
        marker.scale.y = 0.01
        marker.scale.z = 0.5   # mast height
        
        # Color (brown for mast)
        marker.color = ColorRGBA(r=0.6, g=0.3, b=0.0, a=1.0)
        
        return marker
    
    def create_rudder_indicator_marker(self):
        """Create rudder position indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 3
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at stern of boat
        marker.pose.position.x = -0.3  # Behind boat center
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        
        # Orientation based on rudder command
        rudder_angle_deg = self.rudder_cmd * 30.0  # Max 30 degrees
        rudder_angle_rad = math.radians(rudder_angle_deg)
        
        marker.pose.orientation.w = math.cos(rudder_angle_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(rudder_angle_rad * 0.5)
        
        # Scale
        marker.scale.x = 0.2  # arrow length
        marker.scale.y = 0.02 # arrow width
        marker.scale.z = 0.02
        
        # Color (red for rudder)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        
        return marker
    
    def create_sail_indicator_marker(self):
        """Create sail position indicator"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 4
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at mast
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.4  # Up the mast
        
        # Orientation based on sail command (sail trim angle)
        sail_angle_deg = self.sail_cmd * 45.0  # Max 45 degrees
        sail_angle_rad = math.radians(sail_angle_deg)
        
        marker.pose.orientation.w = math.cos(sail_angle_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = math.sin(sail_angle_rad * 0.5)
        marker.pose.orientation.z = 0.0
        
        # Scale
        marker.scale.x = 0.3  # sail length
        marker.scale.y = 0.05 # sail width
        marker.scale.z = 0.01
        
        # Color (white for sail)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
        
        return marker
    
    def create_wind_vector_marker(self):
        """Create wind vector arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 5
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position above boat
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.6  # Above mast
        
        # Wind direction relative to boat heading
        wind_absolute_angle = self.boat_heading + self.wind_angle
        wind_angle_rad = math.radians(wind_absolute_angle)
        
        marker.pose.orientation.w = math.cos(wind_angle_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(wind_angle_rad * 0.5)
        
        # Scale based on wind speed (max 2m arrow length)
        wind_scale = min(self.wind_speed * 0.5, 2.0)  # Scale factor
        marker.scale.x = wind_scale
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        
        # Color (green for wind, intensity based on speed)
        wind_intensity = min(self.wind_speed / 10.0, 1.0)  # Normalize to 0-1
        marker.color = ColorRGBA(r=0.0, g=wind_intensity, b=0.0, a=1.0)
        
        return marker
    
    def create_velocity_vector_marker(self):
        """Create GPS velocity vector arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 6
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at boat center
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1  # Slightly above hull
        
        # Velocity direction
        velocity_heading = math.atan2(self.gps_velocity_east, self.gps_velocity_north)
        
        marker.pose.orientation.w = math.cos(velocity_heading * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(velocity_heading * 0.5)
        
        # Scale based on speed (max 1m arrow length)
        velocity_scale = min(self.gps_velocity_speed * 0.1, 1.0)  # Scale factor
        marker.scale.x = velocity_scale
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        
        # Color (yellow for velocity)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        
        return marker
    
    def create_heading_arrow_marker(self):
        """Create boat heading direction arrow"""
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = 7
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position at bow of boat
        marker.pose.position.x = 0.35  # Front of boat
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.1
        
        # Heading direction
        heading_rad = math.radians(self.boat_heading)
        
        marker.pose.orientation.w = math.cos(heading_rad * 0.5)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(heading_rad * 0.5)
        
        # Scale
        marker.scale.x = 0.2  # arrow length
        marker.scale.y = 0.03 # arrow width
        marker.scale.z = 0.03
        
        # Color (cyan for heading)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        
        return marker
    
    def publish_markers(self):
        """Publish all visualization markers"""
        try:
            marker_array = MarkerArray()
            
            # Add all markers
            marker_array.markers.append(self.create_boat_hull_marker())
            marker_array.markers.append(self.create_mast_marker())
            marker_array.markers.append(self.create_rudder_indicator_marker())
            marker_array.markers.append(self.create_sail_indicator_marker())
            marker_array.markers.append(self.create_wind_vector_marker())
            marker_array.markers.append(self.create_velocity_vector_marker())
            marker_array.markers.append(self.create_heading_arrow_marker())
            
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
    /rudder_sail_cmd (geometry_msgs/Vector3): Rudder and sail commands
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
