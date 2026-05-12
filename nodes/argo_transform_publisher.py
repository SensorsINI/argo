#!/usr/bin/env python3
"""
Argo Transform Publisher for 3D Visualization
=============================================

This ROS2 node publishes coordinate frame transforms needed for 3D visualization in Foxglove.
It establishes the relationship between the map frame, boat frame, and sensor frames.

Coordinate Frame Hierarchy:
- map (fixed world frame at GPS origin)
  └── odom (odometry frame, same as map for now)
      └── base_link (boat center)
          ├── gps_link (GPS antenna position)
          ├── compass_link (IMU/magnetometer position)
          ├── wind_sensor_link (anemometer position)
          └── rudder_link (rudder position)

Published Topics:
- /tf (geometry_msgs/TransformStamped): Dynamic transforms
- /tf_static (geometry_msgs/TransformStamped): Static transforms

Subscribed Topics:
- /fix (sensor_msgs/NavSatFix): GPS position for map frame origin
- /pose (geometry_msgs/Vector3): ENU yaw on z in degrees (0°=East, CCW), same as simulator bridge
- /accel (geometry_msgs/Vector3): IMU accelerometer for roll/pitch estimation
"""

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from sensor_msgs.msg import NavSatFix
from rosgraph_msgs.msg import Clock
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math
import numpy as np
import sys
import os
import argparse
import argcomplete
import json
import yaml
from pathlib import Path

# Import ArgoBaseNode
sys.path.append(os.path.join(os.path.dirname(__file__), 'support'))
from argo_base_node import ArgoBaseNode

class ArgoTransformPublisher(ArgoBaseNode):
    def __init__(self, debug_mode=False):
        super().__init__('argo_transform_publisher')
        
        # Transform broadcasters
        # Use standard TF QoS so Foxglove/bridges receive transforms reliably.
        # ROS2 TF conventions:
        # - /tf: best-effort, volatile
        # - /tf_static: reliable, transient-local
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
        tf_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100,
            # Some bridges (e.g., Foxglove / rosbridge stacks) request RELIABLE on `/tf`.
            # RELIABLE publishers are broadly compatible in practice for our use-case.
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_broadcaster = TransformBroadcaster(self, qos=tf_qos)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self, qos=tf_static_qos)
        
        # Frame state
        self.map_origin_set = False
        self.map_origin_lat = 0.0
        self.map_origin_lon = 0.0
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.earth_radius = 6378137.0  # meters
        self.boat_heading = 0.0
        self.boat_roll = 0.0
        self.boat_pitch = 0.0

        # Offsets in base_link frame (meters). These must match publish_static_transforms().
        # We publish base_link at boat center, while /fix is at gps_link (antenna).
        self._gps_offset_base_x = 0.1   # meters forward (+X in base_link)
        self._gps_offset_base_y = 0.0
        self._gps_offset_base_z = 0.05  # meters up (+Z in base_link) - unused for 2D translation
        
        # GPS timestamp tracking - store the timestamp of the GPS message that provided current position
        # This ensures transforms use the correct timestamp to match GPS data, preventing backwards movement
        self.last_gps_timestamp = None
        
        # Debug tracing
        self.debug_trace = self.declare_parameter('debug_trace', False).get_parameter_value().bool_value
        self.tf_publish_counter = 0
        self.last_gps_pos = None
        self.last_gps_time = None
        
        if self.debug_trace:
            from rclpy.logging import LoggingSeverity
            self.get_logger().set_level(LoggingSeverity.DEBUG)
            self.get_logger().info('🔍 Transform publisher trace debugging ENABLED')
        
        # Subscribe to sensor data
        self.create_subscription(NavSatFix, '/fix', self.gps_callback, 10)
        self.create_subscription(Vector3, '/pose', self.pose_callback, 10)
        self.create_subscription(Vector3, '/accel', self.accel_callback, 10)
        
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
        
        # Load map name from config and set map origin from map's "home" waypoint
        # This ensures the transform publisher uses the same coordinate system as sailing_area_publisher
        self._load_map_origin_from_config()
        
        # Publish static transforms at startup
        self.publish_static_transforms()
        # foxglove_bridge often attaches to /tf_static only after a Studio client connects, so a
        # single startup publish can be missed (Foxglove: missing map→base_link). Republish like
        # sailing_area_publisher does for markers.
        self.create_timer(5.0, self.publish_static_transforms)

        # Read publish rate from shared simulation parameters (argo.yaml)
        self.declare_parameter('simulation.publish_rate', 10.0)
        self.publish_rate = self.get_parameter('simulation.publish_rate').get_parameter_value().double_value
        if self.publish_rate <= 0:
            self.publish_rate = 10.0
        
        self.get_logger().info(f"Transform publisher rate: {self.publish_rate:.1f} Hz")
        
        # Timer for dynamic transforms
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_dynamic_transforms)
        
        # Add parameter callback to handle runtime parameter changes (e.g., from Foxglove)
        self.add_on_set_parameters_callback(self._on_parameter_change)
        
        # Initialize health status - healthy when publishing successfully
        self.set_healthy("Transform publisher initialized")
        self.publish_success_count = 0
        self.publish_failure_count = 0
        
        self.get_logger().info("Argo transform publisher started")
    
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
    
    def _load_map_origin_from_config(self):
        """Load map name from ROS2 parameters and set map origin from map's 'home' waypoint.
        This ensures the transform publisher uses the same coordinate system as sailing_area_publisher."""
        try:
            # Read map name from ROS2 parameters (argo.yaml)
            self.declare_parameter('geofence_map_name', 'Argo Irchel pond sailing area')
            map_name = self.get_parameter('geofence_map_name').get_parameter_value().string_value
            
            if not map_name:
                self.get_logger().debug("No geofence_map_name parameter, will use first GPS fix for map origin")
                return
            
            # Load the map's GeoJSON file to find the "home" waypoint
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[1]  # nodes -> argo
            maps_dir = argo_dir / "foxglove" / "maps"
            geojson_path = maps_dir / f"{map_name}.geojson"
            
            if not geojson_path.exists():
                self.get_logger().warn(f"Map file not found: {geojson_path}, will use first GPS fix for map origin")
                return
            
            # Load GeoJSON and find "home" waypoint
            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                if props.get('name') == 'home' and props.get('type') == 'waypoint':
                    coords = feature['geometry']['coordinates']
                    # GeoJSON format: [longitude, latitude, altitude]
                    self.map_origin_lon = coords[0]
                    self.map_origin_lat = coords[1]
                    self.map_origin_set = True
                    self.get_logger().info(f"Map origin set from '{map_name}' home waypoint: "
                                         f"lat={self.map_origin_lat:.6f}, lon={self.map_origin_lon:.6f}")
                    return
            
            self.get_logger().warn(f"No 'home' waypoint found in map '{map_name}', will use first GPS fix for map origin")
        except Exception as e:
            self.get_logger().warn(f"Failed to load map origin from config: {e}, will use first GPS fix for map origin")
    
    def lonlat_to_xy(self, lon, lat):
        """Converts longitude/latitude to local x/y meters using an equirectangular projection."""
        if not self.map_origin_set:
            return 0.0, 0.0
        x = (math.radians(lon) - math.radians(self.map_origin_lon)) * self.earth_radius * math.cos(math.radians(self.map_origin_lat))
        y = (math.radians(lat) - math.radians(self.map_origin_lat)) * self.earth_radius
        return x, y
    
    def gps_callback(self, msg):
        """Update current position from GPS fix. Map origin is set from config if available, otherwise from first GPS fix."""
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return # Ignore invalid GPS data
        
        # Ignore zero or invalid GPS coordinates (0,0) that might indicate a reset
        if abs(msg.latitude) < 0.0001 and abs(msg.longitude) < 0.0001:
            self.get_logger().warn("Ignoring zero GPS coordinates - possible reset")
            return
        
        # Set map origin from first GPS fix only if not already set from config
        if not self.map_origin_set:
            self.map_origin_lat = msg.latitude
            self.map_origin_lon = msg.longitude
            self.map_origin_set = True
            self.get_logger().info(f"Map origin set from first GPS fix: {self.map_origin_lat:.6f}, {self.map_origin_lon:.6f}")
            if self.debug_trace:
                self.get_logger().debug(f"[TF_TRACE:GPS] Map origin set - lat={self.map_origin_lat:.8f}, lon={self.map_origin_lon:.8f}")

        # Always update the current position (but only if valid)
        # Prevent jumps by checking if position changed dramatically (more than 1km)
        if self.map_origin_set:
            # Calculate distance from current position to last known position
            dx = (msg.longitude - self.current_lon) * 111320.0 * math.cos(math.radians(self.map_origin_lat))  # meters
            dy = (msg.latitude - self.current_lat) * 111320.0  # meters
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if new position is near the map origin (home waypoint) - indicates a reset
            # Calculate distance from new position to map origin
            dx_to_origin = (msg.longitude - self.map_origin_lon) * 111320.0 * math.cos(math.radians(self.map_origin_lat))  # meters
            dy_to_origin = (msg.latitude - self.map_origin_lat) * 111320.0  # meters
            distance_to_origin = math.sqrt(dx_to_origin*dx_to_origin + dy_to_origin*dy_to_origin)
            
            # If jump is more than 100m, check if it's a reset (position near origin)
            if distance > 100.0 and self.current_lat != 0.0 and self.current_lon != 0.0:
                # Allow the jump if new position is close to origin (within 50m) - this is a reset
                # 50m threshold accounts for coordinate conversion errors while still detecting resets
                if distance_to_origin < 50.0:
                    self.get_logger().info(f"Detected simulation reset: accepting position jump of {distance:.1f}m to home location (distance to origin: {distance_to_origin:.1f}m)")
                    if self.debug_trace:
                        self.get_logger().debug(f"[TF_TRACE:GPS] Reset detected - accepting jump to origin")
                    # Continue to update position (don't return)
                else:
                    # Large jump but not near origin - likely an error, ignore it
                    self.get_logger().warn(f"Ignoring GPS position jump: {distance:.1f}m (from {self.current_lat:.6f},{self.current_lon:.6f} to {msg.latitude:.6f},{msg.longitude:.6f}) - not near origin (distance to origin: {distance_to_origin:.1f}m)")
                    if self.debug_trace:
                        self.get_logger().debug(f"[TF_TRACE:GPS] Position jump ignored - distance={distance:.1f}m, not a reset")
                    return
        
        old_lat, old_lon = self.current_lat, self.current_lon
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        
        # Store the GPS message timestamp - this is critical for correct transform ordering
        # Transforms that use this GPS position should use this timestamp, not the current clock time
        self.last_gps_timestamp = msg.header.stamp
        
        if self.debug_trace:
            # Convert to x,y for logging
            x, y = self.lonlat_to_xy(self.current_lon, self.current_lat)
            old_x, old_y = self.lonlat_to_xy(old_lon, old_lat) if old_lat != 0.0 or old_lon != 0.0 else (0.0, 0.0)
            self.get_logger().debug(f"[TF_TRACE:GPS] GPS callback - lat={self.current_lat:.8f}, lon={self.current_lon:.8f}, x={x:.6f}, y={y:.6f} (was x={old_x:.6f}, y={old_y:.6f}), timestamp={self.last_gps_timestamp}")
    
    def pose_callback(self, msg):
        """Update boat heading from pose topic"""
        self.boat_heading = msg.z  # Heading in degrees
    
    def accel_callback(self, msg):
        """Estimate roll and pitch from accelerometer data"""
        # Convert accelerometer readings to roll/pitch angles
        # Assuming accelerometer is in g units
        ax, ay, az = msg.x, msg.y, msg.z
        
        # Calculate roll and pitch from accelerometer (assuming no linear acceleration)
        # Roll (rotation around x-axis, positive when starboard side up)
        self.boat_roll = math.degrees(math.atan2(ay, az))
        
        # Pitch (rotation around y-axis, positive when bow up)
        self.boat_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    
    def publish_static_transforms(self):
        """Publish static transforms between sensor frames and base_link"""
        # Identity map->odom on /tf_static (latched). Foxglove/tf2 need this link for map-frame
        # markers; publishing map->odom only on /tf each tick made lookups fragile when stamps
        # or GPS timing diverged from visualization marker stamps.
        map_odom = TransformStamped()
        map_odom.header.stamp = self.get_current_time()
        map_odom.header.frame_id = "map"
        map_odom.child_frame_id = "odom"
        map_odom.transform.translation.x = 0.0
        map_odom.transform.translation.y = 0.0
        map_odom.transform.translation.z = 0.0
        map_odom.transform.rotation.x = 0.0
        map_odom.transform.rotation.y = 0.0
        map_odom.transform.rotation.z = 0.0
        map_odom.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(map_odom)

        # GPS antenna offset (assuming GPS is mounted forward and up from boat center)
        gps_transform = TransformStamped()
        gps_transform.header.stamp = self.get_current_time()
        gps_transform.header.frame_id = "base_link"
        gps_transform.child_frame_id = "gps_link"
        gps_transform.transform.translation.x = 0.1  # 10cm forward
        gps_transform.transform.translation.y = 0.0
        gps_transform.transform.translation.z = 0.05  # 5cm up
        gps_transform.transform.rotation.x = 0.0
        gps_transform.transform.rotation.y = 0.0
        gps_transform.transform.rotation.z = 0.0
        gps_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(gps_transform)
        
        # Compass/IMU offset (assuming IMU is at boat center)
        compass_transform = TransformStamped()
        compass_transform.header.stamp = self.get_current_time()
        compass_transform.header.frame_id = "base_link"
        compass_transform.child_frame_id = "compass_link"
        compass_transform.transform.translation.x = 0.0
        compass_transform.transform.translation.y = 0.0
        compass_transform.transform.translation.z = 0.0
        compass_transform.transform.rotation.x = 0.0
        compass_transform.transform.rotation.y = 0.0
        compass_transform.transform.rotation.z = 0.0
        compass_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(compass_transform)
        
        # Wind sensor offset (assuming anemometer is mounted on mast)
        wind_transform = TransformStamped()
        wind_transform.header.stamp = self.get_current_time()
        wind_transform.header.frame_id = "base_link"
        wind_transform.child_frame_id = "wind_sensor_link"
        wind_transform.transform.translation.x = 0.0
        wind_transform.transform.translation.y = 0.0
        wind_transform.transform.translation.z = 0.3  # 30cm up (mast height)
        wind_transform.transform.rotation.x = 0.0
        wind_transform.transform.rotation.y = 0.0
        wind_transform.transform.rotation.z = 0.0
        wind_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(wind_transform)
        
        # Rudder offset (assuming rudder is at stern)
        rudder_transform = TransformStamped()
        rudder_transform.header.stamp = self.get_current_time()
        rudder_transform.header.frame_id = "base_link"
        rudder_transform.child_frame_id = "rudder_link"
        rudder_transform.transform.translation.x = -0.2  # 20cm aft
        rudder_transform.transform.translation.y = 0.0
        rudder_transform.transform.translation.z = -0.05  # 5cm down
        rudder_transform.transform.rotation.x = 0.0
        rudder_transform.transform.rotation.y = 0.0
        rudder_transform.transform.rotation.z = 0.0
        rudder_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(rudder_transform)
        
        self.get_logger().debug("Static transforms published")
    
    def publish_dynamic_transforms(self):
        """Publish dynamic transforms for boat position and orientation"""
        if not self.map_origin_set:
            return  # Wait for GPS fix to set map origin
        
        self.tf_publish_counter += 1
        tf_id = self.tf_publish_counter
        
        try:
            if self.debug_trace:
                self.get_logger().debug(f"[TF_TRACE:{tf_id}] TF_PUBLISH_START")
            
            # map->odom is static (see publish_static_transforms). Only odom->base_link here.
            # Timestamping:
            # - Prefer GPS message timestamp when available so TF aligns with /fix during bag playback.
            # - Otherwise fall back to /clock (if present) or node time.
            if self.last_gps_timestamp is not None:
                stamp = self.last_gps_timestamp
            else:
                stamp = self.get_current_time()

            # Odom to base_link transform (boat position and orientation)
            # For now, assume boat is at map origin (0,0,0) and only rotates
            # In a full implementation, this would include GPS position converted to map coordinates
            
            # Convert heading to quaternion (yaw rotation around z-axis)
            heading_rad = math.radians(self.boat_heading)
            roll_rad = math.radians(self.boat_roll)
            pitch_rad = math.radians(self.boat_pitch)
            
            # Create rotation quaternion from roll, pitch, yaw
            cy = math.cos(heading_rad * 0.5)
            sy = math.sin(heading_rad * 0.5)
            cp = math.cos(pitch_rad * 0.5)
            sp = math.sin(pitch_rad * 0.5)
            cr = math.cos(roll_rad * 0.5)
            sr = math.sin(roll_rad * 0.5)
            
            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy
            
            odom_to_base = TransformStamped()
            odom_to_base.header.stamp = stamp
            odom_to_base.header.frame_id = "odom"
            odom_to_base.child_frame_id = "base_link"
            
            # Convert current GPS coordinates to map coordinates
            x, y = self.lonlat_to_xy(self.current_lon, self.current_lat)
            # /fix is the GPS antenna position (gps_link). base_link is boat center.
            # Convert antenna position to boat-center position by subtracting the rotated
            # base_link->gps_link offset in map/odom frame.
            cos_yaw = math.cos(heading_rad)
            sin_yaw = math.sin(heading_rad)
            gps_off_map_x = cos_yaw * self._gps_offset_base_x - sin_yaw * self._gps_offset_base_y
            gps_off_map_y = sin_yaw * self._gps_offset_base_x + cos_yaw * self._gps_offset_base_y

            odom_to_base.transform.translation.x = x - gps_off_map_x
            odom_to_base.transform.translation.y = y - gps_off_map_y
            odom_to_base.transform.translation.z = 0.0
            
            odom_to_base.transform.rotation.x = qx
            odom_to_base.transform.rotation.y = qy
            odom_to_base.transform.rotation.z = qz
            odom_to_base.transform.rotation.w = qw
            
            if self.debug_trace:
                self.get_logger().debug(f"[TF_TRACE:{tf_id}] Publishing odom->base_link - x={x:.6f}, y={y:.6f}, heading={self.boat_heading:.1f}°, lat={self.current_lat:.8f}, lon={self.current_lon:.8f}")
            
            self.tf_broadcaster.sendTransform(odom_to_base)
            
            if self.debug_trace:
                self.get_logger().debug(f"[TF_TRACE:{tf_id}] TF_PUBLISH_END - Transforms published")
            
            # Update health status - successful publishing
            if self.publish_success_count % 100 == 0:  # Update health every 1000 successful publishes
                self.set_healthy(f"Published dynamics transforms successfully") # leave out count to avoid log chatter from change detection logging
            self.publish_success_count += 1
                
        except Exception as e:
            # Update health status - publishing failure
            self.publish_failure_count += 1
            self.set_unhealthy(f"Transform publishing failed: {e}")
            self.get_logger().error(f"Error publishing dynamic transforms: {e}")
    
    def _on_parameter_change(self, parameters):
        """Handle runtime parameter changes (called when parameters are set via ros2 param set or Foxglove)"""
        from rcl_interfaces.msg import SetParametersResult
        
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            if param.name == 'simulation.publish_rate':
                old_rate = self.publish_rate
                new_rate = param.get_parameter_value().double_value
                if new_rate > 0:
                    self.publish_rate = new_rate
                    # Cancel and recreate timer with new rate
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0/self.publish_rate, self.publish_dynamic_transforms)
                    self.get_logger().info(
                        f"Transform publisher rate changed from {old_rate:.1f} Hz to {self.publish_rate:.1f} Hz "
                        f"(change via ros2 param set or Foxglove)"
                    )
                else:
                    result.successful = False
                    result.reason = f"Invalid publish_rate: {new_rate} (must be > 0)"
            else:
                # Allow other parameters (don't fail on unknown parameters)
                pass
        
        return result
    
def main(args=None):
    parser = ArgoBaseNode.create_standard_parser(
        'Argo Transform Publisher for 3D Visualization',
        epilog="""
This ROS2 node publishes coordinate frame transforms needed for 3D visualization in Foxglove.
It establishes the relationship between the map frame, boat frame, and sensor frames.

Coordinate Frame Hierarchy:
- map (fixed world frame at GPS origin)
  └── odom (odometry frame, same as map for now)
      └── base_link (boat center)
          ├── gps_link (GPS antenna position)
          ├── compass_link (IMU/magnetometer position)
          ├── wind_sensor_link (anemometer position)
          └── rudder_link (rudder position)

TOPICS:
  Publishes:
    /tf (geometry_msgs/TransformStamped): Dynamic transforms
    /tf_static (geometry_msgs/TransformStamped): Static transforms
    /argo_transform_publisher_health: Bool - Node health status (ArgoBaseNode)

  Subscribes:
    /fix (sensor_msgs/NavSatFix): GPS position for map frame origin
    /pose (geometry_msgs/Vector3): ENU yaw on z (deg, 0°=East CCW), same as simulator
    /accel (geometry_msgs/Vector3): IMU accelerometer for roll/pitch estimation

SERVICES:
  /argo_transform_publisher/health: Trigger - Health status service endpoint

HEALTH CRITERIA:
  - Healthy when successfully publishing transforms
  - Unhealthy when transform publishing fails
        """
    )
    
    try:
        ArgoBaseNode.run_node(ArgoTransformPublisher, args, parser)
    except Exception as e:
        print(f"CRITICAL: Failed to initialize Transform Publisher node: {e}")
        print("CRITICAL: Check ROS2 environment and dependencies.")
        sys.exit(1)

if __name__ == '__main__':
    main()
