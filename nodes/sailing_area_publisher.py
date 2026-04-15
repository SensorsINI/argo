#!/usr/bin/env python3
"""
Publish sailing area data from GeoJSON files as ROS2 markers for Foxglove visualization
"""

import json
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from rosgraph_msgs.msg import Clock
import os
from pathlib import Path
import math
import argparse
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy


class SailingAreaPublisher(Node):
    def __init__(self, map_name=None):
        super().__init__('sailing_area_publisher')
        self.map_name = map_name  # Specific map to use for origin (matches simulator bridge)
        
        # Initialize clock time for timestamp preservation (before any methods use it)
        self.sim_time = None
        
        # Determine Argo repository directory dynamically
        script_path = Path(__file__).resolve()
        self.argo_dir = script_path.parents[1]  # nodes -> argo
        self.maps_dir = self.argo_dir / "foxglove" / "maps"
        self.earth_radius = 6378137.0  # meters

        self.get_logger().info(f"Using maps directory: {self.maps_dir}")

        # RELIABLE + VOLATILE matches ros2 topic echo and foxglove_bridge DDS matching.
        # TRANSIENT_LOCAL caused no data on the WebSocket while CLI echo still worked.
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )

        # Create publishers (periodic republish below covers late subscribers)
        self.waypoint_pub = self.create_publisher(MarkerArray, '/sailing_waypoints', qos_profile)
        self.boundary_pub = self.create_publisher(MarkerArray, '/sailing_boundaries', qos_profile)
        self.hazard_pub = self.create_publisher(MarkerArray, '/sailing_hazards', qos_profile)
        
        # Load sailing area data first
        self.sailing_areas = self.load_sailing_areas()
        
        # Debug: Log loaded maps and requested map
        self.get_logger().info(f"Requested map_name: '{self.map_name}'")
        self.get_logger().info(f"Loaded sailing areas keys: {list(self.sailing_areas.keys())}")
        if self.map_name:
            self.get_logger().info(f"Map name in sailing_areas: {self.map_name in self.sailing_areas}")
            if self.map_name not in self.sailing_areas:
                self.get_logger().warn(f"⚠️ WARNING: Requested map '{self.map_name}' not found in loaded maps!")
                self.get_logger().warn(f"   Available maps: {', '.join(self.sailing_areas.keys())}")
                self.get_logger().warn(f"   Falling back to publishing all maps (may show wrong geofence)")
        
        # Find origin from 'home' waypoint in loaded maps (same as simulator bridge)
        self.origin_lon = 8.5448386  # Default fallback
        self.origin_lat = 47.3981555
        self._find_origin_from_maps()
        
        self._logged_marker_publicaton=False
        # Publish markers at startup
        self.publish_all_markers()
        # foxglove_bridge often starts after this node
        self._startup_republish_timer = self.create_timer(1.0, self._startup_republish_once)

        # Clock time for timestamp preservation during re-recording
        # Subscribe to /clock topic to get simulated time from bag playback
        # Use BEST_EFFORT reliability to match ros2 bag play --clock QoS
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            Clock, '/clock', 
            self.clock_callback, 
            clock_qos
        )
        
        # Create a timer to republish periodically (in case of reconnection)
        # Use shorter interval initially to ensure subscribers receive data
        self.republish_count = 0
        self.timer = self.create_timer(5.0, self.periodic_republish)  # Republish every 5 seconds initially
        
        # Log comprehensive startup information
        self.log_startup_info()
    
    def _startup_republish_once(self):
        self.publish_all_markers()
        self._startup_republish_timer.cancel()

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
    
    def load_sailing_areas(self):
        """Load all GeoJSON sailing area files"""
        sailing_areas = {}
        
        if not self.maps_dir.exists():
            self.get_logger().warn(f"Maps directory {self.maps_dir} does not exist")
            return sailing_areas
        
        for geojson_file in self.maps_dir.glob("*.geojson"):
            try:
                with open(geojson_file, 'r') as f:
                    data = json.load(f)
                    map_key = geojson_file.stem
                    sailing_areas[map_key] = data
                    self.get_logger().info(f"Loaded sailing area: '{map_key}' (from file: {geojson_file.name})")
            except Exception as e:
                self.get_logger().error(f"Failed to load {geojson_file}: {e}")
        
        return sailing_areas
    
    def log_startup_info(self):
        """Log comprehensive startup information about the sailing area publisher"""
        self.get_logger().debug("=" * 60)
        self.get_logger().debug("SAILING AREA PUBLISHER STARTUP INFORMATION")
        self.get_logger().debug("=" * 60)
        
        # Published topics information
        self.get_logger().debug("Published Topics:")
        self.get_logger().debug(f"  • /sailing_waypoints (visualization_msgs/MarkerArray) - Waypoint markers")
        self.get_logger().debug(f"  • /sailing_boundaries (visualization_msgs/MarkerArray) - Boundary markers")
        self.get_logger().debug(f"  • /sailing_hazards (visualization_msgs/MarkerArray) - Hazard markers")
        
        # Publishing rate information
        self.get_logger().debug("Publishing Configuration:")
        self.get_logger().debug(f"  • Initial publish: Immediate on startup")
        self.get_logger().debug(f"  • Periodic republish: Every 60.0 seconds")
        self.get_logger().debug(f"  • QoS depth: 10 messages")
        
        # Maps directory information
        self.get_logger().debug("Maps Configuration:")
        self.get_logger().debug(f"  • Maps directory: {self.maps_dir}")
        self.get_logger().debug(f"  • Directory exists: {self.maps_dir.exists()}")
        
        # Loaded sailing areas information
        self.get_logger().debug("Loaded Sailing Areas:")
        if not self.sailing_areas:
            self.get_logger().warn("  • No sailing areas loaded")
        else:
            for area_name, geojson_data in self.sailing_areas.items():
                features = geojson_data.get('features', [])
                self.get_logger().debug(f"  • {area_name}: {len(features)} features")
                
                # Count feature types
                feature_types = {}
                for feature in features:
                    geom_type = feature['geometry']['type']
                    prop_type = feature['properties'].get('type', 'unknown')
                    key = f"{geom_type} ({prop_type})"
                    feature_types[key] = feature_types.get(key, 0) + 1
                
                for feature_type, count in feature_types.items():
                    self.get_logger().debug(f"    - {feature_type}: {count}")
        
        # Marker statistics
        waypoint_count, boundary_count, hazard_count = self.count_markers()
        self.get_logger().debug("Marker Statistics:")
        self.get_logger().debug(f"  • Total waypoints: {waypoint_count}")
        self.get_logger().debug(f"  • Total boundaries: {boundary_count}")
        self.get_logger().debug(f"  • Total hazards: {hazard_count}")
        self.get_logger().debug(f"  • Total markers: {waypoint_count + boundary_count + hazard_count}")
        
        # Frame and coordinate information
        self.get_logger().debug("Coordinate System:")
        self.get_logger().debug(f"  • Frame ID: map")
        self.get_logger().debug(f"  • Coordinate format: Local x, y, z")
        self.get_logger().debug(f"  • Units: Meters")
        
        self.get_logger().debug("=" * 60)
        self.get_logger().debug("Sailing area publisher ready for Foxglove visualization")
        self.get_logger().debug("=" * 60)
    
    def _find_origin_from_maps(self):
        """Find the 'home' waypoint from loaded maps to use as coordinate origin.
        If map_name is specified, use that map's home waypoint; otherwise use the first one found."""
        # If a specific map is requested, search it first
        if self.map_name and self.map_name in self.sailing_areas:
            geojson_data = self.sailing_areas[self.map_name]
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                if props.get('name') == 'home' and props.get('type') == 'waypoint':
                    coords = feature['geometry']['coordinates']
                    # GeoJSON format: [longitude, latitude, altitude]
                    self.origin_lon = coords[0]
                    self.origin_lat = coords[1]
                    self.get_logger().info(f"Found origin from '{self.map_name}' map home waypoint: "
                                         f"lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}")
                    return
            self.get_logger().warn(f"Map '{self.map_name}' specified but no 'home' waypoint found in it")
        
        # Search all maps (prioritizing specified map if found above)
        for area_name, geojson_data in self.sailing_areas.items():
            # Skip if we already checked the specified map
            if self.map_name and area_name == self.map_name:
                continue
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                if props.get('name') == 'home' and props.get('type') == 'waypoint':
                    coords = feature['geometry']['coordinates']
                    # GeoJSON format: [longitude, latitude, altitude]
                    self.origin_lon = coords[0]
                    self.origin_lat = coords[1]
                    self.get_logger().info(f"Found origin from '{area_name}' map home waypoint: "
                                         f"lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}")
                    return
        self.get_logger().warn(f"No 'home' waypoint found in maps, using default origin: "
                              f"lat={self.origin_lat:.6f}, lon={self.origin_lon:.6f}")
    
    def lonlat_to_xy(self, lon, lat):
        """Converts longitude/latitude to local x/y meters using an equirectangular projection."""
        x = (math.radians(lon) - math.radians(self.origin_lon)) * self.earth_radius * math.cos(math.radians(self.origin_lat))
        y = (math.radians(lat) - math.radians(self.origin_lat)) * self.earth_radius
        return x, y
    
    def count_markers(self):
        """Count the number of markers by type"""
        waypoint_count = 0
        boundary_count = 0
        hazard_count = 0
        
        for area_name, geojson_data in self.sailing_areas.items():
            for feature in geojson_data.get('features', []):
                geom_type = feature['geometry']['type']
                feature_type = feature['properties'].get('type', 'unknown')
                
                if geom_type == 'Point':
                    waypoint_count += 1
                elif geom_type == 'LineString':
                    boundary_count += 1
                elif geom_type == 'Polygon':
                    if feature_type == 'hazard':
                        hazard_count += 1
                    else:
                        boundary_count += 1
        
        return waypoint_count, boundary_count, hazard_count
    
    def periodic_republish(self):
        """Periodic republish with logging"""
        self.republish_count += 1
        # Republish frequently for first 12 iterations (60 seconds), then less frequently
        if self.republish_count <= 12:
            # First minute: republish every 5 seconds
            self.publish_all_markers()
            if self.republish_count % 12 == 0:  # Every minute
                self.get_logger().info(f"Periodic republish of sailing area markers (minute {self.republish_count // 12})")
        else:
            # After first minute: republish every 60 seconds
            if self.republish_count % 12 == 0:  # Every 60 seconds (12 * 5s)
                self.get_logger().info(f"Periodic republish of sailing area markers (every 60 seconds)")
                self.publish_all_markers()
                # Reset timer to 60 seconds for future republishes
                self.timer.cancel()
                self.timer = self.create_timer(60.0, self.periodic_republish)
    
    def create_waypoint_marker(self, feature, marker_id):
        """Create a marker for a waypoint
        
        Special handling for 'home' waypoint: creates a tall pillar (4m high, 0.5m across and transparent)
        for better visibility. Other waypoints are shown as small spheres.
        """
        coords = feature['geometry']['coordinates']
        name = feature['properties']['name']
        
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_current_time()
        
        marker.id = marker_id
        marker.action = Marker.ADD
        
        # Position (convert from lon/lat to x/y/z)
        x, y = self.lonlat_to_xy(coords[0], coords[1])
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = coords[2] if len(coords) > 2 else 0.0  # altitude
        
        # Special handling for 'home' waypoint: create a tall pillar
        # This is the HOME LOCATION - the boat starts here and points toward geofence center
        if name == 'home':
            marker.type = Marker.CYLINDER
            # Scale: 0.5m diameter (x, y), 4m height (z)
            marker.scale.x = 0.5  # diameter in X direction (reduced from 1.0m for better visibility)
            marker.scale.y = 0.5  # diameter in Y direction (reduced from 1.0m for better visibility)
            marker.scale.z = 4.0  # height in Z direction
            # Position cylinder base at ground level (z position is center of cylinder)
            # So we need to offset by half the height to place base at ground
            marker.pose.position.z = (coords[2] if len(coords) > 2 else 0.0) + 2.0  # Center at 2m above ground
            # Color: green, semi-transparent (0.5 alpha) for better visibility
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.2)
        else:
            marker.type = Marker.SPHERE
            # Scale: small sphere for other waypoints
            marker.scale.x = 0.5  # 0.5m sphere
            marker.scale.y = 0.5
            marker.scale.z = 0.2
            # Color (green for waypoints, fully opaque)
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        # Text label
        marker.text = name
        
        # Lifetime - infinite so markers persist
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        return marker
    
    def create_line_marker(self, feature, marker_id):
        """Create a marker for a line/polygon boundary"""
        coords = feature['geometry']['coordinates']
        name = feature['properties']['name']
        feature_type = feature['properties']['type']
        
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_current_time()
        
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Add all points
        for coord in coords:
            x, y = self.lonlat_to_xy(coord[0], coord[1])
            point = Point()
            point.x = x
            point.y = y
            point.z = coord[2] if len(coord) > 2 else 0.0  # altitude
            marker.points.append(point)
        
        # Scale (line width) - make thicker for visibility  
        marker.scale.x = .2  # 1 meter thick line for better visibility
        marker.scale.y = 0.0  # Not used for LINE_STRIP
        marker.scale.z = 0.0  # Not used for LINE_STRIP
        
        # Lifetime - set to 0 (infinite) so markers persist
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Color based on type - fully opaque for better visibility
        if feature_type == "sailing_boundary":
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green, fully opaque
        elif feature_type == "sailing_area":
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue, fully opaque
        else:
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
        
        return marker
    
    def create_polygon_marker(self, feature, marker_id):
        """Create a marker for a polygon area"""
        coords = feature['geometry']['coordinates'][0]  # Get outer ring
        name = feature['properties']['name']
        feature_type = feature['properties']['type']
        
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_current_time()
        
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Add all points (including closing the polygon)
        for coord in coords:
            x, y = self.lonlat_to_xy(coord[0], coord[1])
            point = Point()
            point.x = x
            point.y = y
            point.z = coord[2] if len(coord) > 2 else 0.0  # altitude
            marker.points.append(point)
        
        # Close the polygon
        if coords[0] != coords[-1]:
            x, y = self.lonlat_to_xy(coords[0][0], coords[0][1])
            point = Point()
            point.x = x
            point.y = y
            point.z = coords[0][2] if len(coords[0]) > 2 else 0.0
            marker.points.append(point)
        
        # Scale (line width) - make thicker for visibility  
        marker.scale.x = 0.2  # 1 meter thick line for better visibility
        marker.scale.y = 0.0  # Not used for LINE_STRIP
        marker.scale.z = 0.0  # Not used for LINE_STRIP
        
        # Lifetime - set to 0 (infinite) so markers persist
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        
        # Color based on type
        if feature_type == "sailing_area":
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue, fully opaque
        elif feature_type == "hazard":
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red, fully opaque
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)  # Yellow, fully opaque
        
        return marker
    
    def publish_all_markers(self):
        """Publish all sailing area markers"""
        waypoint_markers = MarkerArray()
        boundary_markers = MarkerArray()
        hazard_markers = MarkerArray()
        
        marker_id = 0
        
        # If a specific map is requested, only publish markers from that map
        # This ensures coordinates use the correct origin
        maps_to_publish = []
        if self.map_name:
            # Try exact match first
            if self.map_name in self.sailing_areas:
                maps_to_publish = [self.map_name]
                self.get_logger().debug(f"✅ Publishing markers ONLY from '{self.map_name}' map (to ensure correct coordinate origin)")
            else:
                # Try case-insensitive match
                map_name_lower = self.map_name.lower()
                matched_map = None
                for loaded_map in self.sailing_areas.keys():
                    if loaded_map.lower() == map_name_lower:
                        matched_map = loaded_map
                        break
                
                if matched_map:
                    maps_to_publish = [matched_map]
                    self.get_logger().warn(f"⚠️ Map name case mismatch: requested '{self.map_name}', using '{matched_map}'")
                    self.get_logger().debug(f"✅ Publishing markers ONLY from '{matched_map}' map")
                else:
                    # No match found - fallback to all maps with warning
                    maps_to_publish = list(self.sailing_areas.keys())
                    self.get_logger().error(f"❌ ERROR: Requested map '{self.map_name}' not found in loaded maps!")
                    self.get_logger().error(f"   Available maps: {', '.join(self.sailing_areas.keys())}")
                    self.get_logger().error(f"   ⚠️ FALLING BACK: Publishing markers from ALL {len(maps_to_publish)} maps (WRONG GEOFENCE!)")
        else:
            maps_to_publish = list(self.sailing_areas.keys())
            self.get_logger().warn(f"⚠️ No map_name specified - publishing markers from all {len(maps_to_publish)} maps")
        
        for area_name in maps_to_publish:
            geojson_data = self.sailing_areas[area_name]
            self.get_logger().debug(f"Publishing markers for {area_name}")
            
            for feature in geojson_data.get('features', []):
                geom_type = feature['geometry']['type']
                feature_type = feature['properties']['type']
                
                if geom_type == 'Point':
                    marker = self.create_waypoint_marker(feature, marker_id)
                    waypoint_markers.markers.append(marker)
                    marker_id += 1
                
                elif geom_type == 'LineString':
                    marker = self.create_line_marker(feature, marker_id)
                    boundary_markers.markers.append(marker)
                    marker_id += 1
                
                elif geom_type == 'Polygon':
                    marker = self.create_polygon_marker(feature, marker_id)
                    if feature_type == 'hazard':
                        hazard_markers.markers.append(marker)
                    else:
                        boundary_markers.markers.append(marker)
                    marker_id += 1
        
        # Publish all marker arrays
        self.waypoint_pub.publish(waypoint_markers)
        self.boundary_pub.publish(boundary_markers)
        self.hazard_pub.publish(hazard_markers)
        
        total_markers = len(waypoint_markers.markers) + len(boundary_markers.markers) + len(hazard_markers.markers)
        if not self._logged_marker_publicaton:
            self.get_logger().info(f"Published sailing area markers: {len(waypoint_markers.markers)} waypoints, "
                                  f"{len(boundary_markers.markers)} boundaries, "
                                  f"{len(hazard_markers.markers)} hazards "
                                  f"(total: {total_markers} markers)")
            self._logged_marker_publicaton = True

def load_map_from_ros2_params():
    """Load map name from ROS2 parameters (argo.yaml)."""
    try:
        import rclpy
        from rclpy.node import Node
        
        # Initialize ROS2 if not already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create a temporary node to read parameters
        temp_node = Node('temp_map_loader')
        temp_node.declare_parameter('geofence_map_name', 'Argo Irchel pond sailing area')
        map_name = temp_node.get_parameter('geofence_map_name').get_parameter_value().string_value
        temp_node.destroy_node()
        
        return map_name if map_name else None
    except Exception as e:
        # Silently fail - map_name will be None and defaults will be used
        return None

def main(args=None):
    parser = argparse.ArgumentParser(
        description='Publish sailing area data from GeoJSON files as ROS2 markers for Foxglove visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 sailing_area_publisher.py                              # Uses map from argo_nodes.yaml
  python3 sailing_area_publisher.py --map "Argo Irchel pond sailing area"  # Override map
        """
    )
    parser.add_argument('--map', type=str, default=None,
                       help='Map name (without .geojson extension) - overrides map from argo_nodes.yaml')
    
    # Parse known args (ROS2 will handle the rest)
    parsed_args, remaining_args = parser.parse_known_args(args)
    
    # Load map name from ROS2 parameters if not provided via command line
    map_name = parsed_args.map
    if not map_name:
        map_name = load_map_from_ros2_params()
        if map_name:
            print(f"📍 Using map from ROS2 parameters (argo.yaml): '{map_name}'")
    
    # Initialize ROS2 if not already initialized (load_map_from_ros2_params may have initialized it)
    if not rclpy.ok():
        rclpy.init(args=remaining_args)
    elif remaining_args:
        # If ROS2 is already initialized but we have args, we can't pass them now
        # This is fine - args are typically handled during init
        pass
    
    publisher = SailingAreaPublisher(map_name=map_name)
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass  # Context already shutdown
    finally:
        publisher.destroy_node()
        # Only shutdown if context is still valid
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except:
                pass  # Already shutdown

if __name__ == '__main__':
    main()
