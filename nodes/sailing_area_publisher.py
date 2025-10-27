#!/usr/bin/env python3
"""
Publish sailing area data from GeoJSON files as ROS2 markers for Foxglove visualization
"""

import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
import os
from pathlib import Path
import math

class SailingAreaPublisher(Node):
    def __init__(self):
        super().__init__('sailing_area_publisher')
        
        # Create publishers for different types of markers
        self.waypoint_pub = self.create_publisher(MarkerArray, '/sailing_waypoints', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/sailing_boundaries', 10)
        self.hazard_pub = self.create_publisher(MarkerArray, '/sailing_hazards', 10)
        
        # Define origin for coordinate conversion (from 'home' waypoint in GeoJSON)
        self.origin_lon = 8.5448386
        self.origin_lat = 47.3981555
        self.earth_radius = 6378137.0  # meters
        
        # Load sailing area data
        self.maps_dir = Path("/home/orangepi/argo/foxglove/maps")
        self.sailing_areas = self.load_sailing_areas()
        
        # Publish markers at startup
        self.publish_all_markers()
        
        # Create a timer to republish periodically (in case of reconnection)
        self.timer = self.create_timer(60.0, self.periodic_republish)
        
        # Log comprehensive startup information
        self.log_startup_info()
    
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
                    sailing_areas[geojson_file.stem] = data
                    self.get_logger().info(f"Loaded sailing area: {geojson_file.stem}")
            except Exception as e:
                self.get_logger().error(f"Failed to load {geojson_file}: {e}")
        
        return sailing_areas
    
    def log_startup_info(self):
        """Log comprehensive startup information about the sailing area publisher"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("SAILING AREA PUBLISHER STARTUP INFORMATION")
        self.get_logger().info("=" * 60)
        
        # Published topics information
        self.get_logger().info("Published Topics:")
        self.get_logger().info(f"  • /sailing_waypoints (visualization_msgs/MarkerArray) - Waypoint markers")
        self.get_logger().info(f"  • /sailing_boundaries (visualization_msgs/MarkerArray) - Boundary markers")
        self.get_logger().info(f"  • /sailing_hazards (visualization_msgs/MarkerArray) - Hazard markers")
        
        # Publishing rate information
        self.get_logger().info("Publishing Configuration:")
        self.get_logger().info(f"  • Initial publish: Immediate on startup")
        self.get_logger().info(f"  • Periodic republish: Every 60.0 seconds")
        self.get_logger().info(f"  • QoS depth: 10 messages")
        
        # Maps directory information
        self.get_logger().info("Maps Configuration:")
        self.get_logger().info(f"  • Maps directory: {self.maps_dir}")
        self.get_logger().info(f"  • Directory exists: {self.maps_dir.exists()}")
        
        # Loaded sailing areas information
        self.get_logger().info("Loaded Sailing Areas:")
        if not self.sailing_areas:
            self.get_logger().warn("  • No sailing areas loaded")
        else:
            for area_name, geojson_data in self.sailing_areas.items():
                features = geojson_data.get('features', [])
                self.get_logger().info(f"  • {area_name}: {len(features)} features")
                
                # Count feature types
                feature_types = {}
                for feature in features:
                    geom_type = feature['geometry']['type']
                    prop_type = feature['properties'].get('type', 'unknown')
                    key = f"{geom_type} ({prop_type})"
                    feature_types[key] = feature_types.get(key, 0) + 1
                
                for feature_type, count in feature_types.items():
                    self.get_logger().info(f"    - {feature_type}: {count}")
        
        # Marker statistics
        waypoint_count, boundary_count, hazard_count = self.count_markers()
        self.get_logger().info("Marker Statistics:")
        self.get_logger().info(f"  • Total waypoints: {waypoint_count}")
        self.get_logger().info(f"  • Total boundaries: {boundary_count}")
        self.get_logger().info(f"  • Total hazards: {hazard_count}")
        self.get_logger().info(f"  • Total markers: {waypoint_count + boundary_count + hazard_count}")
        
        # Frame and coordinate information
        self.get_logger().info("Coordinate System:")
        self.get_logger().info(f"  • Frame ID: map")
        self.get_logger().info(f"  • Coordinate format: Local x, y, z")
        self.get_logger().info(f"  • Units: Meters")
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Sailing area publisher ready for Foxglove visualization")
        self.get_logger().info("=" * 60)
    
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
        self.get_logger().info("Periodic republish of sailing area markers (every 60 seconds)")
        self.publish_all_markers()
    
    def create_waypoint_marker(self, feature, marker_id):
        """Create a marker for a waypoint"""
        coords = feature['geometry']['coordinates']
        name = feature['properties']['name']
        
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position (convert from lon/lat to x/y/z)
        x, y = self.lonlat_to_xy(coords[0], coords[1])
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = coords[2] if len(coords) > 2 else 0.0  # altitude
        
        # Scale
        marker.scale.x = 1.0  # 1-meter sphere
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Color (green for waypoints)
        marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        
        # Text label
        marker.text = name
        
        return marker
    
    def create_line_marker(self, feature, marker_id):
        """Create a marker for a line/polygon boundary"""
        coords = feature['geometry']['coordinates']
        name = feature['properties']['name']
        feature_type = feature['properties']['type']
        
        marker = Marker()
        marker.header = Header()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        
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
        
        # Scale (line width)
        marker.scale.x = 0.2  # 20cm thick line
        
        # Color based on type
        if feature_type == "sailing_boundary":
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green
        elif feature_type == "sailing_area":
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue
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
        marker.header.stamp = self.get_clock().now().to_msg()
        
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
        
        # Scale (line width)
        marker.scale.x = 0.2  # 20cm thick line
        
        # Color based on type
        if feature_type == "sailing_area":
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue
        elif feature_type == "hazard":
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Red
        else:
            marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)  # Yellow
        
        return marker
    
    def publish_all_markers(self):
        """Publish all sailing area markers"""
        waypoint_markers = MarkerArray()
        boundary_markers = MarkerArray()
        hazard_markers = MarkerArray()
        
        marker_id = 0
        
        for area_name, geojson_data in self.sailing_areas.items():
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
        self.get_logger().info(f"Published sailing area markers: {len(waypoint_markers.markers)} waypoints, "
                              f"{len(boundary_markers.markers)} boundaries, "
                              f"{len(hazard_markers.markers)} hazards "
                              f"(total: {total_markers} markers)")

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SailingAreaPublisher()
    
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
