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

class SailingAreaPublisher(Node):
    def __init__(self):
        super().__init__('sailing_area_publisher')
        
        # Create publishers for different types of markers
        self.waypoint_pub = self.create_publisher(MarkerArray, '/sailing_waypoints', 10)
        self.boundary_pub = self.create_publisher(MarkerArray, '/sailing_boundaries', 10)
        self.hazard_pub = self.create_publisher(MarkerArray, '/sailing_hazards', 10)
        
        # Load sailing area data
        self.maps_dir = Path("/home/orangepi/argo/foxglove/maps")
        self.sailing_areas = self.load_sailing_areas()
        
        # Publish markers at startup
        self.publish_all_markers()
        
        # Create a timer to republish periodically (in case of reconnection)
        self.timer = self.create_timer(60.0, self.publish_all_markers)
        
        self.get_logger().info(f"Sailing area publisher started with {len(self.sailing_areas)} areas")
    
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
        marker.pose.position.x = coords[0]  # longitude
        marker.pose.position.y = coords[1]  # latitude  
        marker.pose.position.z = coords[2] if len(coords) > 2 else 0.0  # altitude
        
        # Scale
        marker.scale.x = 0.0001  # Small sphere
        marker.scale.y = 0.0001
        marker.scale.z = 0.0001
        
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
            point = Point()
            point.x = coord[0]  # longitude
            point.y = coord[1]  # latitude
            point.z = coord[2] if len(coord) > 2 else 0.0  # altitude
            marker.points.append(point)
        
        # Scale (line width)
        marker.scale.x = 0.00005  # Very thin line
        
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
            point = Point()
            point.x = coord[0]  # longitude
            point.y = coord[1]  # latitude
            point.z = coord[2] if len(coord) > 2 else 0.0  # altitude
            marker.points.append(point)
        
        # Close the polygon
        if coords[0] != coords[-1]:
            point = Point()
            point.x = coords[0][0]
            point.y = coords[0][1]
            point.z = coords[0][2] if len(coords[0]) > 2 else 0.0
            marker.points.append(point)
        
        # Scale (line width)
        marker.scale.x = 0.0001  # Thin line
        
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
            self.get_logger().info(f"Publishing markers for {area_name}")
            
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
        
        self.get_logger().info(f"Published {len(waypoint_markers.markers)} waypoints, "
                              f"{len(boundary_markers.markers)} boundaries, "
                              f"{len(hazard_markers.markers)} hazards")

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SailingAreaPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
