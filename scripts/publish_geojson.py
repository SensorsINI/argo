#!/usr/bin/env python3
"""
Publish GeoJSON sailing boundaries as ROS2 visualization markers
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import math
from pathlib import Path


class GeoJSONPublisher(Node):
    def __init__(self):
        super().__init__('geojson_publisher')
        
        # Publishers
        self.boundaries_pub = self.create_publisher(MarkerArray, '/sailing_boundaries', 10)
        self.hazards_pub = self.create_publisher(MarkerArray, '/sailing_hazards', 10)
        self.waypoints_pub = self.create_publisher(MarkerArray, '/sailing_waypoints', 10)
        
        # Load and publish GeoJSON data
        self.load_and_publish_geojson()
        
        # Timer to republish periodically
        self.timer = self.create_timer(1.0, self.publish_markers)
        
        self.get_logger().info("GeoJSON publisher started")
    
    def load_and_publish_geojson(self):
        """Load GeoJSON file and convert to markers"""
        try:
            # Determine Argo repository directory dynamically
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[1]  # scripts -> argo
            geojson_path = argo_dir / "foxglove" / "maps" / "Argo Irchel pond sailing area.geojson"
            
            self.get_logger().info(f"Loading geojson from {geojson_path}")

            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            self.boundaries_markers = MarkerArray()
            self.hazards_markers = MarkerArray()
            self.waypoints_markers = MarkerArray()
            
            for feature in geojson_data['features']:
                if feature['properties']['type'] == 'sailing_area':
                    self.create_boundary_marker(feature)
                elif feature['properties']['type'] == 'hazard':
                    self.create_hazard_marker(feature)
                elif feature['properties']['type'] == 'waypoint':
                    self.create_waypoint_marker(feature)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to load GeoJSON: {e}")
    
    def create_boundary_marker(self, feature):
        """Create a line marker for sailing area boundary"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sailing_boundaries"
        marker.id = len(self.boundaries_markers.markers)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set color (blue)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        
        # Set scale
        marker.scale.x = 0.1  # Line width
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        
        # Convert coordinates to points
        coordinates = feature['geometry']['coordinates'][0]  # First ring of polygon
        for coord in coordinates:
            point = Point()
            # Convert lat/lon to local coordinates (simplified)
            # This is a rough conversion - you might need to adjust based on your coordinate system
            point.x = (coord[0] - 8.544) * 111320 * math.cos(math.radians(47.398))  # Longitude to meters
            point.y = (coord[1] - 47.398) * 111320  # Latitude to meters
            point.z = 0.0
            marker.points.append(point)
        
        self.boundaries_markers.markers.append(marker)
    
    def create_hazard_marker(self, feature):
        """Create a polygon marker for hazards"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sailing_hazards"
        marker.id = len(self.hazards_markers.markers)
        marker.type = Marker.LINE_STRIP  # Use LINE_STRIP instead of TRIANGLE_LIST
        marker.action = Marker.ADD
        
        # Set color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Set scale
        marker.scale.x = 0.2  # Line width
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        
        # Convert coordinates to points
        coordinates = feature['geometry']['coordinates'][0]
        for coord in coordinates:
            point = Point()
            point.x = (coord[0] - 8.544) * 111320 * math.cos(math.radians(47.398))
            point.y = (coord[1] - 47.398) * 111320
            point.z = 0.0
            marker.points.append(point)
        
        self.hazards_markers.markers.append(marker)
    
    def create_waypoint_marker(self, feature):
        """Create a sphere marker for waypoints"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sailing_waypoints"
        marker.id = len(self.waypoints_markers.markers)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Set color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Set scale
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        
        # Set position
        coord = feature['geometry']['coordinates']
        marker.pose.position.x = (coord[0] - 8.544) * 111320 * math.cos(math.radians(47.398))
        marker.pose.position.y = (coord[1] - 47.398) * 111320
        marker.pose.position.z = 0.0
        
        # Set orientation (identity quaternion)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        self.waypoints_markers.markers.append(marker)
    
    def publish_markers(self):
        """Publish all markers"""
        self.boundaries_pub.publish(self.boundaries_markers)
        self.hazards_pub.publish(self.hazards_markers)
        self.waypoints_pub.publish(self.waypoints_markers)

def main(args=None):
    rclpy.init(args=args)
    node = GeoJSONPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
