#!/usr/bin/env python3
"""
Unit tests for GeofenceManager module

Tests all major functionality:
- Coordinate conversion (lon/lat ↔ x/y)
- Point-in-polygon algorithm
- Distance to boundary calculations
- Position prediction
- Boundary crossing time prediction
- GeoJSON loading
"""

import unittest
import math
import json
import tempfile
from pathlib import Path
import sys
import os

# Add nodes/support to path to import geofence_manager
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'support'))
from geofence_manager import GeofenceManager


class TestCoordinateConversion(unittest.TestCase):
    """Test coordinate conversion between lon/lat and x/y meters."""
    
    def setUp(self):
        """Set up test fixtures with known origin."""
        # Use actual origin from Argo Irchel pond map
        self.origin_lon = 8.5448386
        self.origin_lat = 47.3981555
        self.manager = GeofenceManager(origin_lon=self.origin_lon, origin_lat=self.origin_lat)
    
    def test_origin_conversion(self):
        """Test that origin converts to (0, 0) in local coordinates."""
        x, y = self.manager.lonlat_to_xy(self.origin_lon, self.origin_lat)
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
    
    def test_round_trip_accuracy(self):
        """Test round-trip conversion accuracy."""
        test_cases = [
            (8.5448386, 47.3981555),  # Origin
            (8.545, 47.398),  # Nearby point
            (8.544, 47.397),  # Another nearby point
            (8.5448386 + 0.001, 47.3981555 + 0.001),  # Small offset
        ]
        
        for lon, lat in test_cases:
            with self.subTest(lon=lon, lat=lat):
                # Convert to x/y
                x, y = self.manager.lonlat_to_xy(lon, lat)
                
                # Convert back to lon/lat
                lon2, lat2 = self.manager.xy_to_lonlat(x, y)
                
                # Should be very close to original (within 1e-6 degrees ≈ 0.1m)
                self.assertAlmostEqual(lon, lon2, places=6)
                self.assertAlmostEqual(lat, lat2, places=6)
    
    def test_known_conversion(self):
        """Test conversion of a known point."""
        # Point 100m east and 100m north of origin
        x, y = 100.0, 100.0
        lon, lat = self.manager.xy_to_lonlat(x, y)
        
        # Convert back and verify
        x2, y2 = self.manager.lonlat_to_xy(lon, lat)
        self.assertAlmostEqual(x, x2, places=1)  # Within 0.1m
        self.assertAlmostEqual(y, y2, places=1)
    
    def test_error_no_origin(self):
        """Test that conversion fails when origin not set."""
        manager = GeofenceManager()  # No origin set
        with self.assertRaises(ValueError):
            manager.lonlat_to_xy(8.5, 47.4)
        with self.assertRaises(ValueError):
            manager.xy_to_lonlat(100.0, 100.0)
    
    def test_different_latitudes(self):
        """Test conversion at different latitudes."""
        # Test at equator
        manager_eq = GeofenceManager(origin_lon=0.0, origin_lat=0.0)
        x, y = manager_eq.lonlat_to_xy(0.001, 0.001)
        self.assertIsInstance(x, float)
        self.assertIsInstance(y, float)
        
        # Test at mid-latitude (Argo location)
        x, y = self.manager.lonlat_to_xy(self.origin_lon + 0.001, self.origin_lat + 0.001)
        self.assertIsInstance(x, float)
        self.assertIsInstance(y, float)


class TestPointInPolygon(unittest.TestCase):
    """Test point-in-polygon algorithm."""
    
    def setUp(self):
        """Set up test fixtures with simple polygons."""
        self.manager = GeofenceManager(origin_lon=0.0, origin_lat=0.0)
        
        # Simple square polygon: 100m x 100m centered at origin
        self.square_polygon = [
            (-50.0, -50.0),
            (50.0, -50.0),
            (50.0, 50.0),
            (-50.0, 50.0)
        ]
        self.manager.polygon_xy = self.square_polygon
    
    def test_point_inside_square(self):
        """Test points clearly inside the square."""
        inside_points = [
            (0.0, 0.0),      # Center
            (10.0, 10.0),    # Inside
            (-10.0, -10.0), # Inside
            (40.0, 40.0),    # Near edge but inside
        ]
        
        for x, y in inside_points:
            with self.subTest(x=x, y=y):
                self.assertTrue(self.manager.is_point_inside_polygon(x, y))
    
    def test_point_outside_square(self):
        """Test points clearly outside the square."""
        outside_points = [
            (100.0, 100.0),   # Far outside
            (-100.0, -100.0), # Far outside
            (60.0, 0.0),      # Outside to the right
            (0.0, 60.0),      # Outside above
        ]
        
        for x, y in outside_points:
            with self.subTest(x=x, y=y):
                self.assertFalse(self.manager.is_point_inside_polygon(x, y))
    
    def test_point_on_edge(self):
        """Test points on polygon edges."""
        # Points exactly on edges (may be inside or outside depending on algorithm)
        edge_points = [
            (50.0, 0.0),   # Right edge
            (0.0, 50.0),   # Top edge
            (-50.0, 0.0),  # Left edge
            (0.0, -50.0),  # Bottom edge
        ]
        
        # Ray casting typically treats edge points as inside
        for x, y in edge_points:
            with self.subTest(x=x, y=y):
                result = self.manager.is_point_inside_polygon(x, y)
                # Should be consistent (either all inside or all outside)
                self.assertIsInstance(result, bool)
    
    def test_point_at_vertex(self):
        """Test points at polygon vertices."""
        vertex_points = [
            (-50.0, -50.0),  # Bottom-left
            (50.0, -50.0),   # Bottom-right
            (50.0, 50.0),    # Top-right
            (-50.0, 50.0),   # Top-left
        ]
        
        for x, y in vertex_points:
            with self.subTest(x=x, y=y):
                result = self.manager.is_point_inside_polygon(x, y)
                self.assertIsInstance(result, bool)
    
    def test_complex_polygon(self):
        """Test with a more complex polygon (L-shape)."""
        l_shape = [
            (0.0, 0.0),
            (50.0, 0.0),
            (50.0, 25.0),
            (25.0, 25.0),
            (25.0, 50.0),
            (0.0, 50.0)
        ]
        self.manager.polygon_xy = l_shape
        
        # Point in main rectangle
        self.assertTrue(self.manager.is_point_inside_polygon(10.0, 10.0))
        
        # Point in L extension
        self.assertTrue(self.manager.is_point_inside_polygon(10.0, 40.0))
        
        # Point in the "hole" of the L
        self.assertFalse(self.manager.is_point_inside_polygon(30.0, 30.0))
        
        # Point outside
        self.assertFalse(self.manager.is_point_inside_polygon(60.0, 60.0))
    
    def test_empty_polygon(self):
        """Test with empty or invalid polygon."""
        self.manager.polygon_xy = None
        self.assertFalse(self.manager.is_point_inside_polygon(0.0, 0.0))
        
        self.manager.polygon_xy = []
        self.assertFalse(self.manager.is_point_inside_polygon(0.0, 0.0))
        
        self.manager.polygon_xy = [(0.0, 0.0)]  # Single point
        self.assertFalse(self.manager.is_point_inside_polygon(0.0, 0.0))
        
        self.manager.polygon_xy = [(0.0, 0.0), (10.0, 10.0)]  # Line (2 points)
        self.assertFalse(self.manager.is_point_inside_polygon(0.0, 0.0))
    
    def test_lonlat_version(self):
        """Test is_point_inside_polygon_lonlat method."""
        # Set up a simple polygon with known origin
        manager = GeofenceManager(origin_lon=8.5448386, origin_lat=47.3981555)
        manager.polygon_xy = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]
        
        # Convert origin to x/y, then back to lon/lat for a point near origin
        x, y = 0.0, 0.0
        lon, lat = manager.xy_to_lonlat(x, y)
        
        # Should be inside
        self.assertTrue(manager.is_point_inside_polygon_lonlat(lon, lat))


class TestDistanceToBoundary(unittest.TestCase):
    """Test distance to boundary calculations."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.manager = GeofenceManager(origin_lon=0.0, origin_lat=0.0)
        
        # Square polygon: 100m x 100m centered at origin
        self.square_polygon = [
            (-50.0, -50.0),
            (50.0, -50.0),
            (50.0, 50.0),
            (-50.0, 50.0)
        ]
        self.manager.polygon_xy = self.square_polygon
    
    def test_distance_inside(self):
        """Test distance calculation for points inside polygon (should be negative)."""
        inside_points = [
            (0.0, 0.0, 50.0),      # Center: 50m to nearest edge
            (10.0, 10.0, 40.0),    # Inside: ~40m to nearest edge
            (40.0, 0.0, 10.0),     # Near right edge: 10m
        ]
        
        for x, y, expected_dist in inside_points:
            with self.subTest(x=x, y=y):
                dist = self.manager.distance_to_boundary(x, y)
                self.assertLess(dist, 0, "Distance should be negative for inside points")
                self.assertAlmostEqual(abs(dist), expected_dist, places=1)
    
    def test_distance_outside(self):
        """Test distance calculation for points outside polygon (should be positive)."""
        outside_points = [
            (100.0, 0.0, 50.0),    # 50m outside right edge
            (0.0, 100.0, 50.0),    # 50m outside top edge
            (60.0, 60.0, math.sqrt(10**2 + 10**2)),  # Diagonal outside
        ]
        
        for x, y, expected_dist in outside_points:
            with self.subTest(x=x, y=y):
                dist = self.manager.distance_to_boundary(x, y)
                self.assertGreater(dist, 0, "Distance should be positive for outside points")
                self.assertAlmostEqual(dist, expected_dist, places=1)
    
    def test_distance_on_edge(self):
        """Test distance for points on or very near edges."""
        # Point exactly on edge
        dist = self.manager.distance_to_boundary(50.0, 0.0)
        self.assertAlmostEqual(abs(dist), 0.0, places=1)
        
        # Point very close to edge
        dist = self.manager.distance_to_boundary(49.9, 0.0)
        self.assertLess(abs(dist), 1.0)  # Should be very small
    
    def test_distance_at_vertex(self):
        """Test distance calculation at polygon vertices."""
        # At vertex, distance should be 0
        dist = self.manager.distance_to_boundary(50.0, 50.0)
        self.assertAlmostEqual(abs(dist), 0.0, places=1)
    
    def test_empty_polygon(self):
        """Test with empty polygon."""
        self.manager.polygon_xy = None
        dist = self.manager.distance_to_boundary(0.0, 0.0)
        self.assertEqual(dist, float('inf'))
        
        self.manager.polygon_xy = []
        dist = self.manager.distance_to_boundary(0.0, 0.0)
        self.assertEqual(dist, float('inf'))
    
    def test_lonlat_version(self):
        """Test distance_to_boundary_lonlat method."""
        manager = GeofenceManager(origin_lon=8.5448386, origin_lat=47.3981555)
        manager.polygon_xy = [(-50.0, -50.0), (50.0, -50.0), (50.0, 50.0), (-50.0, 50.0)]
        
        # Point at origin (should be inside, ~50m from edge)
        lon, lat = 8.5448386, 47.3981555
        dist = manager.distance_to_boundary_lonlat(lon, lat)
        self.assertLess(dist, 0)  # Negative = inside
        self.assertAlmostEqual(abs(dist), 50.0, places=1)


class TestPositionPrediction(unittest.TestCase):
    """Test position prediction functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.origin_lon = 8.5448386
        self.origin_lat = 47.3981555
        self.manager = GeofenceManager(origin_lon=self.origin_lon, origin_lat=self.origin_lat)
    
    def test_predict_north(self):
        """Test prediction heading North (0 degrees)."""
        speed_ms = 1.0  # 1 m/s
        time_sec = 10.0  # 10 seconds
        heading = 0.0  # North
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Should move north (latitude increases)
        self.assertGreater(pred_lat, self.origin_lat)
        # Longitude should be approximately unchanged
        self.assertAlmostEqual(pred_lon, self.origin_lon, places=6)
    
    def test_predict_east(self):
        """Test prediction heading East (90 degrees)."""
        speed_ms = 1.0
        time_sec = 10.0
        heading = 90.0  # East
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Latitude should be approximately unchanged
        self.assertAlmostEqual(pred_lat, self.origin_lat, places=6)
        # Should move east (longitude increases)
        self.assertGreater(pred_lon, self.origin_lon)
    
    def test_predict_south(self):
        """Test prediction heading South (180 degrees)."""
        speed_ms = 1.0
        time_sec = 10.0
        heading = 180.0  # South
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Should move south (latitude decreases)
        self.assertLess(pred_lat, self.origin_lat)
        # Longitude should be approximately unchanged
        self.assertAlmostEqual(pred_lon, self.origin_lon, places=6)
    
    def test_predict_west(self):
        """Test prediction heading West (270 degrees)."""
        speed_ms = 1.0
        time_sec = 10.0
        heading = 270.0  # West
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Latitude should be approximately unchanged
        self.assertAlmostEqual(pred_lat, self.origin_lat, places=6)
        # Should move west (longitude decreases)
        self.assertLess(pred_lon, self.origin_lon)
    
    def test_predict_diagonal(self):
        """Test prediction at diagonal heading (45 degrees - Northeast)."""
        speed_ms = 1.0
        time_sec = 10.0
        heading = 45.0  # Northeast
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Should move both north and east
        self.assertGreater(pred_lat, self.origin_lat)
        self.assertGreater(pred_lon, self.origin_lon)
    
    def test_predict_distance_accuracy(self):
        """Test that predicted distance matches expected distance."""
        speed_ms = 2.0  # 2 m/s
        time_sec = 5.0  # 5 seconds
        expected_distance = speed_ms * time_sec  # 10 meters
        heading = 0.0  # North
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Calculate actual distance using Haversine (approximate)
        # For small distances, simple lat difference is close
        lat_diff_rad = math.radians(pred_lat - self.origin_lat)
        distance_approx = lat_diff_rad * self.manager.earth_radius
        
        # Should be close to expected distance (within 1m for short distances)
        self.assertAlmostEqual(distance_approx, expected_distance, places=0)
    
    def test_predict_zero_speed(self):
        """Test prediction with zero speed (should not move)."""
        speed_ms = 0.0
        time_sec = 10.0
        heading = 45.0
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Should not move
        self.assertAlmostEqual(pred_lat, self.origin_lat, places=8)
        self.assertAlmostEqual(pred_lon, self.origin_lon, places=8)
    
    def test_predict_zero_time(self):
        """Test prediction with zero time (should not move)."""
        speed_ms = 5.0
        time_sec = 0.0
        heading = 90.0
        
        pred_lat, pred_lon = self.manager.predict_future_position(
            self.origin_lat, self.origin_lon, heading, speed_ms, time_sec
        )
        
        # Should not move
        self.assertAlmostEqual(pred_lat, self.origin_lat, places=8)
        self.assertAlmostEqual(pred_lon, self.origin_lon, places=8)


class TestBoundaryCrossingTime(unittest.TestCase):
    """Test boundary crossing time prediction."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.manager = GeofenceManager(origin_lon=0.0, origin_lat=0.0)
        
        # Square polygon: 100m x 100m centered at origin
        self.square_polygon = [
            (-50.0, -50.0),
            (50.0, -50.0),
            (50.0, 50.0),
            (-50.0, 50.0)
        ]
        self.manager.polygon_xy = self.square_polygon
    
    def test_crossing_from_inside(self):
        """Test crossing prediction when boat is inside and heading toward boundary."""
        # Start at center (0, 0), heading east (90°) toward right edge at 50m
        lat, lon = 0.0, 0.0
        heading = 90.0  # East
        speed_ms = 1.0  # 1 m/s
        
        # Convert to lat/lon for prediction
        x, y = 0.0, 0.0  # Center of square
        lon, lat = self.manager.xy_to_lonlat(x, y)
        
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=100.0
        )
        
        # Should cross at approximately 50 seconds (50m at 1 m/s)
        self.assertIsNotNone(crossing_time)
        self.assertAlmostEqual(crossing_time, 50.0, places=0)
    
    def test_crossing_from_outside(self):
        """Test crossing prediction when boat is outside and heading toward boundary."""
        # Start outside (100m east), heading west (270°) toward boundary
        x, y = 100.0, 0.0  # Outside to the right
        lon, lat = self.manager.xy_to_lonlat(x, y)
        heading = 270.0  # West
        speed_ms = 1.0
        
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=100.0
        )
        
        # Should cross at approximately 50 seconds (50m at 1 m/s)
        self.assertIsNotNone(crossing_time)
        self.assertAlmostEqual(crossing_time, 50.0, places=0)
    
    def test_no_crossing_heading_away(self):
        """Test when boat is heading away from boundary (should return None)."""
        # Start at center, heading away from boundary
        x, y = 0.0, 0.0
        lon, lat = self.manager.xy_to_lonlat(x, y)
        heading = 0.0  # North (away from all boundaries if centered)
        speed_ms = 1.0
        
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=30.0  # Short lookahead
        )
        
        # Should not cross within lookahead time
        # (Actually, from center heading north, we will cross the top boundary)
        # Let's test with a point near bottom edge heading south
        x, y = 0.0, -40.0  # Near bottom edge
        lon, lat = self.manager.xy_to_lonlat(x, y)
        heading = 180.0  # South (away from boundary)
        
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=5.0  # Short lookahead
        )
        
        # Should not cross (heading away)
        self.assertIsNone(crossing_time)
    
    def test_zero_speed(self):
        """Test with zero speed (should return None)."""
        x, y = 0.0, 0.0
        lon, lat = self.manager.xy_to_lonlat(x, y)
        heading = 90.0
        speed_ms = 0.0
        
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=100.0
        )
        
        self.assertIsNone(crossing_time)
    
    def test_already_at_boundary(self):
        """Test when boat is already at or very close to boundary."""
        # Start just inside boundary (0.5m inside)
        x, y = 49.5, 0.0  # Just inside right edge
        lon, lat = self.manager.xy_to_lonlat(x, y)
        heading = 90.0  # Heading east (toward boundary)
        speed_ms = 1.0
        
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=10.0
        )
        
        # Should cross very soon (within 1 second at 1 m/s for 0.5m)
        self.assertIsNotNone(crossing_time)
        self.assertLess(crossing_time, 1.0)  # Should be very small
        self.assertGreater(crossing_time, 0.0)  # Should be positive
    
    def test_max_lookahead_limit(self):
        """Test that crossing beyond max_lookahead returns None."""
        # Start far from boundary, heading toward it slowly
        x, y = 0.0, 0.0
        lon, lat = self.manager.xy_to_lonlat(x, y)
        heading = 90.0  # East
        speed_ms = 0.1  # Very slow (0.1 m/s)
        
        # Short lookahead (5 seconds = 0.5m, won't reach boundary at 50m)
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, heading, speed_ms, max_lookahead_sec=5.0
        )
        
        # Should not cross within lookahead
        self.assertIsNone(crossing_time)


class TestGeofenceLoading(unittest.TestCase):
    """Test GeoJSON loading functionality."""
    
    def setUp(self):
        """Set up test fixtures with temporary GeoJSON files."""
        self.temp_dir = tempfile.mkdtemp()
        self.maps_dir = Path(self.temp_dir)
        self.manager = GeofenceManager()
    
    def tearDown(self):
        """Clean up temporary files."""
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)
    
    def create_test_geojson(self, filename, has_home=True, has_polygon=True):
        """Helper to create test GeoJSON file."""
        features = []
        
        if has_home:
            features.append({
                "type": "Feature",
                "properties": {
                    "name": "home",
                    "type": "waypoint"
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [8.5448386, 47.3981555, 0.0]
                }
            })
        
        if has_polygon:
            features.append({
                "type": "Feature",
                "properties": {
                    "name": "sailing_area",
                    "type": "sailing_area"
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                        [8.5448064, 47.3981645, 0.0],
                        [8.5446841, 47.3981151, 0.0],
                        [8.544535, 47.3980946, 0.0],
                        [8.5442261, 47.3977815, 0.0],
                        [8.5448064, 47.3981645, 0.0]  # Close polygon
                    ]]
                }
            })
        
        geojson = {
            "type": "FeatureCollection",
            "features": features
        }
        
        geojson_path = self.maps_dir / f"{filename}.geojson"
        with open(geojson_path, 'w') as f:
            json.dump(geojson, f)
        
        return geojson_path
    
    def test_load_valid_geojson(self):
        """Test loading a valid GeoJSON file."""
        self.create_test_geojson("test_map")
        
        result = self.manager.load_geofence("test_map", maps_dir=self.maps_dir)
        
        self.assertTrue(result)
        self.assertIsNotNone(self.manager.polygon_xy)
        self.assertIsNotNone(self.manager.polygon_lonlat)
        self.assertEqual(self.manager.origin_lon, 8.5448386)
        self.assertEqual(self.manager.origin_lat, 47.3981555)
    
    def test_load_missing_file(self):
        """Test loading a non-existent file."""
        result = self.manager.load_geofence("nonexistent_map", maps_dir=self.maps_dir)
        self.assertFalse(result)
    
    def test_load_without_home_waypoint(self):
        """Test loading GeoJSON without home waypoint (should use default)."""
        # Create GeoJSON without home waypoint
        geojson = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {
                    "name": "sailing_area",
                    "type": "sailing_area"
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [[
                        [8.5448064, 47.3981645, 0.0],
                        [8.5446841, 47.3981151, 0.0],
                        [8.544535, 47.3980946, 0.0],
                        [8.5448064, 47.3981645, 0.0]
                    ]]
                }
            }]
        }
        
        geojson_path = self.maps_dir / "test_map.geojson"
        with open(geojson_path, 'w') as f:
            json.dump(geojson, f)
        
        result = self.manager.load_geofence("test_map", maps_dir=self.maps_dir)
        
        # Should still load, but use default origin
        self.assertTrue(result)
        self.assertEqual(self.manager.origin_lon, 8.5448386)  # Default
        self.assertEqual(self.manager.origin_lat, 47.3981555)  # Default
    
    def test_load_linestring_boundary(self):
        """Test loading LineString boundary (fallback when no Polygon)."""
        geojson = {
            "type": "FeatureCollection",
            "features": [
                {
                    "type": "Feature",
                    "properties": {
                        "name": "home",
                        "type": "waypoint"
                    },
                    "geometry": {
                        "type": "Point",
                        "coordinates": [8.5448386, 47.3981555, 0.0]
                    }
                },
                {
                    "type": "Feature",
                    "properties": {
                        "name": "boundary",
                        "type": "sailing_boundary"
                    },
                    "geometry": {
                        "type": "LineString",
                        "coordinates": [
                            [8.5448064, 47.3981645, 0.0],
                            [8.5446841, 47.3981151, 0.0],
                            [8.544535, 47.3980946, 0.0]
                        ]
                    }
                }
            ]
        }
        
        geojson_path = self.maps_dir / "test_map.geojson"
        with open(geojson_path, 'w') as f:
            json.dump(geojson, f)
        
        result = self.manager.load_geofence("test_map", maps_dir=self.maps_dir)
        
        self.assertTrue(result)
        self.assertIsNotNone(self.manager.polygon_xy)
        self.assertIsNotNone(self.manager.polygon_lonlat)
    
    def test_load_invalid_json(self):
        """Test loading invalid JSON file."""
        invalid_path = self.maps_dir / "invalid.geojson"
        with open(invalid_path, 'w') as f:
            f.write("This is not valid JSON {")
        
        result = self.manager.load_geofence("invalid", maps_dir=self.maps_dir)
        self.assertFalse(result)
    
    def test_load_no_polygon(self):
        """Test loading GeoJSON with no polygon or boundary."""
        geojson = {
            "type": "FeatureCollection",
            "features": [{
                "type": "Feature",
                "properties": {
                    "name": "home",
                    "type": "waypoint"
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [8.5448386, 47.3981555, 0.0]
                }
            }]
        }
        
        geojson_path = self.maps_dir / "test_map.geojson"
        with open(geojson_path, 'w') as f:
            json.dump(geojson, f)
        
        result = self.manager.load_geofence("test_map", maps_dir=self.maps_dir)
        self.assertFalse(result)  # No polygon found


class TestIntegration(unittest.TestCase):
    """Integration tests combining multiple features."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.manager = GeofenceManager(origin_lon=0.0, origin_lat=0.0)
        
        # Square polygon
        self.manager.polygon_xy = [
            (-50.0, -50.0),
            (50.0, -50.0),
            (50.0, 50.0),
            (-50.0, 50.0)
        ]
    
    def test_full_workflow(self):
        """Test complete workflow: load, check position, predict crossing."""
        # Check point inside
        self.assertTrue(self.manager.is_point_inside_polygon(0.0, 0.0))
        
        # Get distance
        dist = self.manager.distance_to_boundary(0.0, 0.0)
        self.assertLess(dist, 0)  # Negative = inside
        self.assertAlmostEqual(abs(dist), 50.0, places=1)
        
        # Predict crossing time
        # Start at center, heading east at 1 m/s
        x, y = 0.0, 0.0
        lon, lat = self.manager.xy_to_lonlat(x, y)
        crossing_time = self.manager.predict_boundary_crossing_time(
            lat, lon, 90.0, 1.0, max_lookahead_sec=100.0
        )
        self.assertIsNotNone(crossing_time)
        self.assertAlmostEqual(crossing_time, 50.0, places=0)


if __name__ == '__main__':
    # Run tests with verbose output
    unittest.main(verbosity=2)

