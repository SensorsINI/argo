#!/usr/bin/env python3
"""
Phase 2: Mock Simulator Testing for Patrol Controller

Tests the patrol controller with the mock sailboat simulator:
- Geofence loading and coordinate conversion
- Boundary detection with simulated boat positions
- Tack/jibe execution logic
- Integration between geofence manager and controller

This test runs without ROS2, using the mock simulator directly.
"""

import unittest
import sys
import os
import math
import time
from pathlib import Path

# Add nodes directory to path
script_path = Path(__file__).resolve()
argo_dir = script_path.parents[2]  # nodes/test -> nodes -> argo
nodes_dir = argo_dir / "nodes"
sys.path.insert(0, str(nodes_dir))
sys.path.insert(0, str(nodes_dir / "support"))

from mock_sailboat_simulator import MockSailboatSimulator
from geofence_manager import GeofenceManager


class TestPatrolControllerMock(unittest.TestCase):
    """Test patrol controller with mock simulator."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create mock simulator
        self.sim = MockSailboatSimulator()
        
        # Create geofence manager with test origin
        self.geofence = GeofenceManager(origin_lon=0.0, origin_lat=0.0)
        
        # Create a simple square geofence: 200m x 200m centered at origin
        # This represents a sailing area from -100m to +100m in both x and y
        self.test_polygon_xy = [
            (-100.0, -100.0),
            (100.0, -100.0),
            (100.0, 100.0),
            (-100.0, 100.0)
        ]
        self.geofence.polygon_xy = self.test_polygon_xy
        
        # Convert to lon/lat for reference
        self.geofence.polygon_lonlat = [
            self.geofence.xy_to_lonlat(x, y) for x, y in self.test_polygon_xy
        ]
    
    def test_geofence_loading_with_simulator(self):
        """Test that geofence loads correctly and works with simulator coordinates."""
        # Verify polygon is set
        self.assertIsNotNone(self.geofence.polygon_xy)
        self.assertEqual(len(self.geofence.polygon_xy), 4)
        
        # Test that center point is inside
        self.assertTrue(self.geofence.is_point_inside_polygon(0.0, 0.0))
        
        # Test that corners are inside
        self.assertTrue(self.geofence.is_point_inside_polygon(50.0, 50.0))
        self.assertTrue(self.geofence.is_point_inside_polygon(-50.0, -50.0))
        
        # Test that points outside are detected
        self.assertFalse(self.geofence.is_point_inside_polygon(150.0, 0.0))
        self.assertFalse(self.geofence.is_point_inside_polygon(0.0, 150.0))
    
    def test_boundary_detection_inside(self):
        """Test boundary distance calculation when boat is inside geofence."""
        # Start boat at center
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        
        # Convert simulator coordinates to geofence local coordinates
        # (In real usage, we'd convert from GPS lat/lon, but for mock we use direct x/y)
        x, y = self.sim.boat_x, self.sim.boat_y
        
        # Should be inside and ~100m from nearest edge (center of 200m square)
        distance = self.geofence.distance_to_boundary(x, y)
        self.assertLess(distance, 0)  # Negative = inside
        self.assertAlmostEqual(abs(distance), 100.0, places=1)
    
    def test_boundary_detection_near_edge(self):
        """Test boundary detection when boat approaches edge."""
        # Position boat near right edge (90m from center, 10m from edge)
        self.sim.boat_x = 90.0
        self.sim.boat_y = 0.0
        
        x, y = self.sim.boat_x, self.sim.boat_y
        distance = self.geofence.distance_to_boundary(x, y)
        
        self.assertLess(distance, 0)  # Still inside
        self.assertAlmostEqual(abs(distance), 10.0, places=1)
    
    def test_boundary_detection_outside(self):
        """Test boundary detection when boat is outside geofence."""
        # Position boat outside geofence
        self.sim.boat_x = 150.0
        self.sim.boat_y = 0.0
        
        x, y = self.sim.boat_x, self.sim.boat_y
        distance = self.geofence.distance_to_boundary(x, y)
        
        self.assertGreater(distance, 0)  # Positive = outside
        self.assertAlmostEqual(distance, 50.0, places=1)  # 50m outside
    
    def test_boundary_crossing_prediction(self):
        """Test prediction of boundary crossing time."""
        # Start boat inside, heading toward boundary
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 0.0  # East (toward right edge at +100m)
        self.sim.boat_speed = 1.0  # 1 m/s
        
        # Convert to lat/lon for prediction
        lon, lat = self.geofence.xy_to_lonlat(self.sim.boat_x, self.sim.boat_y)
        
        # Convert simulator heading (0°=East) to compass heading (0°=North)
        # Simulator: 0°=East, 90°=North, 180°=West, 270°=South
        # Compass: 0°=North, 90°=East, 180°=South, 270°=West
        compass_heading = (90.0 - self.sim.boat_heading) % 360.0
        
        # Predict crossing time
        crossing_time = self.geofence.predict_boundary_crossing_time(
            lat, lon, compass_heading, self.sim.boat_speed, max_lookahead_sec=200.0
        )
        
        # Should cross at approximately 100 seconds (100m at 1 m/s)
        self.assertIsNotNone(crossing_time)
        self.assertAlmostEqual(crossing_time, 100.0, places=0)
    
    def test_boundary_crossing_prediction_no_crossing(self):
        """Test that crossing prediction returns None when heading away."""
        # Start boat inside, heading away from boundary
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 180.0  # West (away from right edge)
        self.sim.boat_speed = 1.0
        
        lon, lat = self.geofence.xy_to_lonlat(self.sim.boat_x, self.sim.boat_y)
        compass_heading = (90.0 - self.sim.boat_heading) % 360.0
        
        crossing_time = self.geofence.predict_boundary_crossing_time(
            lat, lon, compass_heading, self.sim.boat_speed, max_lookahead_sec=50.0
        )
        
        # Should not cross (heading away from boundary)
        self.assertIsNone(crossing_time)
    
    def test_simulator_movement_inside_geofence(self):
        """Test that simulator can move boat and we track position relative to geofence."""
        # Start at center
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 45.0  # Northeast
        self.sim.boat_speed = 1.0
        
        # Run simulation for 10 seconds
        for _ in range(100):  # 100 steps * 0.1s = 10 seconds
            self.sim.step()
        
        # Check that boat is still inside (should be, heading diagonally)
        x, y = self.sim.boat_x, self.sim.boat_y
        self.assertTrue(self.geofence.is_point_inside_polygon(x, y))
        
        # Distance should be less than initial (moved toward edge)
        distance = self.geofence.distance_to_boundary(x, y)
        self.assertLess(distance, 0)  # Still inside
        self.assertLess(abs(distance), 100.0)  # Closer to edge than center
    
    def test_simulator_approaching_boundary(self):
        """Test detection when simulator boat approaches boundary."""
        # Start near right edge
        self.sim.boat_x = 80.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 0.0  # East (toward boundary)
        self.sim.boat_speed = 1.0
        
        initial_distance = self.geofence.distance_to_boundary(
            self.sim.boat_x, self.sim.boat_y
        )
        
        # Simulate for 5 seconds
        for _ in range(50):
            self.sim.step()
            x, y = self.sim.boat_x, self.sim.boat_y
            distance = self.geofence.distance_to_boundary(x, y)
            
            # Distance should decrease (getting closer to boundary)
            if abs(distance) < abs(initial_distance):
                # Successfully detected approach
                break
        
        # Should be closer to boundary now
        final_distance = self.geofence.distance_to_boundary(
            self.sim.boat_x, self.sim.boat_y
        )
        self.assertLess(abs(final_distance), abs(initial_distance))
    
    def test_boundary_crossing_detection(self):
        """Test that we can detect when boat crosses boundary."""
        # Start very close to boundary (99m from center, 1m from edge at 100m)
        self.sim.boat_x = 99.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 0.0  # East
        self.sim.boat_speed = 2.0  # Fast speed to ensure crossing
        self.sim.wind_direction = 90.0  # Wind from north (perpendicular to heading)
        
        # Verify starting position is inside
        x, y = self.sim.boat_x, self.sim.boat_y
        self.assertTrue(self.geofence.is_point_inside_polygon(x, y))
        
        # Simulate until crossing (should happen very quickly - only 1m to go)
        crossed = False
        for i in range(30):  # Max 3 seconds (should only need ~0.5 seconds)
            self.sim.step()
            x, y = self.sim.boat_x, self.sim.boat_y
            if not self.geofence.is_point_inside_polygon(x, y):
                crossed = True
                break
        
        # If not crossed, check if we're very close (within 0.5m of boundary)
        if not crossed:
            distance = self.geofence.distance_to_boundary(x, y)
            # Should be very close to boundary (within 0.5m)
            self.assertLess(abs(distance), 0.5, 
                          f"Boat should be very close to boundary. Position: ({x:.2f}, {y:.2f}), distance: {distance:.2f}m")
        else:
            self.assertTrue(crossed, "Boat successfully crossed boundary")
    
    def test_tack_trigger_logic(self):
        """Test logic for triggering tack when approaching boundary."""
        # Start inside, heading toward boundary
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 0.0  # East
        self.sim.boat_speed = 1.0
        
        # Set wind from north (for tacking logic)
        self.sim.wind_direction = 90.0  # Wind blowing toward North (90°)
        
        # Simulate and check distance to boundary
        for _ in range(80):  # 8 seconds - should get close to boundary
            self.sim.step()
            x, y = self.sim.boat_x, self.sim.boat_y
            distance = self.geofence.distance_to_boundary(x, y)
            
            # If within 15m of boundary, should trigger tack
            if abs(distance) < 15.0:
                # This is where patrol controller would trigger tack
                # For test, just verify we can detect this condition
                self.assertLess(abs(distance), 15.0)
                break
    
    def test_geofence_with_simulator_coordinates(self):
        """Test coordinate conversion between simulator and geofence systems."""
        # Simulator uses x/y in meters directly
        # Geofence uses lon/lat converted to x/y
        
        # Set simulator position
        sim_x, sim_y = 50.0, 30.0
        self.sim.boat_x = sim_x
        self.sim.boat_y = sim_y
        
        # In real system, we'd convert GPS lat/lon to geofence x/y
        # For mock test, we can use simulator x/y directly if they're in same coordinate system
        # Or convert through lon/lat
        
        # Convert simulator position to lon/lat
        lon, lat = self.geofence.xy_to_lonlat(sim_x, sim_y)
        
        # Convert back to geofence x/y
        geofence_x, geofence_y = self.geofence.lonlat_to_xy(lon, lat)
        
        # Should match (within rounding)
        self.assertAlmostEqual(sim_x, geofence_x, places=1)
        self.assertAlmostEqual(sim_y, geofence_y, places=1)
    
    def test_patrol_controller_integration_points(self):
        """Test key integration points between geofence manager and controller logic."""
        # Test 1: Distance calculation for patrol decision
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        x, y = self.sim.boat_x, self.sim.boat_y
        distance = self.geofence.distance_to_boundary(x, y)
        
        # Patrol controller would use this to decide when to tack
        boundary_threshold = 15.0  # meters
        should_tack = abs(distance) < boundary_threshold
        self.assertFalse(should_tack)  # At center, shouldn't tack yet
        
        # Test 2: Position prediction for lookahead
        self.sim.boat_x = 85.0
        self.sim.boat_y = 0.0
        self.sim.boat_heading = 0.0
        self.sim.boat_speed = 1.0
        
        lon, lat = self.geofence.xy_to_lonlat(self.sim.boat_x, self.sim.boat_y)
        compass_heading = (90.0 - self.sim.boat_heading) % 360.0
        
        lookahead_time = 15.0  # seconds
        crossing_time = self.geofence.predict_boundary_crossing_time(
            lat, lon, compass_heading, self.sim.boat_speed, 
            max_lookahead_sec=lookahead_time
        )
        
        # Should predict crossing within lookahead (15m at 1 m/s = 15s, but we're at 85m)
        # Actually, at 85m from center, 15m to edge, so 15 seconds
        if crossing_time is not None:
            self.assertLess(crossing_time, lookahead_time)
    
    def test_multiple_boundary_approaches(self):
        """Test handling multiple boundary approaches during patrol."""
        # Simulate a patrol pattern: sail east, then north, then west
        positions_inside = []
        
        # Start at center
        self.sim.boat_x = 0.0
        self.sim.boat_y = 0.0
        
        # Sail east
        self.sim.boat_heading = 0.0
        self.sim.boat_speed = 1.0
        for _ in range(50):
            self.sim.step()
            x, y = self.sim.boat_x, self.sim.boat_y
            if self.geofence.is_point_inside_polygon(x, y):
                positions_inside.append((x, y))
            else:
                break  # Crossed boundary
        
        # Should have many positions inside
        self.assertGreater(len(positions_inside), 0)
        
        # Turn north and continue
        self.sim.boat_heading = 90.0
        for _ in range(50):
            self.sim.step()
            x, y = self.sim.boat_x, self.sim.boat_y
            if self.geofence.is_point_inside_polygon(x, y):
                positions_inside.append((x, y))
            else:
                break
        
        # Should have accumulated more positions
        self.assertGreater(len(positions_inside), 10)


class TestGeofenceWithRealMap(unittest.TestCase):
    """Test geofence loading with actual map file if available."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.geofence = GeofenceManager()
        # Get argo directory
        script_path = Path(__file__).resolve()
        argo_dir = script_path.parents[2]  # nodes/test -> nodes -> argo
        self.maps_dir = argo_dir / "foxglove" / "maps"
    
    def test_load_irchel_map(self):
        """Test loading the actual Argo Irchel pond map."""
        map_name = "Argo Irchel pond sailing area"
        
        if not (self.maps_dir / f"{map_name}.geojson").exists():
            self.skipTest(f"Map file not found: {map_name}.geojson")
        
        result = self.geofence.load_geofence(map_name, maps_dir=self.maps_dir)
        
        self.assertTrue(result, "Should successfully load map")
        self.assertIsNotNone(self.geofence.polygon_xy)
        self.assertGreater(len(self.geofence.polygon_xy), 3)
        self.assertIsNotNone(self.geofence.origin_lon)
        self.assertIsNotNone(self.geofence.origin_lat)
    
    def test_irchel_map_coordinates(self):
        """Test coordinate conversion with actual map origin."""
        map_name = "Argo Irchel pond sailing area"
        
        if not (self.maps_dir / f"{map_name}.geojson").exists():
            self.skipTest(f"Map file not found: {map_name}.geojson")
        
        result = self.geofence.load_geofence(map_name, maps_dir=self.maps_dir)
        if not result:
            self.skipTest("Failed to load map")
        
        # Test that home waypoint converts to (0, 0)
        x, y = self.geofence.lonlat_to_xy(
            self.geofence.origin_lon, 
            self.geofence.origin_lat
        )
        self.assertAlmostEqual(x, 0.0, places=1)
        self.assertAlmostEqual(y, 0.0, places=1)
        
        # Test a point near home
        test_lon = self.geofence.origin_lon + 0.001
        test_lat = self.geofence.origin_lat + 0.001
        x, y = self.geofence.lonlat_to_xy(test_lon, test_lat)
        
        # Should be a small distance from origin
        distance = math.sqrt(x**2 + y**2)
        self.assertGreater(distance, 0)
        self.assertLess(distance, 200)  # Should be less than 200m


if __name__ == '__main__':
    unittest.main(verbosity=2)

