#!/usr/bin/env python3
"""
Geofence Manager - Utility module for geofence operations

Provides functions for:
- Loading GeoJSON sailing area polygons
- Point-in-polygon checks
- Distance to boundary calculations
- Future position prediction
- Coordinate conversions
"""

import json
import math
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any


class GeofenceManager:
    """Manages geofence polygons and provides geometric operations."""
    
    def __init__(self, origin_lon: float = None, origin_lat: float = None):
        """
        Initialize geofence manager.
        
        Args:
            origin_lon: Longitude of coordinate origin (home waypoint)
            origin_lat: Latitude of coordinate origin (home waypoint)
        """
        self.earth_radius = 6378137.0  # meters
        self.origin_lon = origin_lon
        self.origin_lat = origin_lat
        self.polygon_xy: Optional[List[Tuple[float, float]]] = None  # Polygon in local x/y coordinates
        self.polygon_lonlat: Optional[List[Tuple[float, float]]] = None  # Polygon in lon/lat
        # Exclusion zones (rocks, swim areas): must stay outside these while inside polygon_xy
        self.hazard_polygons_xy: List[List[Tuple[float, float]]] = []
    
    def lonlat_to_xy(self, lon: float, lat: float) -> Tuple[float, float]:
        """
        Convert longitude/latitude to local x/y meters using equirectangular projection.
        
        Args:
            lon: Longitude in degrees
            lat: Latitude in degrees
            
        Returns:
            (x, y) tuple in meters from origin
        """
        if self.origin_lon is None or self.origin_lat is None:
            raise ValueError("Origin coordinates not set. Call load_geofence() first or set origin manually.")
        
        x = (math.radians(lon) - math.radians(self.origin_lon)) * self.earth_radius * math.cos(math.radians(self.origin_lat))
        y = (math.radians(lat) - math.radians(self.origin_lat)) * self.earth_radius
        return x, y
    
    def xy_to_lonlat(self, x: float, y: float) -> Tuple[float, float]:
        """
        Convert local x/y meters to longitude/latitude.
        
        Args:
            x: X coordinate in meters (East from origin)
            y: Y coordinate in meters (North from origin)
            
        Returns:
            (lon, lat) tuple in degrees
        """
        if self.origin_lon is None or self.origin_lat is None:
            raise ValueError("Origin coordinates not set.")
        
        lon = math.degrees(math.radians(self.origin_lon) + x / (self.earth_radius * math.cos(math.radians(self.origin_lat))))
        lat = math.degrees(math.radians(self.origin_lat) + y / self.earth_radius)
        return lon, lat
    
    def load_geofence(self, map_name: str, maps_dir: Optional[Path] = None) -> bool:
        """
        Load geofence polygon from GeoJSON file.
        
        Args:
            map_name: Name of the map (without .geojson extension)
            maps_dir: Directory containing GeoJSON files. If None, uses default location.
            
        Returns:
            True if successfully loaded, False otherwise
        """
        if maps_dir is None:
            # Default to foxglove/maps directory relative to argo root
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[2]  # support -> nodes -> argo
            maps_dir = argo_dir / "foxglove" / "maps"
        
        geojson_path = maps_dir / f"{map_name}.geojson"
        
        if not geojson_path.exists():
            return False
        
        try:
            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            # Find home waypoint for origin
            home_found = False
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                if props.get('name') == 'home' and props.get('type') == 'waypoint':
                    coords = feature['geometry']['coordinates']
                    # GeoJSON format: [longitude, latitude, altitude]
                    self.origin_lon = coords[0]
                    self.origin_lat = coords[1]
                    home_found = True
                    break
            
            if not home_found:
                # Use default origin if home not found
                self.origin_lon = 8.5448386  # Default fallback
                self.origin_lat = 47.3981555
            
            # Sailing boundary: prefer explicit LineString (Hotel dei Pini style) so a mis-tagged
            # hazard Polygon with type "sailing_area" cannot replace the whole course.
            polygon_found = False
            self.hazard_polygons_xy = []
            features = geojson_data.get('features', [])

            for feature in features:
                props = feature.get('properties', {})
                geom = feature.get('geometry', {})
                if geom.get('type') == 'LineString' and props.get('type') == 'sailing_boundary':
                    coords = geom.get('coordinates', [])
                    self.polygon_lonlat = [(coord[0], coord[1]) for coord in coords]
                    self.polygon_xy = [self.lonlat_to_xy(coord[0], coord[1]) for coord in coords]
                    polygon_found = True
                    break

            if not polygon_found:
                for feature in features:
                    props = feature.get('properties', {})
                    geom = feature.get('geometry', {})
                    if geom.get('type') != 'Polygon':
                        continue
                    prop_type = props.get('type', '')
                    name_l = (props.get('name') or '').lower()
                    if prop_type == 'hazard' or 'hazard' in name_l:
                        continue
                    if prop_type != 'sailing_area':
                        continue
                    coords = geom.get('coordinates', [])
                    if not coords:
                        continue
                    outer_ring = coords[0]
                    self.polygon_lonlat = [(coord[0], coord[1]) for coord in outer_ring]
                    self.polygon_xy = [self.lonlat_to_xy(coord[0], coord[1]) for coord in outer_ring]
                    polygon_found = True
                    break

            for feature in features:
                props = feature.get('properties', {})
                geom = feature.get('geometry', {})
                if geom.get('type') != 'Polygon':
                    continue
                prop_type = props.get('type', '')
                name_l = (props.get('name') or '').lower()
                is_hazard = prop_type == 'hazard' or 'hazard' in name_l
                if not is_hazard:
                    continue
                coords = geom.get('coordinates', [])
                if not coords or len(coords[0]) < 3:
                    continue
                outer_ring = coords[0]
                hz_xy = [self.lonlat_to_xy(c[0], c[1]) for c in outer_ring]
                self.hazard_polygons_xy.append(hz_xy)

            return polygon_found
            
        except Exception as e:
            print(f"Error loading geofence: {e}")
            return False

    @staticmethod
    def _is_xy_inside_ring(ring: List[Tuple[float, float]], x: float, y: float) -> bool:
        """Ray casting: True if (x,y) is inside closed ring (>=3 vertices)."""
        if ring is None or len(ring) < 3:
            return False
        n = len(ring)
        inside = False
        p1x, p1y = ring[0]
        for i in range(1, n + 1):
            p2x, p2y = ring[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                            if p1x == p2x or x <= xinters:
                                inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    @staticmethod
    def _min_distance_to_ring_edges(ring: List[Tuple[float, float]], x: float, y: float) -> float:
        """Minimum Euclidean distance from point to edges of closed ring."""
        if ring is None or len(ring) < 2:
            return float('inf')
        min_distance = float('inf')
        n = len(ring)
        for i in range(n):
            p1x, p1y = ring[i]
            p2x, p2y = ring[(i + 1) % n]
            dx = p2x - p1x
            dy = p2y - p1y
            px = x - p1x
            py = y - p1y
            dot = px * dx + py * dy
            len_sq = dx * dx + dy * dy
            if len_sq == 0:
                dist = math.sqrt(px * px + py * py)
            else:
                t = max(0, min(1, dot / len_sq))
                proj_x = p1x + t * dx
                proj_y = p1y + t * dy
                dist = math.sqrt((x - proj_x) ** 2 + (y - proj_y) ** 2)
            min_distance = min(min_distance, dist)
        return min_distance

    def _signed_distance_inclusive_polygon(self, ring: List[Tuple[float, float]], x: float, y: float) -> float:
        """Negative when inside ring (sailing area), positive when outside."""
        min_d = self._min_distance_to_ring_edges(ring, x, y)
        inside = self._is_xy_inside_ring(ring, x, y)
        return -min_d if inside else min_d

    def _signed_distance_exclusion_polygon(self, ring: List[Tuple[float, float]], x: float, y: float) -> float:
        """Hazard / no-go: positive when inside forbidden polygon, negative clearance when outside."""
        min_d = self._min_distance_to_ring_edges(ring, x, y)
        inside = self._is_xy_inside_ring(ring, x, y)
        return min_d if inside else -min_d
    
    def is_point_inside_polygon(self, x: float, y: float) -> bool:
        """
        True if the point is in navigable water: inside the sailing boundary and
        outside all hazard polygons.
        """
        if self.polygon_xy is None or len(self.polygon_xy) < 3:
            return False
        if not self._is_xy_inside_ring(self.polygon_xy, x, y):
            return False
        for haz in self.hazard_polygons_xy:
            if self._is_xy_inside_ring(haz, x, y):
                return False
        return True
    
    def is_point_inside_polygon_lonlat(self, lon: float, lat: float) -> bool:
        """
        True if lon/lat is in navigable water (inside sailing boundary, outside hazards).
        """
        x, y = self.lonlat_to_xy(lon, lat)
        return self.is_point_inside_polygon(x, y)
    
    def distance_to_boundary(self, x: float, y: float) -> float:
        """
        Signed distance to the safe-sailing boundary (outer fence + hazard perimeters).

        Negative: safely inside navigable water (including clearance to nearest constraint).
        Positive: outside sailing area or inside a hazard (violation).
        """
        if self.polygon_xy is None or len(self.polygon_xy) < 2:
            return float('inf')

        d = self._signed_distance_inclusive_polygon(self.polygon_xy, x, y)
        for haz in self.hazard_polygons_xy:
            d = max(d, self._signed_distance_exclusion_polygon(haz, x, y))
        return d
    
    def distance_to_boundary_lonlat(self, lon: float, lat: float) -> float:
        """
        Calculate minimum distance from point (in lon/lat) to polygon boundary.
        
        Args:
            lon: Longitude in degrees
            lat: Latitude in degrees
            
        Returns:
            Distance in meters to nearest edge. Negative if inside, positive if outside.
        """
        x, y = self.lonlat_to_xy(lon, lat)
        return self.distance_to_boundary(x, y)
    
    def predict_future_position(self, lat: float, lon: float, heading_deg: float, 
                                speed_ms: float, time_sec: float) -> Tuple[float, float]:
        """
        Predict future position based on current position, heading, and speed.
        
        Uses simple projection (good for short distances).
        
        Args:
            lat: Current latitude in degrees
            lon: Current longitude in degrees
            heading_deg: Heading in degrees (0-360, 0=North, 90=East)
            speed_ms: Speed in meters per second
            time_sec: Time to predict ahead in seconds
            
        Returns:
            (predicted_lat, predicted_lon) tuple in degrees
        """
        # Convert heading to radians (0 = North, clockwise)
        # Note: heading_deg is compass heading (0=North, 90=East)
        # For lat/lon projection, we need: 0=North, 90=East
        heading_rad = math.radians(heading_deg)
        
        # Calculate distance to travel
        distance_m = speed_ms * time_sec
        
        # Simple projection (good for small distances)
        # North component (latitude)
        dlat = distance_m * math.cos(heading_rad) / self.earth_radius
        # East component (longitude)
        dlon = distance_m * math.sin(heading_rad) / (self.earth_radius * math.cos(math.radians(lat)))
        
        # Convert to degrees
        predicted_lat = lat + math.degrees(dlat)
        predicted_lon = lon + math.degrees(dlon)
        
        return predicted_lat, predicted_lon
    
    def predict_boundary_crossing_time(self, lat: float, lon: float, heading_deg: float,
                                       speed_ms: float, max_lookahead_sec: float = 60.0) -> Optional[float]:
        """
        Predict when the boat will cross the boundary if continuing on current course.
        
        Args:
            lat: Current latitude in degrees
            lon: Current longitude in degrees
            heading_deg: Current heading in degrees
            speed_ms: Current speed in meters per second
            max_lookahead_sec: Maximum time to look ahead (seconds)
            
        Returns:
            Time in seconds until boundary crossing, or None if not crossing within lookahead
        """
        if speed_ms <= 0:
            return None
        
        # Check current position
        x, y = self.lonlat_to_xy(lon, lat)
        is_currently_inside = self.is_point_inside_polygon(x, y)
        
        # Binary search for crossing time
        t_min = 0.0
        t_max = max_lookahead_sec
        tolerance = 0.1  # 0.1 second tolerance
        
        # Check if we'll cross at all
        pred_lat, pred_lon = self.predict_future_position(lat, lon, heading_deg, speed_ms, t_max)
        pred_x, pred_y = self.lonlat_to_xy(pred_lon, pred_lat)
        is_future_inside = self.is_point_inside_polygon(pred_x, pred_y)
        
        # If both inside or both outside, no crossing
        # However, if currently outside and heading further out, don't predict a crossing
        # (we only care about crossing from inside to outside, or returning from outside to inside)
        if is_currently_inside == is_future_inside:
            return None
        
        # If currently outside and heading further out (future also outside), no crossing to predict
        # Only predict if we're crossing from inside to outside, or outside to inside
        if not is_currently_inside and not is_future_inside:
            # Both outside - check if we're heading toward or away from boundary
            current_dist = self.distance_to_boundary(x, y)
            future_dist = self.distance_to_boundary(pred_x, pred_y)
            # If getting further away (more negative), no crossing to predict
            if abs(future_dist) > abs(current_dist):
                return None
        
        # Binary search for crossing point
        while t_max - t_min > tolerance:
            t_mid = (t_min + t_max) / 2.0
            pred_lat, pred_lon = self.predict_future_position(lat, lon, heading_deg, speed_ms, t_mid)
            pred_x, pred_y = self.lonlat_to_xy(pred_lon, pred_lat)
            is_mid_inside = self.is_point_inside_polygon(pred_x, pred_y)
            
            if is_mid_inside == is_currently_inside:
                t_min = t_mid
            else:
                t_max = t_mid
        
        return (t_min + t_max) / 2.0
    
    def get_polygon_xy(self) -> Optional[List[Tuple[float, float]]]:
        """Get polygon in local x/y coordinates."""
        return self.polygon_xy
    
    def get_polygon_lonlat(self) -> Optional[List[Tuple[float, float]]]:
        """Get polygon in lon/lat coordinates."""
        return self.polygon_lonlat

