#!/usr/bin/env python3

# 1) Standard library
import os
import sys
import time
import math
import json
from pathlib import Path

# 2) Third-party
import numpy as np
from std_msgs.msg import Bool, Float64

# 3) Path modifications
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'support'))

# 4) Local imports
from .base import BaseController, BoatState, ControlCommand, signed_angle_difference_degrees

# Local/support utilities
from safe_publish import safe_publish
from geofence_manager import GeofenceManager


class CrosserController(BaseController):
    """
    Crosser controller that crosses from one side of the pond to the other,
    targeting the middle waypoint or center of the sailing area.
    
    State and Maneuver Hierarchy:
    =============================
    
    The controller tracks TWO separate concepts:
    
    1. **Crosser State** (what the controller is trying to achieve):
       - toward_middle: Navigating toward the middle waypoint
       - crossing_through: Maintaining steady heading after passing middle
       - turning_around: Executing a turn at the boundary
       - tacking_upwind: Zigzagging upwind when middle is in no-go zone
    
    2. **Maneuver** (how the boat is currently sailing):
       - sailing: Normal sailing (default maneuver)
       - tacking: Turning through the wind (bow through wind)
       - jibing: Turning with wind behind (stern through wind)
    
    Logs show both: "State (MANEUVER)" e.g., "Turning Around (TACKING)"
    
    State Machine (Crosser States):
    ================================
    
    The controller uses a state machine with four states:
    
    1. TOWARD_MIDDLE
       - Initial state when controller activates or resumes after human control
       - Goal: Navigate toward the middle waypoint (or center of sailing area)
       - Uses P controller for heading control to target bearing
       - Transitions:
         * → CROSSING_THROUGH: When distance_to_middle < arrival_distance_m (reached middle)
         * → TACKING_UPWIND: When middle is in no-go zone (cannot sail directly upwind)
    
    2. CROSSING_THROUGH
       - Active after passing through the middle waypoint
       - Goal: Maintain steady heading to cross to opposite side of pond
       - Stores crossing_heading when entering this state
       - Continues on stored heading until approaching boundary
       - Transitions:
         * → TURNING_AROUND: When abs(distance_to_boundary) < boundary_turn_threshold (approaching boundary)
    
    3. TURNING_AROUND
       - Active when approaching or at boundary
       - Goal: Turn around (tack or jibe) to head back toward middle
       - Determines maneuver type based on wind angle:
         * TACK: If sailing upwind (wind_angle < 80° from heading)
         * JIBE: If sailing downwind (wind_angle > 80° from heading)
       - Uses tack_angle (typically 100°) for target heading to ensure successful turn
       - Monitors for stalling in stays (too close to wind) and adjusts if needed
       - Transitions:
         * → TOWARD_MIDDLE: When heading_error < 20° AND heading_to_middle < 60° (turn complete, heading toward middle)
    
    4. TACKING_UPWIND
       - Active when middle is in no-go zone (cannot sail directly toward it)
       - Goal: Zigzag upwind using tacks to make progress toward middle
       - Calculates optimal tacking heading based on wind direction
       - Switches tacks when:
         * heading_error < 15° (reached tack target)
         * tack_cooldown has passed
         * angle_to_desired > 90° (getting too far off course)
       - Transitions:
         * → TOWARD_MIDDLE: When middle is no longer in no-go zone (can sail directly)
    
    State Reset:
    ============
    - When human control starts: State is preserved (human can reposition boat)
    - When autonomous control resumes: Always resets to TOWARD_MIDDLE (assumes human repositioned boat)
    
    Safety Features:
    ================
    - Minimum rudder enforcement when near boundaries (min_rudder_near_boundary)
    - Emergency turn mode when very close to boundary (< boundary_emergency_threshold)
    - Stalling detection during tacks (tack_min_angle_from_wind)
    - Wind-aware sail control based on apparent wind angle
    """

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        self.name = 'CrosserController'
        # P controller parameters for heading control
        self.p_heading_gain = config.get('p_heading_gain', 0.3)  # Proportional gain for heading error
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)  # Heading error for full rudder deflection
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)
        
        self.arrival_distance_m = config.get('arrival_distance_m', 10.0)  # meters
        self.boundary_turn_threshold = config.get('boundary_turn_threshold', 15.0)  # meters from boundary to start turn
        self.patrol_lookahead_time = config.get('patrol_lookahead_time', 20.0)  # seconds to predict ahead for boundary crossing
        self.tack_angle = config.get('tack_angle', 90.0)  # degrees for tacking
        # Log loaded parameter for debugging
        if self.logger:
            self.logger.info(f"CrosserController: arrival_distance_m = {self.arrival_distance_m:.1f}m, boundary_turn_threshold = {self.boundary_turn_threshold:.1f}m, patrol_lookahead_time = {self.patrol_lookahead_time:.1f}s (from config)")
        self.tack_min_angle_from_wind = config.get('tack_min_angle_from_wind', 50.0)  # minimum angle from wind during tack to avoid stays
        self.no_go_zone_angle = config.get('no_go_zone_angle', 45.0)  # degrees - angle from wind where sailing is impossible
        self.min_rudder_near_boundary = config.get('min_rudder_near_boundary', 0.3)  # minimum rudder command when close to boundary
        self.boundary_emergency_threshold = config.get('boundary_emergency_threshold', 5.0)  # meters - emergency turn threshold
        self.turn_rudder_gain_multiplier = config.get('turn_rudder_gain_multiplier', 2.0)  # multiplier for rudder gain during turns
        self.crossing_state = 'toward_middle'  # 'toward_middle', 'crossing_through', 'turning_around', 'tacking_upwind'
        
        self.geofence_manager = GeofenceManager()
        map_name = config.get('geofence_map_name', 'Argo Irchel pond sailing area')
        if not self.geofence_manager.load_geofence(map_name):
            self.log_entry(f"Failed to load geofence for map '{map_name}'", level="WARN")
        else:
            self.log_entry(f"Geofence loaded: {map_name}", level="INFO")
        
        # Load middle waypoint or calculate center
        self.middle_lon = None
        self.middle_lat = None
        self._load_middle_waypoint(map_name)
        
        if self.middle_lon is None or self.middle_lat is None:
            self.log_entry("No middle waypoint found, calculating center of sailing area", level="INFO")
            self._calculate_sailing_area_center(map_name)
        
        if self.middle_lon is not None and self.middle_lat is not None:
            self.log_entry(f"Target waypoint: lat={self.middle_lat:.6f}, lon={self.middle_lon:.6f}", level="INFO")
        else:
            self.log_entry("Warning: No middle waypoint or center calculated", level="WARN")
        
        self._last_captains_log_time = 0.0
        self._last_logged_rudder = None
        self._last_logged_sail = None
        self._last_logged_target_heading = None
        self._last_logged_distance = None
        self._last_logged_state = None
        
        self._arrival_logged = False
        self._crossing_heading = None  # Store the steady heading when crossing through middle
        self._turning_target_heading = None  # Target heading when turning around
        self._turning_maneuver_type = None  # Type of turn: 'tacking' or 'jibing'
        self._turning_side_sign = None  # Locked side of wind during an active turn (+1 starboard, -1 port)
        self._current_maneuver = 'sailing'  # Current sailing maneuver: 'sailing', 'tacking', 'jibing'
        self._tacking_target_heading = None  # Target heading when tacking upwind
        self._last_tack_time = 0.0  # Time of last tack to prevent rapid switching
        self._tack_cooldown = 5.0  # Minimum seconds between tacks
        self._tack_start_heading = None  # Heading when tack started (to detect when we've passed through wind)
        self._tack_wind_direction = None  # Wind direction when tack started
        self._last_state_log_time = 0.0  # For throttling periodic state logs

    def _log_state_transition(self, old_state: str, new_state: str, reason: str, details: dict = None):
        """Log state transitions with detailed reasons immediately."""
        detail_str = ""
        if details:
            detail_parts = [f"{k}={v:.1f}" if isinstance(v, float) else f"{k}={v}" for k, v in details.items()]
            detail_str = f" ({', '.join(detail_parts)})"
        self.log_entry(f"STATE TRANSITION: {old_state} → {new_state} | Reason: {reason}{detail_str}", level="INFO")

    def _load_middle_waypoint(self, map_name: str):
        """Load the 'middle' waypoint from GeoJSON file."""
        try:
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[2]  # controllers -> nodes -> argo
            maps_dir = argo_dir / "foxglove" / "maps"
            geojson_path = maps_dir / f"{map_name}.geojson"
            
            if not geojson_path.exists():
                if self.logger:
                    self.logger.warn(f"GeoJSON file not found: {geojson_path}")
                return
            
            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            # Find the "middle" waypoint
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                if props.get('name') == 'middle' and props.get('type') == 'waypoint':
                    coords = feature['geometry']['coordinates']
                    # GeoJSON format: [longitude, latitude, altitude]
                    self.middle_lon = coords[0]
                    self.middle_lat = coords[1]
                    if self.logger:
                        self.logger.info(f"Loaded 'middle' waypoint: lat={self.middle_lat:.6f}, lon={self.middle_lon:.6f}")
                    return
                    
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Error loading middle waypoint: {e}")

    def _calculate_sailing_area_center(self, map_name: str):
        """Calculate the center of the sailing area polygon."""
        try:
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[2]  # controllers -> nodes -> argo
            maps_dir = argo_dir / "foxglove" / "maps"
            geojson_path = maps_dir / f"{map_name}.geojson"
            
            if not geojson_path.exists():
                if self.logger:
                    self.logger.warn(f"GeoJSON file not found: {geojson_path}")
                return
            
            with open(geojson_path, 'r') as f:
                geojson_data = json.load(f)
            
            # Find sailing area polygon
            all_coords = []
            for feature in geojson_data.get('features', []):
                props = feature.get('properties', {})
                geom = feature.get('geometry', {})
                
                # Look for Polygon with type "sailing_area"
                if geom.get('type') == 'Polygon' and props.get('type') == 'sailing_area':
                    coords = geom.get('coordinates', [])
                    if coords:
                        # Get outer ring (first ring in coordinates)
                        outer_ring = coords[0]
                        for coord in outer_ring:
                            all_coords.append(coord)
                
                # Also try LineString boundaries
                elif geom.get('type') == 'LineString' and props.get('type') == 'sailing_boundary':
                    coords = geom.get('coordinates', [])
                    for coord in coords:
                        all_coords.append(coord)
            
            if all_coords:
                # Calculate center of all boundary coordinates
                all_lons = [coord[0] for coord in all_coords]
                all_lats = [coord[1] for coord in all_coords]
                
                self.middle_lon = sum(all_lons) / len(all_lons)
                self.middle_lat = sum(all_lats) / len(all_lats)
                
                if self.logger:
                    self.logger.info(f"Calculated sailing area center: lat={self.middle_lat:.6f}, lon={self.middle_lon:.6f}")
                    
        except Exception as e:
            if self.logger:
                self.logger.warn(f"Error calculating sailing area center: {e}")

    def _calculate_bearing_to_middle(self, state: BoatState) -> tuple:
        """
        Calculate bearing and distance to middle waypoint.
        
        Returns:
            (bearing_deg, distance_m) or (None, None) if calculation not possible
        """
        if (self.middle_lon is None or self.middle_lat is None or
            state.current_latitude is None or state.current_longitude is None):
            return None, None
        
        lat1 = math.radians(state.current_latitude)
        lon1 = math.radians(state.current_longitude)
        lat2 = math.radians(self.middle_lat)
        lon2 = math.radians(self.middle_lon)
        
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing = math.atan2(y, x)
        bearing_deg = (math.degrees(bearing) + 360.0) % 360.0
        
        # Calculate distance using haversine formula
        a = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(math.sqrt(a))
        distance_m = 6378137.0 * c  # Earth radius in meters
        
        return bearing_deg, distance_m

    def _calculate_absolute_wind_direction(self, state: BoatState):
        """
        Calculate absolute wind direction from north.
        
        NOTE: This calculation is now centralized in controller.py._calculate_absolute_wind_direction().
        This method is kept for backwards compatibility but should use the centralized version.
        Future: Add absolute_wind_direction to BoatState and remove this method.
        """
        if state.wind_angle is None or state.compass_heading is None:
            return None
        # wind_angle is relative to boat heading, convert to absolute
        absolute_wind = (state.compass_heading + state.wind_angle) % 360.0
        return absolute_wind
    
    def _is_target_in_no_go_zone(self, target_heading: float, state: BoatState) -> bool:
        """Check if target heading is in the no-go zone (upwind, cannot sail directly)."""
        wind_dir_abs = self._calculate_absolute_wind_direction(state)
        if wind_dir_abs is None:
            return False  # No wind data, assume we can sail
        
        # Calculate angle from wind to target heading
        angle_from_wind = abs(signed_angle_difference_degrees(wind_dir_abs, target_heading))
        
        # No-go zone is typically 45 degrees on either side of wind direction
        return angle_from_wind < self.no_go_zone_angle
    
    def _calculate_tacking_target_heading(self, state: BoatState, desired_direction: float) -> float:
        """Calculate optimal tacking heading when target is upwind.
        
        Args:
            state: Current boat state
            desired_direction: The direction we want to go (toward middle)
            
        Returns:
            Target heading for tacking (either port or starboard tack)
        """
        wind_dir_abs = self._calculate_absolute_wind_direction(state)
        if wind_dir_abs is None or state.compass_heading is None:
            # No wind data - just use current heading with tack angle
            return (state.compass_heading + self.tack_angle) % 360.0
        
        # Determine which side of the wind the desired direction is on
        # Calculate angle from wind to desired direction
        angle_to_desired = signed_angle_difference_degrees(wind_dir_abs, desired_direction)
        
        # Choose tack that makes progress toward desired direction
        # If desired is to the right of wind (positive angle), tack to starboard (right)
        # If desired is to the left of wind (negative angle), tack to port (left)
        if angle_to_desired > 0:
            # Desired direction is to starboard of wind - use starboard tack
            # Starboard tack means wind comes from port, so we sail at wind_dir + tack_angle
            target = (wind_dir_abs + self.tack_angle) % 360.0
        else:
            # Desired direction is to port of wind - use port tack
            # Port tack means wind comes from starboard, so we sail at wind_dir - tack_angle
            target = (wind_dir_abs - self.tack_angle) % 360.0
        
        return target

    def reset(self):
        self.crossing_state = 'toward_middle'
        self._arrival_logged = False
        self._crossing_heading = None
        self._turning_target_heading = None
        self._turning_maneuver_type = None
        self._turning_side_sign = None
        self._current_maneuver = 'sailing'
        self._tacking_target_heading = None
        self._last_tack_time = 0.0
        self._tack_start_heading = None
        self._tack_wind_direction = None
        self.log_entry("Crosser controller activated - starting toward middle", level="INFO")
        self.publish_state('toward_middle')

    def on_human_control_started(self, state: BoatState):
        # When autonomous control resumes after human control, always reset to clean state
        # Assume human has taken action (emergency avoidance or repositioning)
        # Goal: head toward middle from current position
        self.crossing_state = 'toward_middle'
        self._arrival_logged = False
        self._crossing_heading = None
        self._turning_target_heading = None
        self._turning_maneuver_type = None
        self._turning_side_sign = None
        self._current_maneuver = 'sailing'
        self._tacking_target_heading = None
        self._last_tack_time = 0.0
        self._tack_start_heading = None
        self._tack_wind_direction = None
        self.log_entry("Autonomous control resumed - resetting crosser to head toward middle", level="INFO")
        self.publish_state('toward_middle')

    def generate_control(self, state: BoatState) -> ControlCommand:
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)
        
        if self.geofence_manager.polygon_xy is not None:
            state.geofence_polygon = self.geofence_manager.polygon_xy
        
        if state.current_latitude is not None and state.current_longitude is not None:
            state.distance_to_boundary = self.geofence_manager.distance_to_boundary_lonlat(
                state.current_longitude, state.current_latitude)
            
            # Predict boundary crossing time in direction of travel (not just nearest distance)
            # This prevents false triggers when close to side boundaries while heading toward middle
            # NOTE: GPS SOG updates at ~0.2 Hz (every 5 seconds), so may be stale.
            # Controller uses position-based speed estimation as fallback (updates at 1 Hz).
            if (state.compass_heading is not None and state.gps_sog is not None and 
                state.gps_sog > 0.1):  # Only predict if we have speed (gps_sog is in knots)
                speed_ms = state.gps_sog * 0.514444  # Convert knots to m/s (1 knot = 0.514444 m/s)
                state.predicted_boundary_crossing_time = self.geofence_manager.predict_boundary_crossing_time(
                    state.current_latitude, state.current_longitude,
                    state.compass_heading, speed_ms, self.patrol_lookahead_time)
            else:
                state.predicted_boundary_crossing_time = None
            
            if self.parent_node and hasattr(self.parent_node, 'pub_geofence_distance'):
                distance_msg = Float64(data=state.distance_to_boundary if state.distance_to_boundary is not None else 0.0)
                safe_publish(self.parent_node.pub_geofence_distance, distance_msg, self.parent_node)
                is_violation = state.distance_to_boundary is not None and state.distance_to_boundary > 0
                violation_msg = Bool(data=is_violation)
                safe_publish(self.parent_node.pub_geofence_violation, violation_msg, self.parent_node)
                is_grounded = state.distance_to_boundary is not None and state.distance_to_boundary >= 1.0
                grounding_msg = Bool(data=is_grounded)
                safe_publish(self.parent_node.pub_grounding, grounding_msg, self.parent_node)
        
        # Calculate bearing and distance to middle
        bearing_to_middle, distance_to_middle = self._calculate_bearing_to_middle(state)
        
        if bearing_to_middle is None or distance_to_middle is None:
            # Fall back to maintaining current heading
            if state.compass_heading is not None and state.target_heading is not None:
                compass_err = signed_angle_difference_degrees(
                    state.target_heading, state.compass_heading)
                normalized_error = compass_err / self.rudder_full_scale_deg
                cmd_rudder = self.p_heading_gain * normalized_error
                cmd_rudder = float(np.clip(cmd_rudder, -1.0, 1.0))
            else:
                cmd_rudder = 0.0
            cmd_sail = 0.0
            return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)
        
        # Determine crossing state and target heading
        distance_to_boundary = state.distance_to_boundary if state.distance_to_boundary is not None else float('inf')
        abs_distance_to_boundary = abs(distance_to_boundary)
        
        # Use predicted boundary crossing time to check if we're approaching boundary in direction of travel
        # This prevents false triggers when close to side boundaries while heading toward middle
        predicted_crossing_time = state.predicted_boundary_crossing_time if state.predicted_boundary_crossing_time is not None else float('inf')
        approaching_boundary = (predicted_crossing_time < self.patrol_lookahead_time and 
                                abs_distance_to_boundary < self.boundary_turn_threshold)
        
        # Periodic status logging (throttled to every 5 seconds to reduce chatter)
        current_time = time.time()
        if current_time - self._last_state_log_time > 5.0:
            if self.logger:
                crossing_info = f"predicted_crossing={predicted_crossing_time:.1f}s" if predicted_crossing_time != float('inf') else "no_crossing_predicted"
                self.logger.debug(f"Status: state={self.crossing_state}, dist_to_middle={distance_to_middle:.1f}m, dist_to_boundary={abs_distance_to_boundary:.1f}m, {crossing_info}")
            self._last_state_log_time = current_time
        
        if distance_to_middle < self.arrival_distance_m:
            if not self._arrival_logged:
                self.log_entry(f"Arrived at middle waypoint (distance: {distance_to_middle:.1f}m < {self.arrival_distance_m:.1f}m)", level="INFO")
                self._arrival_logged = True
            
            # When at middle, store current heading and continue on same heading
            if self.crossing_state == 'toward_middle':
                old_state = self.crossing_state
                if state.compass_heading is not None:
                    self._crossing_heading = state.compass_heading
                    self.crossing_state = 'crossing_through'
                    self._log_state_transition(old_state, self.crossing_state, 
                                              f"Arrived at middle waypoint (distance={distance_to_middle:.1f}m < {self.arrival_distance_m:.1f}m)",
                                              {'distance_to_middle': distance_to_middle, 'arrival_threshold': self.arrival_distance_m, 'crossing_heading': self._crossing_heading})
                    self.publish_state('crossing_through')
                else:
                    # Fallback: use bearing to middle as crossing heading
                    if bearing_to_middle is not None:
                        self._crossing_heading = bearing_to_middle
                        self.crossing_state = 'crossing_through'
                        self._log_state_transition(old_state, self.crossing_state,
                                                  f"Arrived at middle waypoint (distance={distance_to_middle:.1f}m < {self.arrival_distance_m:.1f}m, using bearing as heading)",
                                                  {'distance_to_middle': distance_to_middle, 'arrival_threshold': self.arrival_distance_m, 'crossing_heading': self._crossing_heading})
                        self.publish_state('crossing_through')
            
            # Continue on stored crossing heading
            if self._crossing_heading is not None:
                target_bearing = self._crossing_heading
            elif state.compass_heading is not None:
                target_bearing = state.compass_heading
            else:
                target_bearing = bearing_to_middle if bearing_to_middle is not None else 0.0
        
        # Check if we're approaching the boundary (need to turn around)
        # IMPORTANT: When in 'crossing_through' state, check boundary more aggressively
        # Use predicted crossing time for 'toward_middle' to prevent false triggers,
        # but for 'crossing_through', also check distance directly since we're actively crossing
        elif self.crossing_state == 'crossing_through' or self.crossing_state == 'toward_middle':
            # For 'crossing_through', check boundary more aggressively (both prediction and direct distance)
            # For 'toward_middle', use prediction to avoid false triggers from side boundaries
            if self.crossing_state == 'crossing_through':
                # In crossing_through, check boundary more proactively
                # Trigger if: predicted crossing OR close to boundary (more aggressive)
                boundary_check = (approaching_boundary or 
                                 abs_distance_to_boundary < self.boundary_turn_threshold)
            else:  # toward_middle
                # Use prediction to avoid false triggers when close to side boundaries
                boundary_check = approaching_boundary
            
            if boundary_check:
                # Approaching boundary - need to turn around
                if self.logger:
                    crossing_info = f", predicted_crossing={predicted_crossing_time:.1f}s" if predicted_crossing_time != float('inf') else ""
                    self.logger.warn(f"⚠️ Approaching boundary: distance={abs_distance_to_boundary:.1f}m < threshold={self.boundary_turn_threshold:.1f}m{crossing_info}, state={self.crossing_state} → starting turn")
                if self._turning_target_heading is None:
                    # Determine whether to tack or jibe based on wind
                    wind_dir_abs = self._calculate_absolute_wind_direction(state)
                    if wind_dir_abs is not None and state.compass_heading is not None:
                        relative_wind_angle = abs(signed_angle_difference_degrees(wind_dir_abs, state.compass_heading))
                        if relative_wind_angle > 80.0:
                            # Sailing downwind - jibe
                            old_state = self.crossing_state
                            self.crossing_state = 'turning_around'
                            self._turning_maneuver_type = 'jibing'
                            self._current_maneuver = 'jibing'
                            self._log_state_transition(old_state, self.crossing_state,
                                                      f"Approaching boundary - sailing downwind, jibing",
                                                      {'boundary_distance': abs_distance_to_boundary, 'threshold': self.boundary_turn_threshold,
                                                       'wind_angle': relative_wind_angle, 'predicted_crossing': predicted_crossing_time if predicted_crossing_time != float('inf') else None})
                            # Store start heading for turn completion detection
                            self._tack_start_heading = state.compass_heading
                            # Publish 'jibing' state for visualization
                            self.publish_state('jibing')
                            
                            # Calculate jibe target: turn to point back toward middle
                            # CRITICAL: Ensure target points toward middle, not away from it
                            if state.compass_heading is not None:
                                candidate_target_1 = (state.compass_heading + self.tack_angle) % 360.0
                                candidate_target_2 = (state.compass_heading - self.tack_angle) % 360.0
                                
                                # Choose the candidate that points more toward the middle
                                if bearing_to_middle is not None:
                                    angle_to_middle_1 = abs(signed_angle_difference_degrees(bearing_to_middle, candidate_target_1))
                                    angle_to_middle_2 = abs(signed_angle_difference_degrees(bearing_to_middle, candidate_target_2))
                                    
                                    if angle_to_middle_1 < angle_to_middle_2:
                                        self._turning_target_heading = candidate_target_1
                                        if self.logger:
                                            self.logger.info(f"Jibe target: {candidate_target_1:.1f}° (angle to middle: {angle_to_middle_1:.1f}°)")
                                    else:
                                        self._turning_target_heading = candidate_target_2
                                        if self.logger:
                                            self.logger.info(f"Jibe target: {candidate_target_2:.1f}° (angle to middle: {angle_to_middle_2:.1f}°)")
                                else:
                                    # Fallback: reverse heading to point back
                                    self._turning_target_heading = (state.compass_heading + 180.0) % 360.0
                                if self._turning_target_heading is not None:
                                    target_angle_to_wind = signed_angle_difference_degrees(wind_dir_abs, self._turning_target_heading)
                                    self._turning_side_sign = 1 if target_angle_to_wind > 0 else -1
                            else:
                                # Last resort: reverse bearing to middle
                                self._turning_target_heading = (bearing_to_middle + 180.0) % 360.0 if bearing_to_middle is not None else 0.0
                                if self._turning_target_heading is not None:
                                    target_angle_to_wind = signed_angle_difference_degrees(wind_dir_abs, self._turning_target_heading)
                                    self._turning_side_sign = 1 if target_angle_to_wind > 0 else -1
                        else:
                            # Sailing upwind - tack (need to be careful to avoid stays)
                            old_state = self.crossing_state
                            self.crossing_state = 'turning_around'
                            self._turning_maneuver_type = 'tacking'
                            self._current_maneuver = 'tacking'
                            self._log_state_transition(old_state, self.crossing_state,
                                                      f"Approaching boundary - sailing upwind, tacking",
                                                      {'boundary_distance': abs_distance_to_boundary, 'threshold': self.boundary_turn_threshold,
                                                       'wind_angle': relative_wind_angle, 'predicted_crossing': predicted_crossing_time if predicted_crossing_time != float('inf') else None})
                            # Store tack start info to detect when we've passed through wind
                            self._tack_start_heading = state.compass_heading
                            self._tack_wind_direction = wind_dir_abs
                            # Publish 'tacking' state for visualization
                            self.publish_state('tacking')
                            
                            # Calculate tack target: turn through the wind using tack_angle
                            # Use wider angle for boundary turns to ensure successful tack with momentum
                            # CRITICAL: Ensure target points toward middle, not away from it
                            candidate_target_1 = (wind_dir_abs + self.tack_angle) % 360.0
                            candidate_target_2 = (wind_dir_abs - self.tack_angle) % 360.0
                            
                            # Choose the candidate that points more toward the middle
                            if bearing_to_middle is not None:
                                angle_to_middle_1 = abs(signed_angle_difference_degrees(bearing_to_middle, candidate_target_1))
                                angle_to_middle_2 = abs(signed_angle_difference_degrees(bearing_to_middle, candidate_target_2))
                                
                                if angle_to_middle_1 < angle_to_middle_2:
                                    self._turning_target_heading = candidate_target_1
                                    if self.logger:
                                        self.logger.info(f"Tack target: {candidate_target_1:.1f}° (angle to middle: {angle_to_middle_1:.1f}°)")
                                else:
                                    self._turning_target_heading = candidate_target_2
                                    if self.logger:
                                        self.logger.info(f"Tack target: {candidate_target_2:.1f}° (angle to middle: {angle_to_middle_2:.1f}°)")
                            else:
                                # Fallback: use wind angle sensor if available
                                if state.wind_angle is not None:
                                    if state.wind_angle > 180:
                                        # Wind from port, tack to starboard
                                        self._turning_target_heading = candidate_target_1
                                    else:
                                        # Wind from starboard, tack to port
                                        self._turning_target_heading = candidate_target_2
                                else:
                                    # Last resort: use first candidate
                                    self._turning_target_heading = candidate_target_1
                            if self._turning_target_heading is not None:
                                target_angle_to_wind = signed_angle_difference_degrees(wind_dir_abs, self._turning_target_heading)
                                self._turning_side_sign = 1 if target_angle_to_wind > 0 else -1
                    else:
                        # No wind data - just reverse heading
                        old_state = self.crossing_state
                        self.crossing_state = 'turning_around'
                        self._turning_maneuver_type = 'turn'  # Generic turn (no wind data to determine tack vs jibe)
                        self._turning_side_sign = None
                        self._log_state_transition(old_state, self.crossing_state,
                                                  f"Approaching boundary - no wind data, reversing heading",
                                                  {'boundary_distance': abs_distance_to_boundary, 'threshold': self.boundary_turn_threshold,
                                                   'predicted_crossing': predicted_crossing_time if predicted_crossing_time != float('inf') else None})
                        # Store start heading for turn completion detection
                        self._tack_start_heading = state.compass_heading if state.compass_heading is not None else None
                        # Publish generic 'turning_around' state for visualization
                        self.publish_state('turning_around')
                        if state.compass_heading is not None:
                            self._turning_target_heading = (state.compass_heading + 180.0) % 360.0
                        else:
                            self._turning_target_heading = (bearing_to_middle + 180.0) % 360.0 if bearing_to_middle is not None else 0.0
                target_bearing = self._turning_target_heading if self._turning_target_heading is not None else state.compass_heading if state.compass_heading is not None else 0.0
            else:
                # Not approaching boundary - continue normal operation
                if self.crossing_state == 'crossing_through':
                    # Continue on stored crossing heading
                    if self._crossing_heading is not None:
                        target_bearing = self._crossing_heading
                    elif state.compass_heading is not None:
                        target_bearing = state.compass_heading
                    else:
                        target_bearing = bearing_to_middle if bearing_to_middle is not None else 0.0
                else:  # toward_middle
                    # Continue heading toward middle
                    target_bearing = bearing_to_middle if bearing_to_middle is not None else state.compass_heading if state.compass_heading is not None else 0.0
        
        # Handle turning_around state (active turn execution)
        elif self.crossing_state == 'turning_around':
            # During turn, check if we're going into stays and adjust
            wind_dir_abs = self._calculate_absolute_wind_direction(state)
            if wind_dir_abs is not None and state.compass_heading is not None and self._turning_target_heading is not None:
                # Check if current heading is too close to wind (in stays)
                angle_to_wind = abs(signed_angle_difference_degrees(wind_dir_abs, state.compass_heading))
                if angle_to_wind < self.tack_min_angle_from_wind:
                    # Too close to wind - adjust target to get away from wind
                    if self._tack_wind_direction is not None:
                        # Keep the same side of wind throughout the turn to avoid side-flip oscillation.
                        # Without this lock, the correction can alternate each control tick and command +/- max rudder.
                        if self._turning_side_sign is None:
                            target_angle_to_wind = signed_angle_difference_degrees(wind_dir_abs, self._turning_target_heading)
                            self._turning_side_sign = 1 if target_angle_to_wind > 0 else -1
                        if self._turning_side_sign > 0:
                            # Starboard side of wind
                            adjusted_target = (wind_dir_abs + self.tack_min_angle_from_wind) % 360.0
                        else:
                            # Port side of wind
                            adjusted_target = (wind_dir_abs - self.tack_min_angle_from_wind) % 360.0
                        self._turning_target_heading = adjusted_target
                        if self.logger:
                            self.logger.warn(f"Adjusting tack to avoid stays - new target: {adjusted_target:.1f}°")
            
            # Use turning target heading (always use it during turn)
            if self._turning_target_heading is not None:
                target_bearing = self._turning_target_heading
            elif state.compass_heading is not None:
                # Fallback: continue current heading if no target set
                target_bearing = state.compass_heading
            else:
                target_bearing = bearing_to_middle if bearing_to_middle is not None else 0.0
            
            # Check if turn is complete (heading toward middle)
            # Don't check completion too early - ensure we've actually turned
            if self._turning_target_heading is not None and state.compass_heading is not None and bearing_to_middle is not None:
                heading_error = abs(signed_angle_difference_degrees(self._turning_target_heading, state.compass_heading))
                heading_to_middle = abs(signed_angle_difference_degrees(bearing_to_middle, state.compass_heading))
                
                # Turn complete if: heading toward middle (primary criterion)
                # Be more lenient - if we're heading generally toward middle, complete the turn
                turn_complete = False
                turn_progress = None
                
                # SAFETY CHECK: Don't complete turn if still close to boundary AND heading toward middle would head back to boundary
                # This prevents the turn-complete-turn cycle near boundaries
                safe_to_complete_turn = True
                if abs_distance_to_boundary < (self.boundary_turn_threshold * 1.5):
                    # Still close to boundary - check if middle is in a safe direction
                    # If predicted_crossing_time is low, heading toward middle will hit boundary
                    if predicted_crossing_time < self.patrol_lookahead_time:
                        safe_to_complete_turn = False
                        if self.logger and (time.time() - self._last_state_log_time) > 5.0:
                            self.logger.debug(
                                f"Turn not complete yet - still too close to boundary "
                                f"(dist={abs_distance_to_boundary:.1f}m, predicted_crossing={predicted_crossing_time:.1f}s)"
                            )
                
                # Primary check: are we heading toward middle AND safe to complete?
                if heading_to_middle < 70.0 and safe_to_complete_turn:  # Within 70 degrees of middle (relaxed from 90)
                    # We're heading toward middle - check if we've made progress
                    # If we started the turn, we should have changed heading significantly
                    if self._tack_start_heading is not None:
                        # Check if we've turned significantly from start
                        turn_progress = abs(signed_angle_difference_degrees(state.compass_heading, self._tack_start_heading))
                        if turn_progress > 40.0:  # We've turned at least 40 degrees (increased from 30)
                            turn_complete = True
                            if self.logger:
                                self.logger.info(f"Turn complete: turned {turn_progress:.1f}°, heading to middle {heading_to_middle:.1f}°")
                    else:
                        # No start heading recorded - just check if heading toward middle
                        if heading_to_middle < 60.0:  # Stricter when no start heading
                            turn_complete = True
                            if self.logger:
                                self.logger.info(f"Turn complete: heading to middle {heading_to_middle:.1f}°")
                
                # Secondary check: are we close to target heading AND heading toward middle?
                if not turn_complete and heading_error < 40.0 and heading_to_middle < 80.0 and safe_to_complete_turn:
                    # Close to target and heading toward middle (both thresholds tightened)
                    turn_complete = True
                    if self.logger:
                        self.logger.info(f"Turn complete: heading error {heading_error:.1f}°, heading to middle {heading_to_middle:.1f}°")
                
                if turn_complete:
                    # Turn complete, head toward middle
                    old_state = self.crossing_state
                    self.crossing_state = 'toward_middle'
                    self._current_maneuver = 'sailing'  # Return to normal sailing
                    details = {'heading_error': heading_error, 'heading_to_middle': heading_to_middle,
                              'distance_to_boundary': abs_distance_to_boundary if abs_distance_to_boundary is not None else None}
                    if turn_progress is not None:
                        details['turn_progress'] = turn_progress
                    self._log_state_transition(old_state, self.crossing_state, 
                                              f"Turn complete - heading toward middle (safe distance: {abs_distance_to_boundary:.1f}m)",
                                              details)
                    self._turning_target_heading = None
                    self._turning_maneuver_type = None
                    self._turning_side_sign = None
                    self._crossing_heading = None
                    self._tack_start_heading = None
                    self._tack_wind_direction = None
                    self.publish_state('toward_middle')
                    target_bearing = bearing_to_middle
        
        # Normal crossing: heading toward middle
        elif self.crossing_state == 'toward_middle':
            self._arrival_logged = False
            
            # Check if middle is in no-go zone (upwind)
            if bearing_to_middle is not None and self._is_target_in_no_go_zone(bearing_to_middle, state):
                # Middle is upwind - need to tack
                old_state = self.crossing_state
                self.crossing_state = 'tacking_upwind'
                self._current_maneuver = 'tacking'  # Tacking maneuver when sailing upwind
                self._tacking_target_heading = self._calculate_tacking_target_heading(state, bearing_to_middle)
                self._last_tack_time = time.time()
                wind_dir_abs = self._calculate_absolute_wind_direction(state)
                angle_from_wind = abs(signed_angle_difference_degrees(wind_dir_abs, bearing_to_middle)) if wind_dir_abs is not None else None
                self._log_state_transition(old_state, self.crossing_state,
                                          f"Middle is upwind (in no-go zone) - switching to tacking mode",
                                          {'bearing_to_middle': bearing_to_middle, 'angle_from_wind': angle_from_wind,
                                           'no_go_zone_angle': self.no_go_zone_angle})
                self.publish_state('tacking_upwind')
                
                # Use tacking target heading
                target_bearing = self._tacking_target_heading if self._tacking_target_heading is not None else bearing_to_middle
            else:
                # Middle is not upwind - can sail directly
                target_bearing = bearing_to_middle if bearing_to_middle is not None else state.compass_heading if state.compass_heading is not None else 0.0
        
        # Tacking upwind toward middle
        elif self.crossing_state == 'tacking_upwind':
            if bearing_to_middle is None:
                # Lost bearing to middle - fall back to current heading
                target_bearing = state.compass_heading if state.compass_heading is not None else 0.0
            elif not self._is_target_in_no_go_zone(bearing_to_middle, state):
                # Middle is no longer upwind - can sail directly
                old_state = self.crossing_state
                self.crossing_state = 'toward_middle'
                self._current_maneuver = 'sailing'  # Return to normal sailing
                self._tacking_target_heading = None
                wind_dir_abs = self._calculate_absolute_wind_direction(state)
                angle_from_wind = abs(signed_angle_difference_degrees(wind_dir_abs, bearing_to_middle)) if wind_dir_abs is not None else None
                self._log_state_transition(old_state, self.crossing_state,
                                          f"Middle is no longer upwind - switching to direct sailing",
                                          {'bearing_to_middle': bearing_to_middle, 'angle_from_wind': angle_from_wind,
                                           'no_go_zone_angle': self.no_go_zone_angle})
                self.publish_state('toward_middle')
                target_bearing = bearing_to_middle
            else:
                # Still upwind - continue tacking
                current_time = time.time()
                
                # Check if we should switch tacks (zigzag pattern)
                if self._tacking_target_heading is not None and state.compass_heading is not None:
                    # Check if we've reached the tacking target heading
                    heading_error = abs(signed_angle_difference_degrees(self._tacking_target_heading, state.compass_heading))
                    
                    # If we're close to target heading and enough time has passed, consider switching tacks
                    if heading_error < 15.0 and (current_time - self._last_tack_time) > self._tack_cooldown:
                        # Check if we've made enough progress or if we should switch tacks
                        # Switch if we're getting too far from the desired direction
                        angle_to_desired = abs(signed_angle_difference_degrees(bearing_to_middle, state.compass_heading))
                        
                        # If we're more than 90 degrees off from desired direction, switch tacks
                        if angle_to_desired > 90.0:
                            # Switch to opposite tack
                            wind_dir_abs = self._calculate_absolute_wind_direction(state)
                            if wind_dir_abs is not None:
                                # Calculate opposite tack
                                current_angle_to_wind = signed_angle_difference_degrees(wind_dir_abs, state.compass_heading)
                                if current_angle_to_wind > 0:
                                    # Currently on starboard tack, switch to port
                                    self._tacking_target_heading = (wind_dir_abs - self.tack_angle) % 360.0
                                else:
                                    # Currently on port tack, switch to starboard
                                    self._tacking_target_heading = (wind_dir_abs + self.tack_angle) % 360.0
                                self._last_tack_time = current_time
                                self.log_entry(f"Switching tacks - new target: {self._tacking_target_heading:.1f}°", level="INFO")
                
                target_bearing = self._tacking_target_heading if self._tacking_target_heading is not None else bearing_to_middle
        
        # Crossing through (after middle, before boundary)
        else:  # crossing_through
            # Continue on stored crossing heading
            if self._crossing_heading is not None:
                target_bearing = self._crossing_heading
            elif state.compass_heading is not None:
                target_bearing = state.compass_heading
            else:
                target_bearing = bearing_to_middle if bearing_to_middle is not None else 0.0
        
        state.target_heading = target_bearing
        
        # Calculate rudder command using P controller
        # P controller: output = Kp * error
        # where Kp = p_heading_gain and error = heading_error / rudder_full_scale_deg
        distance_to_boundary = abs(state.distance_to_boundary) if state.distance_to_boundary is not None else float('inf')
        
        if state.compass_heading is not None:
            compass_err = signed_angle_difference_degrees(target_bearing, state.compass_heading)
            # Normalize error to [-1, 1] range based on full scale deflection
            normalized_error = compass_err / self.rudder_full_scale_deg
            
            # Use more aggressive rudder control during turns
            if self.crossing_state == 'turning_around':
                # During turns, use higher gain for more aggressive control
                effective_gain = self.p_heading_gain * self.turn_rudder_gain_multiplier
                cmd_rudder = effective_gain * normalized_error
                
                # Always ensure minimum rudder during turns if we have ANY error
                # Be very aggressive during turns - any error should get minimum rudder
                if abs(compass_err) > 5.0:  # Very low threshold - be aggressive for any error
                    # Ensure minimum rudder for any error during turns
                    if abs(cmd_rudder) < self.min_rudder_near_boundary:
                        # Increase rudder command to minimum
                        if cmd_rudder >= 0:
                            cmd_rudder = max(cmd_rudder, self.min_rudder_near_boundary)
                        else:
                            cmd_rudder = min(cmd_rudder, -self.min_rudder_near_boundary)
                
                # For moderate errors during turns, use higher minimum rudder
                if abs(compass_err) > 20.0:
                    # Moderate error - use higher minimum rudder for faster turn
                    aggressive_min_rudder = min(0.5, self.min_rudder_near_boundary * 1.3)
                    if abs(cmd_rudder) < aggressive_min_rudder:
                        if cmd_rudder >= 0:
                            cmd_rudder = max(cmd_rudder, aggressive_min_rudder)
                        else:
                            cmd_rudder = min(cmd_rudder, -aggressive_min_rudder)
                
                # For very large errors during turns, use even more aggressive rudder
                if abs(compass_err) > 45.0:
                    # Large error - use higher minimum rudder for faster turn
                    aggressive_min_rudder = min(0.7, self.min_rudder_near_boundary * 2.0)
                    if abs(cmd_rudder) < aggressive_min_rudder:
                        if cmd_rudder >= 0:
                            cmd_rudder = max(cmd_rudder, aggressive_min_rudder)
                        else:
                            cmd_rudder = min(cmd_rudder, -aggressive_min_rudder)
            else:
                # Normal P controller for non-turning states
                cmd_rudder = self.p_heading_gain * normalized_error
            
            # Emergency: if very close to boundary, ensure minimum rudder to turn away
            if distance_to_boundary < self.boundary_emergency_threshold:
                # Very close to boundary - need aggressive turn
                # Determine which direction to turn (away from boundary)
                if state.distance_to_boundary is not None and state.distance_to_boundary > 0:
                    # Outside boundary - turn toward inside (away from boundary)
                    # Use minimum rudder in direction that turns away from boundary
                    if abs(cmd_rudder) < self.min_rudder_near_boundary:
                        # Increase rudder command to minimum
                        if cmd_rudder >= 0:
                            cmd_rudder = max(cmd_rudder, self.min_rudder_near_boundary)
                        else:
                            cmd_rudder = min(cmd_rudder, -self.min_rudder_near_boundary)
                        if self.logger:
                            self.logger.warn(f"Emergency: near boundary ({distance_to_boundary:.1f}m) - applying minimum rudder {cmd_rudder:+.3f}")
                else:
                    # Inside boundary but very close - ensure we can turn
                    if abs(cmd_rudder) < self.min_rudder_near_boundary and abs(compass_err) > 5.0:
                        # If we have a heading error but small rudder, increase it
                        if cmd_rudder >= 0:
                            cmd_rudder = max(cmd_rudder, self.min_rudder_near_boundary)
                        else:
                            cmd_rudder = min(cmd_rudder, -self.min_rudder_near_boundary)
            
            cmd_rudder = float(np.clip(cmd_rudder, -1.0, 1.0))
        else:
            cmd_rudder = 0.0
            compass_err = 0.0
        
        # Wind-aware sail control
        cmd_sail = 0.0
        sail_reason = "no wind data"
        if state.wind_angle is not None:
            wind_sail_cmd = (state.wind_angle - 90.0) / 90.0
            wind_sail_cmd = float(np.clip(wind_sail_cmd, -1.0, 1.0))
            cmd_sail = self.sail_wind_gain * wind_sail_cmd
            sail_reason = f"wind_angle={state.wind_angle:.1f}° (wind-based control, gain={self.sail_wind_gain:.2f})"
        
        # Logging
        current_time = time.time()
        should_log = False
        time_since_last_log = current_time - self._last_captains_log_time
        if self._last_captains_log_time == 0.0:
            should_log = True
        elif time_since_last_log >= 5.0:  # Log at most every 5 seconds
            should_log = True
        if not should_log:
            rudder_changed = (self._last_logged_rudder is None or abs(cmd_rudder - self._last_logged_rudder) > 0.05)
            sail_changed = (self._last_logged_sail is None or abs(cmd_sail - self._last_logged_sail) > 0.05)
            target_changed = (self._last_logged_target_heading is None or abs(target_bearing - self._last_logged_target_heading) > 5.0)
            distance_changed = (self._last_logged_distance is None or (distance_to_middle is not None and abs(distance_to_middle - self._last_logged_distance) > 2.0))
            state_changed = (self._last_logged_state != self.crossing_state)
            if rudder_changed or sail_changed or target_changed or distance_changed or state_changed:
                should_log = True
        
        if should_log:
            log_parts = []
            # Always include controller name to avoid confusion
            log_parts.append(f"Controller: {self.name}")
            # Show both state and maneuver: "state (maneuver)"
            state_name = self.crossing_state
            maneuver_name = self._current_maneuver
            
            # Format state name for display
            state_display = state_name.replace('_', ' ').title()
            maneuver_display = maneuver_name.upper()
            
            # Combine state and maneuver
            mode_desc = f"{state_display} ({maneuver_display})"
            
            # Add extra context for specific states
            if self.crossing_state == 'tacking_upwind':
                mode_desc = f"{state_display} ({maneuver_display}) - middle is upwind"
            
            # Publish the formatted state to /controller/state for visualization
            # This includes both the goal state and the current maneuver
            self.publish_state(mode_desc)
            
            log_parts.append(f"Mode: {mode_desc}")
            if state.compass_heading is not None:
                heading_info = f"heading: current={state.compass_heading:.1f}°, target={target_bearing:.1f}°"
                if state.gps_sog is not None:
                    speed_ms = state.gps_sog * 0.514444
                    heading_info += f", speed={speed_ms:.2f}m/s"
                log_parts.append(f"Heading: {heading_info}")
            if state.compass_heading is not None:
                rudder_reason = f"rudder={cmd_rudder:+.3f}"
                if abs(cmd_rudder) > 0.01:
                    direction = "port" if cmd_rudder < 0 else "starboard"
                    raw_rudder = self.p_heading_gain * (-compass_err / self.rudder_full_scale_deg)
                    if abs(raw_rudder) > 1.0:
                        rudder_reason += f" ({direction}, heading_error={compass_err:+.1f}° (target-current), P_gain={self.p_heading_gain:.2f}, CLIPPED from {raw_rudder:+.3f})"
                    else:
                        rudder_reason += f" ({direction}, heading_error={compass_err:+.1f}° (target-current), P_gain={self.p_heading_gain:.2f})"
                else:
                    rudder_reason += " (neutral, on target)"
                log_parts.append(f"Rudder: {rudder_reason}")
            else:
                log_parts.append(f"Rudder: 0.000 (no heading data)")
            log_parts.append(f"Sail: {cmd_sail:+.3f} ({sail_reason})")
            if distance_to_middle is not None:
                if state.distance_to_boundary is not None:
                    log_parts.append(
                        f"Distance to middle: {distance_to_middle:.1f}m | Distance to boundary: {abs(state.distance_to_boundary):.1f}m"
                    )
                else:
                    log_parts.append(f"Distance to middle: {distance_to_middle:.1f}m")
            log_message = "\n  ".join(log_parts)
            self.log_entry(log_message, level="INFO")
            self._last_captains_log_time = current_time
            self._last_logged_rudder = cmd_rudder
            self._last_logged_sail = cmd_sail
            self._last_logged_target_heading = target_bearing
            self._last_logged_distance = distance_to_middle
            self._last_logged_state = self.crossing_state
        
        self.log_periodic_state(state, interval_sec=30.0)
        return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)

