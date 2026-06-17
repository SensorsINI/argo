#!/usr/bin/env python3

# 1) Standard library
import json
import math
import os
import re
import sys
from pathlib import Path
from typing import List, Optional, Tuple

# 2) Third-party
import numpy as np

# 3) Path modifications
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'support'))

# 4) Local imports
from .base import BaseController, BoatState, ControlCommand, relative_wind_angle_to_sail_cmd, signed_angle_difference_degrees


class ReturnToHomeController(BaseController):
    """
    Return-to-home controller that navigates back to starting position.
    """

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        self.name = 'ReturnToHomeController'
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)
        self.connection_timeout = config.get('shore_connection_timeout', 120.0)  # seconds
        # Arrival / resume thresholds in meters (home distance from BoatState is converted from nm).
        arrival_m = config.get('arrival_distance_m')
        if arrival_m is None:
            nm_legacy = config.get('arrival_distance_nm')
            arrival_m = (nm_legacy * 1852.0) if nm_legacy is not None else 10.0
        self.arrival_distance_m = float(arrival_m)
        # Wider radius to *leave* "at home" hold — avoids GPS noise crossing the arrival
        # threshold and immediately re-commanding a full approach (boat keeps sailing).
        resume_m = config.get('resume_navigation_distance_m')
        if resume_m is None:
            nm_resume = config.get('resume_navigation_distance_nm')
            if nm_resume is not None:
                resume_m = nm_resume * 1852.0
            else:
                resume_m = max(self.arrival_distance_m * 1.5, self.arrival_distance_m + 8.0)
        self.resume_navigation_distance_m = float(resume_m)
        self.hold_sail = config.get('hold_sail', -1.0)  # sheet in while holding at home
        self._at_home_hold = False
        self._logged_hold_entry = False
        # Reverse of approach_1…N (opposite course) before final leg to GPS home
        map_name = config.get('geofence_map_name')
        forward = self._load_approach_waypoints(map_name) if map_name else []
        self._rth_reverse_legs: List[Tuple[float, float]] = list(reversed(forward))
        self._rth_leg_index = 0
        self._rth_logged_start = False

    def _load_approach_waypoints(self, map_name: str) -> List[Tuple[float, float]]:
        """Load approach_1, approach_2, … from GeoJSON (same convention as Crosser)."""
        try:
            script_path = Path(__file__).resolve()
            argo_dir = script_path.parents[2]
            geojson_path = argo_dir / "foxglove" / "maps" / f"{map_name}.geojson"
            if not geojson_path.exists():
                return []
            with open(geojson_path, 'r') as f:
                data = json.load(f)
            found: List[Tuple[int, float, float]] = []
            pat = re.compile(r'^approach_(\d+)$', re.I)
            for feature in data.get('features', []):
                props = feature.get('properties', {})
                if props.get('type') != 'waypoint':
                    continue
                name = props.get('name') or ''
                m = pat.match(name.strip())
                if not m:
                    continue
                geom = feature.get('geometry', {})
                if geom.get('type') != 'Point':
                    continue
                coords = geom.get('coordinates', [])
                if len(coords) < 2:
                    continue
                found.append((int(m.group(1)), float(coords[0]), float(coords[1])))
            found.sort(key=lambda t: t[0])
            return [(lon, lat) for _, lon, lat in found]
        except Exception as e:
            if self.logger:
                self.logger.warn(f"RTH: could not load approach waypoints: {e}")
            return []

    def _bearing_and_distance_to_lonlat(
        self, state: BoatState, lon: float, lat: float
    ) -> Tuple[Optional[float], Optional[float]]:
        if state.current_latitude is None or state.current_longitude is None:
            return None, None
        lat1 = math.radians(state.current_latitude)
        lon1 = math.radians(state.current_longitude)
        lat2 = math.radians(lat)
        lon2 = math.radians(lon)
        dlon = lon2 - lon1
        y = math.sin(dlon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        bearing_deg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
        a = math.sin((lat2 - lat1) / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
        c = 2 * math.asin(min(1.0, math.sqrt(a)))
        distance_m = 6378137.0 * c
        return bearing_deg, distance_m

    def reset(self):
        super().reset()
        self._at_home_hold = False
        self._logged_hold_entry = False
        self._rth_leg_index = 0
        self._rth_logged_start = False

    def should_activate(self, state: BoatState) -> bool:
        if state.return_to_home_active:
            return True
        if state.last_shore_contact is not None:
            import time
            time_since_contact = time.time() - state.last_shore_contact
            if time_since_contact > self.connection_timeout and not state.shore_connected:
                return True
        return False

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate control commands to navigate toward home position."""
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)

        self.publish_state('return_to_home')

        bearing_to_home = state.get_bearing_to_home()
        distance_to_home_nm = state.get_distance_to_home()
        distance_to_home_m = (
            distance_to_home_nm * 1852.0
            if distance_to_home_nm is not None
            else None
        )

        if bearing_to_home is None or distance_to_home_m is None:
            # Fall back to maintaining current heading
            compass_err = signed_angle_difference_degrees(
                state.target_heading, state.compass_heading)
            cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
            cmd_rudder = self.clip_rudder(cmd_rudder)
            cmd_sail = 0.0
            return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)

        # Opposite course: approach_N → … → approach_1, then GPS home (same GeoJSON as Crosser launch).
        if self._rth_reverse_legs and self._rth_leg_index < len(self._rth_reverse_legs):
            while self._rth_leg_index < len(self._rth_reverse_legs):
                lon_w, lat_w = self._rth_reverse_legs[self._rth_leg_index]
                brg_w, dist_w = self._bearing_and_distance_to_lonlat(state, lon_w, lat_w)
                if dist_w is None or brg_w is None:
                    break
                if dist_w < self.arrival_distance_m:
                    self._rth_leg_index += 1
                    if self._rth_leg_index >= len(self._rth_reverse_legs):
                        self.log_entry(
                            "RTH: reverse approach legs complete — navigating to home",
                            level="INFO",
                        )
                    continue
                break
            if self._rth_leg_index < len(self._rth_reverse_legs):
                lon_w, lat_w = self._rth_reverse_legs[self._rth_leg_index]
                brg_w, dist_w = self._bearing_and_distance_to_lonlat(state, lon_w, lat_w)
                if brg_w is not None:
                    if not self._rth_logged_start:
                        self.log_entry(
                            f"RTH: {len(self._rth_reverse_legs)} reverse waypoint(s) then home",
                            level="INFO",
                        )
                        self._rth_logged_start = True
                    state.target_heading = brg_w
                    compass_err = signed_angle_difference_degrees(brg_w, state.compass_heading)
                    cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
                    cmd_rudder = self.clip_rudder(cmd_rudder)
                    cmd_sail = 0.0
                    if state.wind_angle is not None:
                        wind_sail_cmd = relative_wind_angle_to_sail_cmd(state.wind_angle)
                        cmd_sail = self.sail_wind_gain * wind_sail_cmd
                    return ControlCommand(
                        rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp
                    )

        # Hysteresis: enter hold inside inner radius; only resume navigation outside outer radius.
        if distance_to_home_m < self.arrival_distance_m:
            self._at_home_hold = True
        elif distance_to_home_m > self.resume_navigation_distance_m:
            self._at_home_hold = False
            self._logged_hold_entry = False

        if self._at_home_hold:
            if not self._logged_hold_entry:
                self.log_entry(
                    f"Arrived at home position (distance: {distance_to_home_m:.1f}m, "
                    f"holding until >{self.resume_navigation_distance_m:.1f}m to resume)",
                    level="INFO")
                self._logged_hold_entry = True
            if state.compass_heading is not None:
                state.target_heading = state.compass_heading
            hold_sail = float(np.clip(self.hold_sail, -1.0, 1.0))
            return ControlCommand(
                rudder=0.0, sail=hold_sail, timestamp=state.timestamp)

        # Navigate toward home bearing
        state.target_heading = bearing_to_home

        compass_err = signed_angle_difference_degrees(bearing_to_home, state.compass_heading)
        cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = self.clip_rudder(cmd_rudder)

        # Wind-aware sail control
        cmd_sail = 0.0
        if state.wind_angle is not None:
            wind_sail_cmd = relative_wind_angle_to_sail_cmd(state.wind_angle)
            cmd_sail = self.sail_wind_gain * wind_sail_cmd

        return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)


