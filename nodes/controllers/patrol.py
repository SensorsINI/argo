#!/usr/bin/env python3

# 1) Standard library
import os
import sys
import time

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


class PatrolController(BaseController):
    """
    Patrol controller that autonomously patrols a geofence sailing area.
    """

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        self.name = 'PatrolController'
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)

        self.patrol_lookahead_time = config.get('patrol_lookahead_time', 15.0)
        self.boundary_turn_threshold = config.get('boundary_turn_threshold', 15.0)
        self.tack_angle = config.get('tack_angle', 90.0)
        self.broad_reach_angle = config.get('broad_reach_angle', 110.0)

        if parent_node:
            try:
                param_value = parent_node.get_parameter('simulation.grounding_behavior')
                self.grounding_behavior = param_value.get_parameter_value().string_value
            except Exception as e:
                self.grounding_behavior = config.get('grounding_behavior', 'terminate')
                if self.logger:
                    self.logger.warn(f"Could not read simulation.grounding_behavior parameter: {e}, using default 'terminate'")
        else:
            self.grounding_behavior = config.get('grounding_behavior', 'terminate')

        if self.grounding_behavior not in ['terminate', 'reset', 'continue']:
            self.log_entry(f"Invalid grounding_behavior '{self.grounding_behavior}', using 'terminate'", level="WARN")
            self.grounding_behavior = 'terminate'

        self.reset_service_client = None
        if self.grounding_behavior == 'reset' and parent_node:
            from std_srvs.srv import Trigger
            self.reset_service_client = parent_node.create_client(Trigger, '/simulator/reset')

        self.geofence_manager = GeofenceManager()
        map_name = config.get('geofence_map_name', 'Argo Irchel pond sailing area')
        if not self.geofence_manager.load_geofence(map_name):
            self.log_entry(f"Failed to load geofence for map '{map_name}'", level="WARN")
        else:
            self.log_entry(f"Geofence loaded: {map_name}", level="INFO")
            self.publish_state('broad_reach')

        self.patrol_mode = 'broad_reach'
        self.last_turn_time = 0.0

        self._last_captains_log_time = 0.0
        self._last_logged_rudder = None
        self._last_logged_sail = None
        self._last_logged_mode = None
        self._last_logged_target_heading = None
        self._last_logged_distance = None
        self.turn_cooldown = 5.0

        self._stable_broad_reach_target = None
        self._broad_reach_target_lock_time = 0.0
        self._broad_reach_target_lock_duration = 10.0

        self.wind_direction_absolute = None
        self._last_absolute_wind = None

        self._stable_tacking_target = None
        self._stable_jibing_target = None

        self.last_geofence_log_time = 0.0
        self.geofence_log_interval = 5.0
        self.last_reported_distance = None
        self.last_reported_inside = None

        self.true_wind_direction_from_bridge = None

    def reset(self):
        old_mode = self.patrol_mode
        self.patrol_mode = 'broad_reach'
        self.last_turn_time = 0.0
        self._stable_broad_reach_target = None
        self._stable_tacking_target = None
        self._stable_jibing_target = None
        self._broad_reach_target_lock_time = 0.0
        if old_mode != 'broad_reach':
            self.log_entry("Patrol controller activated - starting broad reach sailing", level="INFO")
            self.publish_state('broad_reach')

    def on_human_control_started(self, state: BoatState):
        if self.patrol_mode in ['tacking', 'jibing']:
            old_mode = self.patrol_mode
            self.patrol_mode = 'broad_reach'
            self.last_turn_time = 0.0
            self.log_entry(f"Human control detected - exiting {old_mode} mode", level="INFO")
            self.publish_state('broad_reach')
        if state.compass_heading is not None:
            self._stable_broad_reach_target = state.compass_heading
            self._broad_reach_target_lock_time = time.time()
            self.log_entry(f"New target heading set to current heading: {state.compass_heading:.1f}°", level="INFO")

    def _calculate_absolute_wind_direction(self, state: BoatState):
        # First priority: Use true wind direction from bridge if available (simulation mode)
        if self.true_wind_direction_from_bridge is not None:
            return self.true_wind_direction_from_bridge
        if state.wind_angle is None or state.compass_heading is None:
            return None
        # Fallback: Try to detect if wind_angle is absolute or relative
        # In simulation, wind_angle is absolute (0-360, compass convention)
        # On real boat, wind_angle is relative (0-360, CW from front of boat)
        # Heuristic: If wind_angle is close to compass_heading, it's likely absolute
        # Otherwise, assume it's relative and convert
        calculated_abs_from_relative = (state.compass_heading + state.wind_angle) % 360.0
        heading_match_threshold = 5.0
        if abs(signed_angle_difference_degrees(state.wind_angle, state.compass_heading)) < heading_match_threshold:
            # wind_angle is close to compass_heading, likely absolute
            return state.wind_angle % 360.0
        # Assume wind_angle is relative, convert to absolute
        return calculated_abs_from_relative

    def _determine_sailing_mode(self, state: BoatState) -> str:
        current_time = time.time()
        if (state.current_latitude is None or state.current_longitude is None or
            state.compass_heading is None):
            return 'broad_reach'
        distance = self.geofence_manager.distance_to_boundary_lonlat(
            state.current_longitude, state.current_latitude)
        if current_time - self.last_turn_time < self.turn_cooldown:
            if self.patrol_mode in ['tacking', 'jibing']:
                if distance is not None and abs(distance) > self.boundary_turn_threshold * 2:
                    pass
                else:
                    return self.patrol_mode
            else:
                return self.patrol_mode
        if distance is not None and distance >= 1.0:
            self.log_entry(f"⚠️ Geofence violation detected: {distance:.1f}m outside boundary | Position: ({state.current_latitude:.6f}, {state.current_longitude:.6f})", level="WARN")
        if self.patrol_mode in ['tacking', 'jibing']:
            if abs(distance) > self.boundary_turn_threshold * 2:
                return 'broad_reach'
            if state.compass_heading is not None:
                if self.patrol_mode == 'tacking':
                    target_heading = self._calculate_target_heading_tack(state)
                else:
                    target_heading = self._calculate_target_heading_jibe(state)
                if target_heading is not None:
                    heading_error = abs(signed_angle_difference_degrees(target_heading, state.compass_heading))
                    if heading_error < 10.0 and abs(distance) > self.boundary_turn_threshold:
                        return 'broad_reach'
        if abs(distance) < self.boundary_turn_threshold:
            speed_ms = state.gps_sog * 0.514444 if state.gps_sog is not None else 1.0
            crossing_time = self.geofence_manager.predict_boundary_crossing_time(
                state.current_latitude, state.current_longitude,
                state.compass_heading, speed_ms, self.patrol_lookahead_time)
            if crossing_time is not None and crossing_time < self.patrol_lookahead_time:
                wind_direction_abs_from_north_compass = self._calculate_absolute_wind_direction(state)
                if wind_direction_abs_from_north_compass is not None and state.compass_heading is not None:
                    relative_wind_angle = abs(signed_angle_difference_degrees(
                        wind_direction_abs_from_north_compass, state.compass_heading))
                    if relative_wind_angle > 80.0:
                        if self.logger:
                            self.logger.info(f"Approaching boundary: sailing downwind (wind_angle={relative_wind_angle:.1f}°) → JIBING")
                        return 'jibing'
                    else:
                        if self.logger:
                            self.logger.info(f"Approaching boundary: sailing upwind (wind_angle={relative_wind_angle:.1f}°) → TACKING")
                        return 'tacking'
                else:
                    return 'tacking'
        if distance < -5.0:
            wind_direction_abs_from_north_compass = self._calculate_absolute_wind_direction(state)
            if wind_direction_abs_from_north_compass is not None:
                return 'tacking'
        return 'broad_reach'

    def _calculate_target_heading_broad_reach(self, state: BoatState):
        if state.wind_angle is None or state.compass_heading is None:
            return None
        wind_dir = self._calculate_absolute_wind_direction(state)
        if wind_dir is None:
            return None
        current_time = time.time()
        if (state.target_heading is not None and
            self._stable_broad_reach_target is None and
            abs(signed_angle_difference_degrees(state.target_heading, state.compass_heading)) < 90.0):
            self._stable_broad_reach_target = state.target_heading
            self._broad_reach_target_lock_time = current_time
            self._last_absolute_wind = wind_dir
            if self.logger:
                self.logger.info(
                    f"Using human control handoff target heading: {state.target_heading:.1f}° "
                    f"(will maintain this until lock expires or wind changes)"
                )
            return self._stable_broad_reach_target
        should_recalculate = False
        if self._stable_broad_reach_target is None:
            should_recalculate = True
        elif (current_time - self._broad_reach_target_lock_time) > self._broad_reach_target_lock_duration:
            should_recalculate = True
        elif self._last_absolute_wind is not None and abs(wind_dir - self._last_absolute_wind) > 15.0:
            should_recalculate = True
        if should_recalculate:
            downwind_direction = (wind_dir + 180.0) % 360.0
            heading1 = (wind_dir + self.broad_reach_angle) % 360.0
            heading2 = (wind_dir - self.broad_reach_angle) % 360.0
            angle1_to_downwind = abs(signed_angle_difference_degrees(heading1, downwind_direction))
            angle2_to_downwind = abs(signed_angle_difference_degrees(heading2, downwind_direction))
            err1 = abs(signed_angle_difference_degrees(heading1, state.compass_heading))
            err2 = abs(signed_angle_difference_degrees(heading2, state.compass_heading))
            if angle1_to_downwind < 90.0 and angle2_to_downwind < 90.0:
                old_target = self._stable_broad_reach_target
                self._stable_broad_reach_target = heading1 if err1 < err2 else heading2
            elif angle1_to_downwind < 90.0:
                old_target = self._stable_broad_reach_target
                self._stable_broad_reach_target = heading1
            elif angle2_to_downwind < 90.0:
                old_target = self._stable_broad_reach_target
                self._stable_broad_reach_target = heading2
            else:
                old_target = self._stable_broad_reach_target
                self._stable_broad_reach_target = heading1 if err1 < err2 else heading2
                if self.logger:
                    self.logger.warn(
                        f"Broad reach target may be upwind: heading={self._stable_broad_reach_target:.1f}°, "
                        f"wind={wind_dir:.1f}°, downwind={downwind_direction:.1f}°"
                    )
            self._broad_reach_target_lock_time = current_time
            self._last_absolute_wind = wind_dir
            if old_target is not None and abs(signed_angle_difference_degrees(self._stable_broad_reach_target, old_target)) > 5.0:
                if self.logger:
                    self.logger.info(
                        f"Broad reach target heading changed: {old_target:.1f}° -> {self._stable_broad_reach_target:.1f}° "
                        f"(wind={wind_dir:.1f}°)"
                    )
        return self._stable_broad_reach_target

    def _calculate_target_heading_tack(self, state: BoatState):
        if state.compass_heading is None:
            return None
        if self._stable_tacking_target is not None:
            return self._stable_tacking_target
        wind_dir = self._calculate_absolute_wind_direction(state)
        if wind_dir is None:
            self._stable_tacking_target = (state.compass_heading + self.tack_angle) % 360.0
            return self._stable_tacking_target
        relative_wind_angle = signed_angle_difference_degrees(wind_dir, state.compass_heading)
        if relative_wind_angle > 0:
            self._stable_tacking_target = (state.compass_heading + self.tack_angle) % 360.0
        else:
            self._stable_tacking_target = (state.compass_heading - self.tack_angle) % 360.0
        return self._stable_tacking_target

    def _calculate_target_heading_jibe(self, state: BoatState):
        if state.compass_heading is None:
            return None
        if self._stable_jibing_target is not None:
            return self._stable_jibing_target
        wind_dir = self._calculate_absolute_wind_direction(state)
        if wind_dir is None:
            self._stable_jibing_target = (state.compass_heading + self.tack_angle) % 360.0
            return self._stable_jibing_target
        relative_wind_angle = signed_angle_difference_degrees(wind_dir, state.compass_heading)
        if relative_wind_angle > 0:
            self._stable_jibing_target = (state.compass_heading - self.tack_angle) % 360.0
        else:
            self._stable_jibing_target = (state.compass_heading + self.tack_angle) % 360.0
        return self._stable_jibing_target

    def generate_control(self, state: BoatState) -> ControlCommand:
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)
        if self.geofence_manager.polygon_xy is not None:
            state.geofence_polygon = self.geofence_manager.polygon_xy
        if state.current_latitude is not None and state.current_longitude is not None:
            state.distance_to_boundary = self.geofence_manager.distance_to_boundary_lonlat(
                state.current_longitude, state.current_latitude)
            if self.parent_node and hasattr(self.parent_node, 'pub_geofence_distance'):
                distance_msg = Float64(data=state.distance_to_boundary if state.distance_to_boundary is not None else 0.0)
                safe_publish(self.parent_node.pub_geofence_distance, distance_msg, self.parent_node)
                is_violation = state.distance_to_boundary is not None and state.distance_to_boundary > 0
                violation_msg = Bool(data=is_violation)
                safe_publish(self.parent_node.pub_geofence_violation, violation_msg, self.parent_node)
                is_grounded = state.distance_to_boundary is not None and state.distance_to_boundary >= 1.0
                grounding_msg = Bool(data=is_grounded)
                safe_publish(self.parent_node.pub_grounding, grounding_msg, self.parent_node)
            current_time = time.time()
            if (current_time - self.last_geofence_log_time) >= self.geofence_log_interval:
                is_inside = state.distance_to_boundary is not None and state.distance_to_boundary < 0
                distance_abs = abs(state.distance_to_boundary) if state.distance_to_boundary is not None else None
                should_log = False
                if distance_abs != self.last_reported_distance or is_inside != self.last_reported_inside:
                    should_log = True
                elif distance_abs is not None and distance_abs < self.boundary_turn_threshold * 2:
                    should_log = True
                elif not is_inside:
                    should_log = True
                elif (current_time - self.last_geofence_log_time) >= 10.0:
                    should_log = True
                if should_log and distance_abs is not None:
                    status = "INSIDE" if is_inside else "OUTSIDE ⚠️"
                    if not is_inside:
                        log_msg = (f"Geofence violation: {distance_abs:.1f}m outside boundary | "
                                   f"Position: ({state.current_latitude:.6f}, {state.current_longitude:.6f})")
                        self.log_entry(log_msg, level="WARN")
                    elif distance_abs < self.boundary_turn_threshold:
                        log_msg = f"Approaching boundary: {distance_abs:.1f}m from edge"
                        self.log_entry(log_msg, level="INFO")
                    else:
                        log_msg = f"Geofence status: {status} | Distance to boundary: {distance_abs:.1f}m"
                        self.log_entry(log_msg, level="INFO")
                    self.last_geofence_log_time = current_time
                    self.last_reported_distance = distance_abs
                    self.last_reported_inside = is_inside
            # NOTE: GPS SOG updates at ~0.2 Hz (every 5 seconds), so may be stale.
            # Controller uses position-based speed estimation as fallback (updates at 1 Hz).
            if state.gps_sog is not None and state.compass_heading is not None:
                speed_ms = state.gps_sog * 0.514444
                state.predicted_boundary_crossing_time = self.geofence_manager.predict_boundary_crossing_time(
                    state.current_latitude, state.current_longitude,
                    state.compass_heading, speed_ms, self.patrol_lookahead_time)
        new_mode = self._determine_sailing_mode(state)
        if new_mode != self.patrol_mode:
            old_mode = self.patrol_mode
            self.patrol_mode = new_mode
            self.last_turn_time = time.time()
            if new_mode == 'tacking':
                self._stable_tacking_target = None
            elif new_mode == 'jibing':
                self._stable_jibing_target = None
            elif old_mode in ['tacking', 'jibing'] and new_mode == 'broad_reach':
                self._stable_broad_reach_target = None
            self.publish_state(new_mode)
            mode_descriptions = {
                'broad_reach': 'Broad reach sailing',
                'tacking': 'Executing tack',
                'jibing': 'Executing jibe',
                'upwind_recovery': 'Recovering upwind'
            }
            description = mode_descriptions.get(new_mode, new_mode)
            context_parts = []
            if state.distance_to_boundary is not None:
                dist_abs = abs(state.distance_to_boundary)
                if dist_abs < self.boundary_turn_threshold:
                    context_parts.append(f"{dist_abs:.1f}m from boundary")
            if state.wind_angle is not None:
                if new_mode == 'tacking':
                    if state.wind_angle > 180:
                        context_parts.append("starboard tack")
                    else:
                        context_parts.append("port tack")
                elif new_mode == 'jibing':
                    if state.wind_angle > 180:
                        context_parts.append("port jibe")
                    else:
                        context_parts.append("starboard jibe")
            context = f" ({', '.join(context_parts)})" if context_parts else ""
            self.log_entry(f"{description}{context}", level="INFO")
        if self.patrol_mode == 'broad_reach':
            target_heading = self._calculate_target_heading_broad_reach(state)
        elif self.patrol_mode == 'tacking':
            target_heading = self._calculate_target_heading_tack(state)
        elif self.patrol_mode == 'jibing':
            target_heading = self._calculate_target_heading_jibe(state)
        else:
            target_heading = state.compass_heading
        if target_heading is None:
            target_heading = state.compass_heading if state.compass_heading is not None else 0.0
        state.target_heading = target_heading
        if state.compass_heading is not None:
            compass_err = signed_angle_difference_degrees(target_heading, state.compass_heading)
            cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
            cmd_rudder = float(np.clip(cmd_rudder, -1.0, 1.0))
        else:
            cmd_rudder = 0.0
            compass_err = 0.0
        cmd_sail = 0.0
        sail_reason = "no wind data"
        if state.wind_angle is not None:
            wind_sail_cmd = (state.wind_angle - 90.0) / 90.0
            wind_sail_cmd = float(np.clip(wind_sail_cmd, -1.0, 1.0))
            cmd_sail = self.sail_wind_gain * wind_sail_cmd
            sail_reason = f"wind_angle={state.wind_angle:.1f}° (wind-based control, gain={self.sail_wind_gain:.2f})"
        if state.human_controlled:
            return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)
        current_time = time.time()
        distance_abs = abs(state.distance_to_boundary) if state.distance_to_boundary is not None else None
        should_log = False
        time_since_last_log = current_time - self._last_captains_log_time
        if self._last_captains_log_time == 0.0:
            should_log = True
        elif time_since_last_log >= 1.0:
            should_log = True
        if not should_log:
            rudder_changed = (self._last_logged_rudder is None or abs(cmd_rudder - self._last_logged_rudder) > 0.05)
            sail_changed = (self._last_logged_sail is None or abs(cmd_sail - self._last_logged_sail) > 0.05)
            mode_changed = (self._last_logged_mode != self.patrol_mode)
            target_changed = (self._last_logged_target_heading is None or abs(target_heading - self._last_logged_target_heading) > 5.0)
            distance_changed = (self._last_logged_distance is None or (distance_abs is not None and abs(distance_abs - self._last_logged_distance) > 2.0))
            if rudder_changed or sail_changed or mode_changed or target_changed or distance_changed:
                should_log = True
        if should_log:
            log_parts = []
            # Always include controller name to avoid confusion
            log_parts.append(f"Controller: {self.name}")
            if self.patrol_mode == 'broad_reach':
                mode_reason = f"maintaining broad reach ({self.broad_reach_angle:.0f}° to wind)"
            elif self.patrol_mode == 'tacking':
                mode_reason = "tacking (turn in progress)"
            elif self.patrol_mode == 'jibing':
                mode_reason = "jibing (downwind turn)"
            else:
                mode_reason = f"mode: {self.patrol_mode}"
            log_parts.append(f"Mode: {self.patrol_mode} ({mode_reason})")
            if state.compass_heading is not None:
                heading_info = f"heading: current={state.compass_heading:.1f}°, target={target_heading:.1f}°"
                log_parts.append(f"Heading: {heading_info}")
            if state.compass_heading is not None:
                rudder_reason = f"rudder={cmd_rudder:+.3f}"
                if abs(cmd_rudder) > 0.01:
                    direction = "port" if cmd_rudder < 0 else "starboard"
                    raw_rudder = self.rudder_gain * (-compass_err / self.rudder_full_scale_deg)
                    if abs(raw_rudder) > 1.0:
                        rudder_reason += f" ({direction}, heading_error={compass_err:+.1f}° (target-current), gain={self.rudder_gain:.1f}, CLIPPED from {raw_rudder:+.3f})"
                    else:
                        rudder_reason += f" ({direction}, heading_error={compass_err:+.1f}° (target-current), gain={self.rudder_gain:.1f})"
                else:
                    rudder_reason += " (neutral, on target)"
                log_parts.append(f"Rudder: {rudder_reason}")
            else:
                log_parts.append(f"Rudder: 0.000 (no heading data)")
            log_parts.append(f"Sail: {cmd_sail:+.3f} ({sail_reason})")
            if distance_abs is not None:
                status = "INSIDE" if state.distance_to_boundary < 0 else "OUTSIDE"
                boundary_line = f"Boundary: {distance_abs:.1f}m {status}"
                if (state.predicted_boundary_crossing_time is not None and 
                    state.predicted_boundary_crossing_time < self.patrol_lookahead_time and
                    abs(state.distance_to_boundary) < self.boundary_turn_threshold * 2):
                    boundary_line += f" | Predicted crossing in {state.predicted_boundary_crossing_time:.1f}s"
                log_parts.append(boundary_line)
            log_message = "\n  ".join(log_parts)
            self.log_entry(log_message, level="INFO")
            self._last_captains_log_time = current_time
            self._last_logged_rudder = cmd_rudder
            self._last_logged_sail = cmd_sail
            self._last_logged_mode = self.patrol_mode
            self._last_logged_target_heading = target_heading
            self._last_logged_distance = distance_abs
        self.log_periodic_state(state, interval_sec=30.0)
        return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)


