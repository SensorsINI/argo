#!/usr/bin/env python3

# 1) Standard library
import os
import sys

# 2) Third-party
import numpy as np

# 3) Path modifications
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'support'))

# 4) Local imports
from .base import BaseController, BoatState, ControlCommand, signed_angle_difference_degrees


class ReturnToHomeController(BaseController):
    """
    Return-to-home controller that navigates back to starting position.
    """

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)
        self.connection_timeout = config.get('shore_connection_timeout', 120.0)  # seconds
        self.arrival_distance = config.get('arrival_distance_nm', 0.05)  # ~90m

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
        distance_to_home = state.get_distance_to_home()

        if bearing_to_home is None or distance_to_home is None:
            # Fall back to maintaining current heading
            compass_err = signed_angle_difference_degrees(
                state.target_heading, state.compass_heading)
            cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
            cmd_rudder = float(np.clip(cmd_rudder, -1.0, 1.0))
            cmd_sail = 0.0
            return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)

        if distance_to_home < self.arrival_distance:
            if not hasattr(self, '_arrival_logged') or not self._arrival_logged:
                self.log_entry(f"Arrived at home position (distance: {distance_to_home:.3f}nm)", level="INFO")
                self._arrival_logged = True
            return ControlCommand(rudder=0.0, sail=0.0, timestamp=state.timestamp)

        if hasattr(self, '_arrival_logged'):
            self._arrival_logged = False

        # Navigate toward home bearing
        state.target_heading = bearing_to_home

        compass_err = signed_angle_difference_degrees(bearing_to_home, state.compass_heading)
        cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = float(np.clip(cmd_rudder, -1.0, 1.0))

        # Wind-aware sail control
        cmd_sail = 0.0
        if state.wind_angle is not None:
            wind_sail_cmd = (state.wind_angle - 90.0) / 90.0
            wind_sail_cmd = float(np.clip(wind_sail_cmd, -1.0, 1.0))
            cmd_sail = self.sail_wind_gain * wind_sail_cmd

        return ControlCommand(rudder=cmd_rudder, sail=cmd_sail, timestamp=state.timestamp)


