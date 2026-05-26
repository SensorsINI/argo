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


class WindAwareController(BaseController):
    """Enhanced controller that considers wind conditions."""

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        self.name = 'WindAwareController'
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)
        self.sail_wind_gain = config.get('sail_wind_gain', 0.5)

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate control commands considering wind conditions."""
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)

        # Publish controller state
        self.publish_state('wind_aware')

        # Heading control (same as proportional)
        compass_err = signed_angle_difference_degrees(
            state.target_heading, state.compass_heading)
        cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = self.clip_rudder(cmd_rudder)

        # Wind-aware sail control
        cmd_sail = state.radio_sail if state.radio_sail is not None else 0.0
        if state.wind_angle is not None:
            # Simple sail control based on wind angle
            wind_sail_cmd = (state.wind_angle - 90.0) / 90.0
            wind_sail_cmd = float(np.clip(wind_sail_cmd, -1.0, 1.0))
            # Blend radio command with wind-based command
            cmd_sail = (1 - self.sail_wind_gain) * cmd_sail + self.sail_wind_gain * wind_sail_cmd

        return ControlCommand(
            rudder=cmd_rudder,
            sail=cmd_sail,
            timestamp=state.timestamp
        )


