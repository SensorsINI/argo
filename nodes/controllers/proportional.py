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


class ProportionalHeadingController(BaseController):
    """Simple proportional controller for heading maintenance."""

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        self.rudder_gain = config.get('rudder_gain', 1.0)
        self.rudder_full_scale_deg = config.get('rudder_full_scale_deg', 60.0)

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate control commands using proportional heading control."""
        if not state.is_valid_for_control():
            return ControlCommand(timestamp=state.timestamp)

        # Publish controller state
        self.publish_state('proportional')

        # Calculate heading error (target - current)
        compass_err = signed_angle_difference_degrees(
            state.target_heading, state.compass_heading)

        # Proportional controller for rudder
        cmd_rudder = self.rudder_gain * (compass_err / self.rudder_full_scale_deg)
        cmd_rudder = float(np.clip(cmd_rudder, -1.0, 1.0))

        # Pass through sail command from radio (could be enhanced later)
        cmd_sail = state.radio_sail if state.radio_sail is not None else 0.0

        return ControlCommand(
            rudder=cmd_rudder,
            sail=cmd_sail,
            timestamp=state.timestamp
        )


