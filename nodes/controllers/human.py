#!/usr/bin/env python3

# 1) Standard library
import os
import sys

# 2) Third-party
# (none needed)

# 3) Path modifications
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'support'))

# 4) Local imports
from .base import BaseController, BoatState, ControlCommand


class HumanController(BaseController):
    """Human controller that passes through keyboard/radio commands.
    
    This controller does not generate any autonomous control commands.
    Instead, it passes through control commands from keyboard/radio input
    (state.radio_rudder and state.radio_sail). Useful for studying simulator
    dynamics with full manual control.
    
    Behavior:
    - Always passes through radio/keyboard input when available
    - If radio is lost (None or zero), returns neutral (0.0) which may trigger RTH
    - If radio turns back on, immediately resumes passing through radio commands
    """

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        if logger:
            logger.info("HumanController initialized - passing through keyboard/radio commands only")

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Pass through keyboard/radio commands (no autonomous control)."""
        # Publish controller state
        self.publish_state('human')
        
        # Always pass through radio/keyboard commands directly
        # If radio is None (not received), use 0.0 (neutral)
        # If radio is 0.0 (center position or lost), use 0.0 (neutral)
        # This allows RTH to trigger when radio is lost (handled elsewhere)
        cmd_rudder = state.radio_rudder if state.radio_rudder is not None else 0.0
        cmd_sail = state.radio_sail if state.radio_sail is not None else 0.0
        
        return ControlCommand(
            rudder=cmd_rudder,
            sail=cmd_sail,
            timestamp=state.timestamp
        )
