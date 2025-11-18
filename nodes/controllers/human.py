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
    """Human controller that does nothing - allows full manual control.
    
    This controller generates neutral commands (0, 0) so that no autonomous
    control actions are taken. Useful for studying simulator dynamics with
    full manual control via keyboard/radio input.
    """

    def __init__(self, config, logger=None, parent_node=None):
        super().__init__(config, logger=logger, parent_node=parent_node)
        if logger:
            logger.info("HumanController initialized - no autonomous control, manual control only")

    def generate_control(self, state: BoatState) -> ControlCommand:
        """Generate neutral control commands (no autonomous actions)."""
        # Publish controller state
        self.publish_state('human')
        
        # Return neutral commands - no autonomous control
        # Manual control via keyboard/radio will still work
        return ControlCommand(
            rudder=0.0,
            sail=0.0,
            timestamp=state.timestamp
        )

