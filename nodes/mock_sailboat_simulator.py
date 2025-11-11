#!/usr/bin/env python3
"""
Lightweight fallback sailboat simulator used when sailboat-playground is unavailable.

This module keeps the mock simulator in a dedicated file so the behaviour can be
iterated independently of the ROS2 bridge. The physics model is intentionally
simple: it tracks planar position, heading, boat speed, and applies rudder and
sail inputs with crude wind interaction to approximate basic sailing dynamics.
"""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class MockSimulatorState:
    """Container for mock simulator state variables."""

    boat_x: float = 0.0  # metres
    boat_y: float = 0.0  # metres
    boat_heading: float = 180.0  # degrees, simulator convention (0° = East)
    boat_speed: float = 1.0  # metres / second
    rudder_angle: float = 0.0  # normalised [-1, 1]
    sail_angle: float = 0.0  # normalised [-1, 1]
    wind_speed: float = 1.0  # metres / second
    wind_direction: float = 0.0  # degrees (direction wind is blowing toward)
    stalled: bool = False  # indicates whether the boat is stalled (in irons)


class MockSailboatSimulator:
    """Mock simulator for testing when sailboat-playground is not available."""

    def __init__(self):
        self.state = MockSimulatorState()

        # Physics parameters
        self.dt = 0.1  # seconds per integration step
        self.max_turn_rate = 30.0  # degrees per second at full rudder and speed
        self.max_speed = 1.5  # metres / second (approximate hull speed)
        self.no_go_angle_deg = 40.0  # approximate close-hauled limit
        self.stall_decay_rate = 2.5  # m/s^2 equivalent deceleration while stalled
        self.stall_recovery_threshold = 0.1  # m/s; once above this we treat stall as cleared

    # ------------------------------------------------------------------
    # Compatibility properties (legacy direct attribute access)
    # ------------------------------------------------------------------
    @property
    def boat_x(self) -> float:
        return self.state.boat_x

    @boat_x.setter
    def boat_x(self, value: float) -> None:
        self.state.boat_x = float(value)

    @property
    def boat_y(self) -> float:
        return self.state.boat_y

    @boat_y.setter
    def boat_y(self, value: float) -> None:
        self.state.boat_y = float(value)

    @property
    def boat_heading(self) -> float:
        return self.state.boat_heading

    @boat_heading.setter
    def boat_heading(self, value: float) -> None:
        self.state.boat_heading = float(value) % 360.0

    @property
    def boat_speed(self) -> float:
        return self.state.boat_speed

    @boat_speed.setter
    def boat_speed(self, value: float) -> None:
        self.state.boat_speed = max(0.05, float(value))

    @property
    def rudder_angle(self) -> float:
        return self.state.rudder_angle

    @rudder_angle.setter
    def rudder_angle(self, value: float) -> None:
        self.state.rudder_angle = float(np.clip(value, -1.0, 1.0))

    @property
    def sail_angle(self) -> float:
        return self.state.sail_angle

    @sail_angle.setter
    def sail_angle(self, value: float) -> None:
        self.state.sail_angle = float(np.clip(value, -1.0, 1.0))

    @property
    def wind_speed(self) -> float:
        return self.state.wind_speed

    @wind_speed.setter
    def wind_speed(self, value: float) -> None:
        self.state.wind_speed = max(0.05, float(value))

    @property
    def wind_direction(self) -> float:
        return self.state.wind_direction

    @wind_direction.setter
    def wind_direction(self, value: float) -> None:
        self.state.wind_direction = float(value) % 360.0

    def set_control(self, rudder: float, sail: float) -> None:
        """Set rudder and sail positions (-1 to +1)."""
        self.state.rudder_angle = float(np.clip(rudder, -1.0, 1.0))
        self.state.sail_angle = float(np.clip(sail, -1.0, 1.0))

    def step(self) -> dict:
        """Advance simulation by a single time step and return the new state."""
        # Calculate apparent wind angle relative to boat heading
        wind_boat_angle = (self.state.wind_direction - self.state.boat_heading) % 360.0
        if wind_boat_angle > 180.0:
            wind_boat_angle -= 360.0

        # Detect if we are attempting to sail inside the "no-go" zone
        abs_apparent = abs(wind_boat_angle)
        is_in_no_go_zone = abs_apparent < self.no_go_angle_deg

        # Estimate speed based on wind alignment and sail trim
        if is_in_no_go_zone:
            # Deep stall: boat luffs head-to-wind and rapidly loses way
            target_speed = 0.01
            self.state.stalled = True

            # Apply an aggressive decay to current speed to simulate "in irons"
            stall_delta = self.stall_decay_rate * self.dt
            self.state.boat_speed = max(0.05, self.state.boat_speed - stall_delta)
        else:
            wind_efficiency = max(0.1, abs(math.sin(math.radians(wind_boat_angle))))
            sail_efficiency = 1.0 - abs(self.state.sail_angle * 0.3)
            target_speed = self.state.wind_speed * 0.5 * wind_efficiency * sail_efficiency
            target_speed = min(target_speed, self.max_speed)
            # Recover from stall once moving again
            if self.state.stalled and self.state.boat_speed <= self.stall_recovery_threshold:
                self.state.stalled = False

        # Smooth speed changes towards target
        speed_diff = target_speed - self.state.boat_speed
        self.state.boat_speed += float(np.clip(speed_diff * 2.0 * self.dt, -1.0, 1.0))
        self.state.boat_speed = max(0.0, self.state.boat_speed)

        # Ensure stall flag clears once boat resumes sailing on a valid tack
        if not is_in_no_go_zone and self.state.stalled and self.state.boat_speed > self.stall_recovery_threshold:
            self.state.stalled = False

        # Apply rudder-induced turn rate when moving
        if self.state.boat_speed >= 0.05:
            speed_ratio = self.state.boat_speed / self.max_speed if self.max_speed > 0 else 0.0
            effective_ratio = max(0.2, min(1.0, speed_ratio))
            turn_rate = self.state.rudder_angle * self.max_turn_rate * effective_ratio
            self.state.boat_heading = (self.state.boat_heading + turn_rate * self.dt) % 360.0

        # Update position in simulator frame (0° = East)
        heading_rad = math.radians(self.state.boat_heading)
        self.state.boat_x += self.state.boat_speed * math.cos(heading_rad) * self.dt
        self.state.boat_y += self.state.boat_speed * math.sin(heading_rad) * self.dt

        return {
            "x": self.state.boat_x,
            "y": self.state.boat_y,
            "heading": self.state.boat_heading,
            "speed": self.state.boat_speed,
            "wind_speed": self.state.wind_speed,
            "wind_direction": wind_boat_angle,  # relative to boat
            "rudder": self.state.rudder_angle,
            "sail": self.state.sail_angle,
            "stalled": self.state.stalled,
        }
