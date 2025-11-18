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
from typing import Optional

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
    tack_in_progress: bool = False
    tack_start_sign: int = 0
    tack_time_elapsed_s: float = 0.0
    last_wind_boat_angle: Optional[float] = None  # previous apparent wind angle (degrees)


class MockSailboatSimulator:
    """Mock simulator for testing when sailboat-playground is not available."""

    def __init__(self, dt=0.1, max_turn_rate=None, max_speed=None, no_go_angle_deg=None,
                 stall_decay_rate=None, stall_recovery_threshold=None,
                 tack_entry_buffer_deg=None, tack_exit_buffer_deg=None,
                 tack_speed_threshold=None, tack_min_speed_to_continue=None,
                 tack_time_limit_s=None, tack_turn_boost=None):
        """Initialize mock simulator.
        
        Args:
            dt: Time step in seconds per integration step (default: 0.1 = 10 Hz)
                Should match the simulation rate from the bridge (1.0 / simulation_rate)
            max_turn_rate: Degrees per second at full rudder and speed (default: 30.0)
            max_speed: Metres per second, approximate hull speed (default: 1.5)
            no_go_angle_deg: Approximate close-hauled limit in degrees (default: 40.0)
            stall_decay_rate: m/s^2 equivalent deceleration while stalled (default: 0.5)
            stall_recovery_threshold: m/s; once above this we treat stall as cleared (default: 0.1)
            tack_entry_buffer_deg: Allow tacks to begin slightly before the no-go boundary (default: 12.0)
            tack_exit_buffer_deg: Required clearance before finishing a tack (default: 6.0)
            tack_speed_threshold: m/s minimum speed to initiate a tack (default: 0.35)
            tack_min_speed_to_continue: m/s minimum to keep crossing the wind (default: 0.25)
            tack_time_limit_s: Abort tack if it takes longer than this (default: 8.0)
            tack_turn_boost: Multiplier applied to turn rate during a tack window (default: 1.8)
        """
        self.state = MockSimulatorState()

        # Physics parameters (use provided values or defaults)
        self.dt = float(dt)  # seconds per integration step
        self.max_turn_rate = float(max_turn_rate) if max_turn_rate is not None else 30.0
        self.max_speed = float(max_speed) if max_speed is not None else 1.5
        self.no_go_angle_deg = float(no_go_angle_deg) if no_go_angle_deg is not None else 40.0
        self.stall_decay_rate = float(stall_decay_rate) if stall_decay_rate is not None else 0.5
        self.stall_recovery_threshold = float(stall_recovery_threshold) if stall_recovery_threshold is not None else 0.1
        self.tack_entry_buffer_deg = float(tack_entry_buffer_deg) if tack_entry_buffer_deg is not None else 12.0
        self.tack_exit_buffer_deg = float(tack_exit_buffer_deg) if tack_exit_buffer_deg is not None else 6.0
        self.tack_speed_threshold = float(tack_speed_threshold) if tack_speed_threshold is not None else 0.35
        self.tack_min_speed_to_continue = float(tack_min_speed_to_continue) if tack_min_speed_to_continue is not None else 0.25
        self.tack_time_limit_s = float(tack_time_limit_s) if tack_time_limit_s is not None else 8.0
        self.tack_turn_boost = float(tack_turn_boost) if tack_turn_boost is not None else 1.8
        self.debug_tack = False # TODO: disable after tacking is working

    def set_debug_tack_logging(self, enabled: bool) -> None:
        """Enable or disable verbose logging for tack detection."""
        self.debug_tack = bool(enabled)

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
        # Calculate apparent wind angle relative to boat heading. The simulator stores
        # wind_direction as the heading the wind is BLOWING TOWARD (trigonometric
        # convention, 0° = East). For sail trim we need the direction the wind is
        # COMING FROM relative to the bow, so add 180° before computing the offset.
        wind_from_direction = (self.state.wind_direction + 180.0) % 360.0
        wind_boat_angle = (wind_from_direction - self.state.boat_heading) % 360.0
        if wind_boat_angle > 180.0:
            wind_boat_angle -= 360.0

        previous_angle = (
            self.state.last_wind_boat_angle
            if self.state.last_wind_boat_angle is not None
            else wind_boat_angle
        )
        previous_sign = 1 if previous_angle >= 0.0 else -1
        current_sign = 1 if wind_boat_angle >= 0.0 else -1

        # Detect if we are attempting to sail inside the "no-go" zone
        abs_apparent = abs(wind_boat_angle)
        is_in_no_go_zone = abs_apparent < self.no_go_angle_deg

        tack_candidate = (
            abs(self.state.rudder_angle) > 0.25
            and self.state.boat_speed > self.tack_speed_threshold
            and abs_apparent < (self.no_go_angle_deg + self.tack_entry_buffer_deg)
        )

        if self.debug_tack and abs_apparent <= (self.no_go_angle_deg + self.tack_entry_buffer_deg + 5.0):
            print(
                "[MockSim] tack check | "
                f"abs_angle={abs_apparent:.1f} deg | "
                f"speed={self.state.boat_speed:.2f} m/s | "
                f"rudder={self.state.rudder_angle:.2f} | "
                f"candidate={tack_candidate} | "
                f"in_progress={self.state.tack_in_progress}"
            )

        if not self.state.tack_in_progress and tack_candidate:
            self.state.tack_in_progress = True
            self.state.tack_start_sign = current_sign
            self.state.tack_time_elapsed_s = 0.0
            self.state.stalled = False
            if self.debug_tack:
                print(
                    "[MockSim] tack initiated | "
                    f"heading={self.state.boat_heading:.1f} deg | "
                    f"wind_angle={wind_boat_angle:.1f} deg | "
                    f"speed={self.state.boat_speed:.2f} m/s"
                )

        in_tack_window = False
        effective_no_go = is_in_no_go_zone

        if self.state.tack_in_progress:
            self.state.tack_time_elapsed_s += self.dt
            in_tack_window = True
            if (
                self.state.boat_speed < self.tack_min_speed_to_continue
                or self.state.tack_time_elapsed_s > self.tack_time_limit_s
            ):
                # Tack failed – fall back to stall behaviour
                self.state.tack_in_progress = False
                in_tack_window = False
                effective_no_go = True
                self.state.stalled = True
                if self.debug_tack:
                    reason = (
                        "low_speed"
                        if self.state.boat_speed < self.tack_min_speed_to_continue
                        else "timeout"
                    )
                    print(
                        "[MockSim] tack failed | "
                        f"reason={reason} | "
                        f"time={self.state.tack_time_elapsed_s:.2f}s | "
                        f"speed={self.state.boat_speed:.2f} m/s"
                    )
            else:
                crossing_complete = (
                    abs_apparent >= (self.no_go_angle_deg + self.tack_exit_buffer_deg)
                    and current_sign != self.state.tack_start_sign
                )
                sign_flip_without_clearance = (
                    current_sign != self.state.tack_start_sign and abs_apparent >= self.no_go_angle_deg
                )
                if crossing_complete or (sign_flip_without_clearance and previous_sign != current_sign):
                    # Successful tack: crossed the wind and cleared the no-go zone
                    self.state.tack_in_progress = False
                    in_tack_window = False
                    effective_no_go = False
                    self.state.stalled = False
                    if self.debug_tack:
                        print(
                            "[MockSim] tack complete | "
                            f"heading={self.state.boat_heading:.1f} deg | "
                            f"wind_angle={wind_boat_angle:.1f} deg | "
                            f"time={self.state.tack_time_elapsed_s:.2f}s"
                        )
                else:
                    effective_no_go = False

        self.state.last_wind_boat_angle = wind_boat_angle

        # Estimate speed based on wind alignment and sail trim
        if effective_no_go:
            # Deep stall: drift slowly downwind
            target_speed = 0.1
            self.state.stalled = True
            if self.debug_tack and not in_tack_window:
                print(
                    "[MockSim] stall enforced | "
                    f"heading={self.state.boat_heading:.1f} deg | "
                    f"wind_angle={wind_boat_angle:.1f} deg"
                )
        else:
            abs_sin = abs(math.sin(math.radians(wind_boat_angle)))
            wind_efficiency = 0.5 + 0.5 * abs_sin  # 0.5 dead downwind, 1.0 beam reach
            sail_efficiency = 1.0 - abs(self.state.sail_angle * 0.3)
            target_speed = self.state.wind_speed * 0.5 * wind_efficiency * sail_efficiency
            target_speed = min(target_speed, self.max_speed)
            # Recover from stall once moving again
            if self.state.stalled and self.state.boat_speed <= self.stall_recovery_threshold:
                self.state.stalled = False

        # Smooth speed changes towards target
        speed_diff = target_speed - self.state.boat_speed
        if effective_no_go:
            adjustment = np.clip(speed_diff * 1.5 * self.dt,
                                 -self.stall_decay_rate * self.dt,
                                 self.stall_decay_rate * self.dt)
        else:
            adjustment = np.clip(speed_diff * 2.0 * self.dt, -1.0, 1.0)
        self.state.boat_speed += float(adjustment)
        self.state.boat_speed = max(0.0, self.state.boat_speed)

        # Ensure stall flag clears once boat resumes sailing on a valid tack
        if not effective_no_go and self.state.stalled and self.state.boat_speed > self.stall_recovery_threshold:
            self.state.stalled = False

        # Apply rudder-induced turn rate when moving
        if effective_no_go:
            # When in no-go zone, force boat to fall off to downwind side
            # Calculate which direction is downwind (away from wind)
            # Wind is coming from wind_from_direction, so downwind is opposite
            downwind_direction = (wind_from_direction + 180.0) % 360.0
            # Calculate angle from current heading to downwind direction
            heading_to_downwind = (downwind_direction - self.state.boat_heading) % 360.0
            if heading_to_downwind > 180.0:
                heading_to_downwind -= 360.0
            
            # Turn toward downwind direction (fall off)
            if abs(heading_to_downwind) < 5.0:
                # Already pointing downwind, just maintain heading
                turn_rate = 0.0
            else:
                # Turn toward downwind
                turn_sign = 1.0 if heading_to_downwind > 0.0 else -1.0
                turn_rate = self.max_turn_rate * 0.6  # More aggressive fall-off
            self.state.boat_heading = (self.state.boat_heading + turn_sign * turn_rate * self.dt) % 360.0
        elif self.state.boat_speed >= 0.05:
            speed_ratio = self.state.boat_speed / self.max_speed if self.max_speed > 0 else 0.0
            effective_ratio = max(0.2, min(1.0, speed_ratio))
            turn_rate = -self.state.rudder_angle * self.max_turn_rate * effective_ratio
            if in_tack_window:
                turn_rate *= self.tack_turn_boost
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
            "tack_in_progress": self.state.tack_in_progress,
        }
