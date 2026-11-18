"""Memoryless inside-rear auxiliary brake experiment, not McLaren control logic."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FiddleBrakeSettings:
    strength_fraction: float = .5
    maximum_yaw_moment_nm: float = 120.0
    minimum_speed_mps: float = 2.0
    lateral_deadband_mps2: float = .2
    steering_deadband_rad: float = .003

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative.')
        if self.strength_fraction > 1:
            raise ValueError('Fiddle-brake strength must be in [0, 1].')


def auxiliary_braking(settings, speed, lateral_acceleration, steering,
                      rear_track, remaining_capacity_n, remaining_supply_n):
    """Return positive extra brake forces in FL/FR/RL/RR order.

    Signals must agree in direction; no curvature or future-track input.
    Authority ramps over 1 g above the lateral deadband and 0.03 rad above
    the steering deadband. These are explicit experimental settings, not
    an understeer detector or a claim of optimal driver pedal operation.
    """
    signals = (speed, lateral_acceleration, steering, rear_track)
    if not all(math.isfinite(value) for value in signals) or speed < 0 or rear_track <= 0:
        raise ValueError('Invalid fiddle-brake state or rear track.')
    for limits in (remaining_capacity_n, remaining_supply_n):
        if len(limits) != 4 or any(not math.isfinite(value) or value < 0 for value in limits):
            raise ValueError('Four finite nonnegative wheel limits are required.')
    forces = [0.0] * 4
    if speed < settings.minimum_speed_mps or lateral_acceleration * steering <= 0:
        return tuple(forces)
    lateral_ramp = min(1.0, max(0.0, abs(lateral_acceleration) - settings.lateral_deadband_mps2) / 9.81)
    steering_ramp = min(1.0, max(0.0, abs(steering) - settings.steering_deadband_rad) / .03)
    wheel = 2 if lateral_acceleration > 0 else 3
    demand = settings.strength_fraction * settings.maximum_yaw_moment_nm * lateral_ramp * steering_ramp / (rear_track / 2)
    forces[wheel] = min(demand, remaining_capacity_n[wheel], remaining_supply_n[wheel])
    return tuple(forces)
