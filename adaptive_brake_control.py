"""Sensor-driven force-allocation benchmark; requires active wheel control.

Driver front/rear preference is an axle tire-force fraction. The separate
left/right strength scales the capacity-matched differential on each axle.
This is not a passive-valve controller or a vehicle stability proof.
"""

from dataclasses import dataclass
import math

from four_wheel_model import (
    BrakingAllocation, InfeasibleOperatingPoint,
    calculate_four_wheel_grip, evaluate_braking_allocation,
)


def _finite(value, name, minimum=None, maximum=None):
    if not math.isfinite(value) or (minimum is not None and value < minimum) or (maximum is not None and value > maximum):
        raise ValueError(f'{name} is outside its finite allowed range.')


@dataclass(frozen=True)
class AdaptiveBrakeSettings:
    strength_fraction: float = 0.0
    modulated_axles: str = 'both'
    maximum_asymmetry_fraction: float = 1.0
    maximum_yaw_moment_nm: float | None = None
    maximum_asymmetry_change_per_s: float | None = None
    lateral_filter_time_constant_s: float = 0.0
    lateral_deadband_mps2: float = 0.0
    steering_deadband_rad: float = 0.0
    minimum_speed_mps: float = 0.0
    longitudinal_capacity_margin_fraction: float = 0.0

    def __post_init__(self):
        for name in ('strength_fraction', 'maximum_asymmetry_fraction', 'longitudinal_capacity_margin_fraction'):
            _finite(getattr(self, name), name, 0, 1)
        for name in ('lateral_filter_time_constant_s', 'lateral_deadband_mps2', 'steering_deadband_rad', 'minimum_speed_mps'):
            _finite(getattr(self, name), name, 0)
        for name in ('maximum_yaw_moment_nm', 'maximum_asymmetry_change_per_s'):
            if getattr(self, name) is not None:
                _finite(getattr(self, name), name, 0)
        if self.modulated_axles not in ('front', 'rear', 'both'):
            raise ValueError('modulated_axles must be front, rear, or both.')


@dataclass(frozen=True)
class BrakeControlRequest:
    deceleration_demand_mps2: float
    front_braking_fraction: float
    forward_speed_mps: float
    lateral_acceleration_mps2: float
    steering_angle_rad: float

    def __post_init__(self):
        _finite(self.deceleration_demand_mps2, 'deceleration_demand_mps2', 0)
        _finite(self.front_braking_fraction, 'front_braking_fraction', 0, 1)
        _finite(self.forward_speed_mps, 'forward_speed_mps', 0)
        _finite(self.lateral_acceleration_mps2, 'lateral_acceleration_mps2')
        _finite(self.steering_angle_rad, 'steering_angle_rad')


@dataclass(frozen=True)
class BrakeControlState:
    filtered_lateral_acceleration_mps2: float = 0.0
    front_asymmetry_fraction: float = 0.0
    rear_asymmetry_fraction: float = 0.0

    def __post_init__(self):
        _finite(self.filtered_lateral_acceleration_mps2, 'filtered_lateral_acceleration_mps2')
        _finite(self.front_asymmetry_fraction, 'front_asymmetry_fraction', -1, 1)
        _finite(self.rear_asymmetry_fraction, 'rear_asymmetry_fraction', -1, 1)


@dataclass(frozen=True)
class BrakeControlResult:
    allocation: BrakingAllocation | None
    state: BrakeControlState
    peak_asymmetry_fractions: tuple[float, float]
    applied_asymmetry_fractions: tuple[float, float]
    yaw_acceleration_radps2: float | None
    lateral_force_reserves_n: tuple[float, ...]
    limiting_reasons: tuple[str, ...]
    authority_limit_overrode_slew: bool
    front_braking_fraction_requested: float
    meets_capacity_and_supply_limits: bool

    @property
    def peak_tire_utilization(self):
        if self.allocation is None:
            return None
        values = []
        for wheel, force in zip(self.allocation.wheels, self.allocation.braking_forces_n):
            magnitude = math.hypot(force, wheel.lateral_force_n)
            values.append(magnitude / wheel.friction_capacity_n if wheel.friction_capacity_n > 0 else (0 if magnitude == 0 else math.inf))
        return max(values)


def calculate_adaptive_braking(
    parameters, request, settings, previous_state=None, *, time_step_s,
    wheel_force_supply_limits_n=None,
    wheel_grip_override=None,
):
    """One control step using current signals, with no track/preview argument.

    q = (B_right-B_left)/(B_right+B_left). Full-strength q_peak matches the
    remaining longitudinal capacities on an axle, minimizing their maximum
    demand/capacity ratio at fixed axle force. It is not a lap-time optimum.
    q_command = strength*q_peak; front/rear axle force totals are preserved.

    A margin, yaw cap, straight-line/countersteering gate, and fraction slew
    limit constrain assistance. Hard authority/yaw limits take precedence
    over slew, and any such override is reported. Infeasible demands are
    reported, never repaired by secretly changing front/rear preference.
    Supply limits are optional per-wheel force ceilings from another model.
    """
    _finite(time_step_s, 'time_step_s', 0)
    if time_step_s == 0:
        raise ValueError('time_step_s must be positive.')
    if wheel_force_supply_limits_n is not None:
        if len(wheel_force_supply_limits_n) != 4:
            raise ValueError('Supply limits require four FL, FR, RL, RR values.')
        for value in wheel_force_supply_limits_n:
            _finite(value, 'wheel_force_supply_limit_n', 0)
    previous = previous_state or BrakeControlState()
    tau = settings.lateral_filter_time_constant_s
    blend = 1.0 if tau == 0 else -math.expm1(-time_step_s / tau)
    filtered_lateral = previous.filtered_lateral_acceleration_mps2 + blend * (
        request.lateral_acceleration_mps2 - previous.filtered_lateral_acceleration_mps2
    )
    try:
        # Raw current lateral demand is retained in grip checks; filtering
        # must not hide an actual force-limit violation during a transition.
        if wheel_grip_override is not None:
            if len(wheel_grip_override) != 4:
                raise ValueError('wheel_grip_override requires four FL, FR, RL, RR grip records.')
            wheels = tuple(wheel_grip_override)
        else:
            wheels = calculate_four_wheel_grip(parameters, request.deceleration_demand_mps2, request.lateral_acceleration_mps2)
    except InfeasibleOperatingPoint as error:
        return BrakeControlResult(
            None, BrakeControlState(filtered_lateral), (0, 0), (0, 0), None, (),
            (str(error),), bool(previous.front_asymmetry_fraction or previous.rear_asymmetry_fraction),
            request.front_braking_fraction, False,
        )
    capacities = [wheel.maximum_braking_force_n * (1 - settings.longitudinal_capacity_margin_fraction) for wheel in wheels]
    if wheel_force_supply_limits_n is not None:
        capacities = [min(grip, supply) for grip, supply in zip(capacities, wheel_force_supply_limits_n)]
    peaks = tuple(
        (capacities[i + 1] - capacities[i]) / (capacities[i] + capacities[i + 1])
        if capacities[i] + capacities[i + 1] > 0 else 0.0
        for i in (0, 2)
    )
    total_force = parameters.vehicle.vehicle_mass_kg * request.deceleration_demand_mps2
    axle_forces = (total_force * request.front_braking_fraction, total_force * (1 - request.front_braking_fraction))
    reasons = []
    cornering = (
        abs(filtered_lateral) > settings.lateral_deadband_mps2
        and abs(request.lateral_acceleration_mps2) > settings.lateral_deadband_mps2
        and abs(request.steering_angle_rad) > settings.steering_deadband_rad
        and request.steering_angle_rad * request.lateral_acceleration_mps2 > 0
        and request.forward_speed_mps > settings.minimum_speed_mps
    )
    if not cornering:
        reasons.append('Assistance gated off: straight line, low speed, deadband, or countersteering.')
    previous_q = (previous.front_asymmetry_fraction, previous.rear_asymmetry_fraction)
    applied = []
    step_limit = None if settings.maximum_asymmetry_change_per_s is None else settings.maximum_asymmetry_change_per_s * time_step_s
    for index, axle in enumerate(('front', 'rear')):
        enabled = cornering and settings.modulated_axles in (axle, 'both') and axle_forces[index] > 0
        target = settings.strength_fraction * peaks[index] if enabled else 0.0
        target = max(-settings.maximum_asymmetry_fraction, min(settings.maximum_asymmetry_fraction, target))
        candidate = target
        if step_limit is not None:
            candidate = max(previous_q[index] - step_limit, min(previous_q[index] + step_limit, target))
        # Strength is a hard fraction-of-peak bound, including when the
        # driver reduces authority or the corner direction changes.
        candidate = max(min(0.0, target), min(max(0.0, target), candidate))
        applied.append(candidate)
    braking_yaw = -sum(track * force * q / 2 for track, force, q in zip(
        (parameters.front_track_m, parameters.rear_track_m), axle_forces, applied
    ))
    if settings.maximum_yaw_moment_nm is not None and abs(braking_yaw) > settings.maximum_yaw_moment_nm:
        yaw_scale = settings.maximum_yaw_moment_nm / abs(braking_yaw)
        applied = [q * yaw_scale for q in applied]
        reasons.append('Yaw-moment cap reduced left/right assistance.')
    overrode_slew = step_limit is not None and any(abs(q - old) > step_limit + 1e-12 for q, old in zip(applied, previous_q))
    if overrode_slew:
        reasons.append('Hard authority/yaw boundary overrode the fraction slew limit.')
    forces = tuple(force * (1 + side * q) / 2 for force, q in zip(axle_forces, applied) for side in (-1, 1))
    allocation = evaluate_braking_allocation(wheels, forces)
    meets_limits = all(force <= capacity + 1e-7 for force, capacity in zip(forces, capacities))
    if not meets_limits:
        reasons.append('Grip, margin, or supply limit exceeded: lower braking demand at the requested front/rear split.')
    lateral_reserves = tuple(
        math.sqrt(max(0.0, wheel.friction_capacity_n**2 - force**2)) - abs(wheel.lateral_force_n)
        for wheel, force in zip(wheels, forces)
    )
    next_state = BrakeControlState(filtered_lateral, *applied)
    return BrakeControlResult(
        allocation, next_state, peaks, tuple(applied),
        allocation.yaw_moment_nm / parameters.yaw_moment_of_inertia_kgm2,
        lateral_reserves, tuple(reasons), overrode_slew,
        request.front_braking_fraction, meets_limits,
    )
