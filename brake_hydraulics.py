"""Two master cylinders with fixed, pressure-dependent branch valves.

Wheel order is FL, FR, RL, RR. This quasi-static module calculates pressure
and unsaturated tire-force demand; it does not certify component suitability.
"""

from dataclasses import dataclass
import math


def _nonnegative(value, name):
    if not math.isfinite(value) or value < 0:
        raise ValueError(f'{name} must be finite and nonnegative.')


@dataclass(frozen=True)
class ProportioningValve:
    knee_pressure_pa: float
    post_knee_slope: float

    def __post_init__(self):
        _nonnegative(self.knee_pressure_pa, 'knee_pressure_pa')
        if not math.isfinite(self.post_knee_slope) or not 0 <= self.post_knee_slope <= 1:
            raise ValueError('post_knee_slope must be between 0 and 1.')

    def outlet_pressure_pa(self, inlet_pressure_pa):
        """Continuous knee curve; same static curve on application/release.

        Tilton 98-1261 supports 1:1 below the knee and incremental slope 1/3
        above it. Supply the actual component curve/setting explicitly.
        Hysteresis, compliance, flow transients, and release lag are unknown.
        """
        _nonnegative(inlet_pressure_pa, 'inlet_pressure_pa')
        if inlet_pressure_pa <= self.knee_pressure_pa:
            return inlet_pressure_pa
        return self.knee_pressure_pa + self.post_knee_slope * (inlet_pressure_pa - self.knee_pressure_pa)


@dataclass(frozen=True)
class HydraulicParameters:
    front_master_cylinder_area_m2: float
    rear_master_cylinder_area_m2: float
    wheel_force_gain_n_per_pa: tuple[float, float, float, float]

    def __post_init__(self):
        for name in ('front_master_cylinder_area_m2', 'rear_master_cylinder_area_m2'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive.')
        if len(self.wheel_force_gain_n_per_pa) != 4:
            raise ValueError('Supply four FL, FR, RL, RR pressure-to-tire-force gains.')
        for value in self.wheel_force_gain_n_per_pa:
            _nonnegative(value, 'wheel_force_gain_n_per_pa')


@dataclass(frozen=True)
class HydraulicConfiguration:
    valves: tuple[ProportioningValve | None, ...] = (None, None, None, None)
    wheel_faults: tuple[str, ...] = ('normal', 'normal', 'normal', 'normal')
    lost_circuit: str | None = None

    def __post_init__(self):
        if len(self.valves) != 4 or any(valve is not None and not isinstance(valve, ProportioningValve) for valve in self.valves):
            raise ValueError('valves must have four ProportioningValve or None entries.')
        if len(self.wheel_faults) != 4 or any(fault not in ('normal', 'blocked', 'bypassed', 'retained') for fault in self.wheel_faults):
            raise ValueError('Faults must be four normal, blocked, bypassed, or retained entries.')
        if self.lost_circuit not in (None, 'front', 'rear'):
            raise ValueError('lost_circuit must be front, rear, or None.')


@dataclass(frozen=True)
class HydraulicResult:
    front_master_pressure_pa: float
    rear_master_pressure_pa: float
    wheel_pressures_pa: tuple[float, ...]
    wheel_braking_force_demands_n: tuple[float, ...]
    front_master_force_fraction: float

    @property
    def front_braking_fraction(self):
        total = sum(self.wheel_braking_force_demands_n)
        return sum(self.wheel_braking_force_demands_n[:2]) / total if total > 0 else None


def calculate_hydraulic_braking(
    parameters, pushrod_force_n, front_master_force_fraction,
    configuration=None, previous_wheel_pressures_pa=(0.0, 0.0, 0.0, 0.0),
):
    """Map balance-bar force split through two masters and four branches.

    pushrod_force_n is force at the bar, after pedal leverage. Pressure gains
    must include caliper geometry, pad friction, effective rotor radius and
    tire radius (or calibration). They are required inputs, not guessed.

    Fault abstractions: blocked means no delivery from a depressurized branch;
    retained means pressure cannot release below its previous value; bypassed
    removes proportioning. A lost circuit sets both axle outputs to zero.
    Retention can exceed CURRENT inlet pressure by retaining earlier pressure;
    it is not pressure amplification. These are failure probes, not fluid-flow
    or leak-rate models, and do not establish real failure behavior.
    """
    _nonnegative(pushrod_force_n, 'pushrod_force_n')
    if not math.isfinite(front_master_force_fraction) or not 0 <= front_master_force_fraction <= 1:
        raise ValueError('front_master_force_fraction must be between 0 and 1.')
    if len(previous_wheel_pressures_pa) != 4:
        raise ValueError('Supply four previous wheel pressures.')
    for value in previous_wheel_pressures_pa:
        _nonnegative(value, 'previous_wheel_pressure_pa')
    configuration = configuration or HydraulicConfiguration()
    front = pushrod_force_n * front_master_force_fraction / parameters.front_master_cylinder_area_m2
    rear = pushrod_force_n * (1 - front_master_force_fraction) / parameters.rear_master_cylinder_area_m2
    if configuration.lost_circuit == 'front':
        front = 0.0
    elif configuration.lost_circuit == 'rear':
        rear = 0.0
    pressures = []
    for index, inlet in enumerate((front, front, rear, rear)):
        valve = configuration.valves[index]
        fault = configuration.wheel_faults[index]
        pressure = inlet if valve is None or fault == 'bypassed' else valve.outlet_pressure_pa(inlet)
        if fault == 'blocked':
            pressure = 0.0
        elif fault == 'retained':
            pressure = max(pressure, previous_wheel_pressures_pa[index])
        if configuration.lost_circuit == ('front' if index < 2 else 'rear'):
            pressure = 0.0
        pressures.append(pressure)
    forces = tuple(pressure * gain for pressure, gain in zip(pressures, parameters.wheel_force_gain_n_per_pa))
    return HydraulicResult(front, rear, tuple(pressures), forces, front_master_force_fraction)


def solve_master_force_fraction(parameters, pushrod_force_n, target_front_braking_fraction, configuration=None):
    """Find the bar-force split giving a desired tire-force fraction at ONE load.

    This is a setting/calibration calculation, not automatic passive hardware
    compensation. Repeating it online would require an active front/rear
    adjuster with separately modeled limits. Fault configurations are excluded.
    """
    _nonnegative(pushrod_force_n, 'pushrod_force_n')
    if pushrod_force_n == 0:
        raise ValueError('A force fraction is undefined at zero braking demand.')
    if not math.isfinite(target_front_braking_fraction) or not 0 <= target_front_braking_fraction <= 1:
        raise ValueError('target_front_braking_fraction must be between 0 and 1.')
    configuration = configuration or HydraulicConfiguration()
    if configuration.lost_circuit is not None or any(fault != 'normal' for fault in configuration.wheel_faults):
        raise ValueError('Bias compensation requires a fault-free configuration.')
    # Positive effective gain at both endpoints makes the continuous fraction
    # monotone over the interval and avoids claiming an all-zero solution.
    if calculate_hydraulic_braking(parameters, pushrod_force_n, 1, configuration).front_braking_fraction != 1:
        raise ValueError('Front axle cannot produce the requested braking allocation.')
    if calculate_hydraulic_braking(parameters, pushrod_force_n, 0, configuration).front_braking_fraction != 0:
        raise ValueError('Rear axle cannot produce the requested braking allocation.')
    lower, upper = 0.0, 1.0
    for _ in range(60):
        candidate = (lower + upper) / 2
        fraction = calculate_hydraulic_braking(parameters, pushrod_force_n, candidate, configuration).front_braking_fraction
        if fraction is None:
            raise ValueError('No pressure-supported braking allocation is available.')
        if fraction < target_front_braking_fraction:
            lower = candidate
        else:
            upper = candidate
    return (lower + upper) / 2
