"""Four-wheel quasi-steady path equilibrium for comparative lap estimates.

The environment supplies path curvature; the brake controller receives only
current-state signals. Unknowns are actuation, sideslip and front steering.
This is not a transient driver, yaw-stability, or wheel-lockup simulation.
"""

from dataclasses import dataclass, field
from functools import lru_cache
import math

import numpy as np

from adaptive_brake_control import AdaptiveBrakeSettings, BrakeControlRequest, calculate_adaptive_braking
from brake_hydraulics import HydraulicConfiguration, HydraulicParameters, calculate_hydraulic_braking
from brake_model import GRAVITY_MPS2
from four_wheel_model import FourWheelParameters, InfeasibleOperatingPoint, WheelGrip, calculate_four_wheel_contacts
from tire_model import calculate_lateral_tire_force
from fiddle_brake import FiddleBrakeSettings, auxiliary_braking
from vehicle_model import transform_tire_forces_to_vehicle_coordinates


@dataclass(frozen=True)
class PerformanceParameters:
    wheels: FourWheelParameters = field(default_factory=FourWheelParameters)
    axle_cornering_stiffness_n_per_rad: float = 20000.0
    maximum_steering_angle_rad: float = math.radians(15)
    maximum_sideslip_angle_rad: float = math.radians(10)
    drive_power_w: float = 40000.0
    maximum_drive_force_n: float = 3000.0
    maximum_speed_mps: float = 25.0
    drag_force_per_speed_squared: float = 0.0
    downforce_per_speed_squared_n_s2_per_m2: float = 0.0
    aerodynamic_front_fraction: float = .5
    rolling_resistance_force_n: float = 0.0
    body_width_m: float = .76
    front_overhang_m: float = .48
    rear_overhang_m: float = .24
    tire_plan_width_m: float = .22
    tire_plan_length_m: float = .46
    equilibrium_tolerance: float = 2e-6

    def __post_init__(self):
        positive = ('axle_cornering_stiffness_n_per_rad', 'maximum_steering_angle_rad', 'maximum_sideslip_angle_rad', 'drive_power_w', 'maximum_drive_force_n', 'maximum_speed_mps', 'body_width_m', 'tire_plan_width_m', 'tire_plan_length_m', 'equilibrium_tolerance')
        nonnegative = ('drag_force_per_speed_squared', 'downforce_per_speed_squared_n_s2_per_m2', 'rolling_resistance_force_n', 'front_overhang_m', 'rear_overhang_m')
        for name in positive + nonnegative:
            value = getattr(self, name)
            if not math.isfinite(value) or value < 0 or (name in positive and value == 0):
                raise ValueError(f'{name} must be finite and {"positive" if name in positive else "nonnegative"}.')
        if max(self.maximum_steering_angle_rad, self.maximum_sideslip_angle_rad) >= math.pi / 2:
            raise ValueError('Steering and sideslip limits must be below 90 degrees.')
        if not math.isfinite(self.aerodynamic_front_fraction) or not 0 <= self.aerodynamic_front_fraction <= 1:
            raise ValueError('Aerodynamic front fraction must be in [0, 1].')


@dataclass(frozen=True)
class BrakeStrategy:
    name: str
    kind: str = 'conventional'
    front_braking_fraction: float = .6
    assistance: AdaptiveBrakeSettings = field(default_factory=AdaptiveBrakeSettings)
    hydraulics: HydraulicParameters | None = None
    configuration: HydraulicConfiguration = field(default_factory=HydraulicConfiguration)
    front_master_force_fraction: float = .6
    maximum_pushrod_force_n: float = 4000.0
    fiddle: FiddleBrakeSettings = field(default_factory=FiddleBrakeSettings)

    def __post_init__(self):
        if self.kind not in ('conventional', 'passive', 'adaptive', 'fiddle'):
            raise ValueError('Unknown brake strategy.')
        for fraction in (self.front_braking_fraction, self.front_master_force_fraction):
            if not math.isfinite(fraction) or not 0 <= fraction <= 1:
                raise ValueError('Brake fractions must be in [0, 1].')
        if not math.isfinite(self.maximum_pushrod_force_n) or self.maximum_pushrod_force_n <= 0:
            raise ValueError('maximum_pushrod_force_n must be positive.')
        if self.kind == 'passive' and self.hydraulics is None:
            raise ValueError('Passive comparisons require explicit hydraulic component inputs.')
        if self.kind == 'fiddle' and (self.hydraulics is None or any(self.configuration.valves) or self.configuration.lost_circuit is not None or any(fault != 'normal' for fault in self.configuration.wheel_faults)):
            raise ValueError('Fiddle-brake comparisons require normal hydraulic branches and explicit supply inputs.')
        if self.kind == 'conventional' and any(self.configuration.valves):
            raise ValueError('Use passive kind for a branch-valve configuration.')
        if 'retained' in self.configuration.wheel_faults:
            raise ValueError('Pressure retention is stateful; use the timed hydraulic study, not a quasi-steady lap envelope.')
        if self.kind == 'adaptive' and (self.assistance.maximum_asymmetry_change_per_s is not None or self.assistance.lateral_filter_time_constant_s != 0):
            raise ValueError('Quasi-steady lap assistance must be memoryless. Rate/filter effects require the timed model.')


@dataclass(frozen=True)
class OperatingPoint:
    is_feasible: bool
    reason: str
    speed_mps: float
    tangential_acceleration_mps2: float
    curvature_per_m: float
    steering_angle_rad: float = 0.0
    sideslip_angle_rad: float = 0.0
    yaw_rate_radps: float = 0.0
    yaw_acceleration_radps2: float = 0.0
    yaw_moment_nm: float = 0.0
    brake_yaw_moment_nm: float = 0.0
    peak_tire_utilization: float = math.inf
    equilibrium_error: float = math.inf
    tire_longitudinal_forces_n: tuple[float, ...] = ()
    tire_lateral_forces_n: tuple[float, ...] = ()
    wheel_loads_n: tuple[float, ...] = ()
    wheel_pressures_pa: tuple[float, ...] = ()
    pushrod_force_n: float = 0.0
    drive_force_n: float = 0.0
    body_longitudinal_acceleration_mps2: float = 0.0
    body_lateral_acceleration_mps2: float = 0.0
    force_balance_errors_n: tuple[float, float] = (math.inf, math.inf)
    yaw_balance_error_nm: float = math.inf
    auxiliary_braking_forces_n: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0)


def _road_footprint_fits(parameters, sideslip, steering, curvature, width):
    """Check body and tire outline corners against a local circular corridor."""
    if width is None:
        return True
    vehicle = parameters.wheels.vehicle
    points = [(x, y) for x in (vehicle.center_of_gravity_to_front_axle_m + parameters.front_overhang_m, -vehicle.center_of_gravity_to_rear_axle_m - parameters.rear_overhang_m) for y in (-parameters.body_width_m / 2, parameters.body_width_m / 2)]
    for x, track, angle in ((vehicle.center_of_gravity_to_front_axle_m, parameters.wheels.front_track_m, steering), (-vehicle.center_of_gravity_to_rear_axle_m, parameters.wheels.rear_track_m, 0)):
        for y in (-track / 2, track / 2):
            for dx in (-parameters.tire_plan_length_m / 2, parameters.tire_plan_length_m / 2):
                for dy in (-parameters.tire_plan_width_m / 2, parameters.tire_plan_width_m / 2):
                    points.append((x + dx * math.cos(angle) - dy * math.sin(angle), y + dx * math.sin(angle) + dy * math.cos(angle)))
    for x, y in points:
        path_x = x * math.cos(sideslip) + y * math.sin(sideslip)
        path_y = -x * math.sin(sideslip) + y * math.cos(sideslip)
        if abs(curvature) < 1e-9:
            offset = path_y
        else:
            radius = 1 / curvature
            # Rational form avoids subtracting two large radii near straights.
            distance = math.hypot(path_x, path_y - radius)
            offset = (2 * radius * path_y - path_x**2 - path_y**2) / (abs(radius) + distance)
        if abs(offset) > width / 2 + 1e-8:
            return False
    return True


def solve_operating_point(parameters, strategy, speed_mps, tangential_acceleration_mps2, curvature_per_m, curvature_gradient_per_m2=0.0, road_width_m=None):
    """Solve all body force equations and Mz = Iz*(a*kappa+v²*kappa').

    Linear tire stiffness is distributed across each axle by normal load,
    and combined-force saturation holds requested longitudinal force fixed.
    Four local wheel velocities include yaw and lateral position. Both front
    wheels use one steering angle (Ackermann and suspension geometry omitted).
    Sideslip is locally constant; transient sideslip/roll and actuator dynamics
    are outside this quasi-steady performance model.
    """
    values = (speed_mps, tangential_acceleration_mps2, curvature_per_m, curvature_gradient_per_m2)
    if not all(math.isfinite(value) for value in values) or speed_mps <= 0:
        raise ValueError('Operating-point inputs must be finite and speed positive.')
    if road_width_m is not None and (not math.isfinite(road_width_m) or road_width_m <= 0):
        raise ValueError('Road width must be positive.')
    preferred_mode = 'drive' if tangential_acceleration_mps2 >= 0 else 'brake'
    values = (parameters, speed_mps, tangential_acceleration_mps2, curvature_per_m, curvature_gradient_per_m2, road_width_m)
    def solve_branch(mode):
        # An auxiliary brake can operate under throttle: its drive equilibrium
        # is strategy-dependent and must never use the shared drive-only cache.
        if mode == 'drive' and strategy.kind != 'fiddle':
            return _solve_drive(*values)
        return _solve_mode(parameters, strategy, *values[1:], mode)
    point = solve_branch(preferred_mode)
    # Tangential acceleration alone does not identify the actuator branch:
    # cornering losses can need drive during light deceleration, while changing
    # yaw motion can require a tiny brake input at constant path speed. Check
    # the other nonnegative-actuation branch before rejecting the point.
    if not point.is_feasible:
        alternate = 'brake' if preferred_mode == 'drive' else 'drive'
        fallback = solve_branch(alternate)
        if fallback.is_feasible:
            return fallback
    return point


_DRIVE_STRATEGY = BrakeStrategy('Drive-only equilibrium')


@lru_cache(maxsize=32768)
def _solve_drive(parameters, speed, acceleration, curvature, gradient, road_width):
    """Reuse identical propulsion equilibria across brake configurations.

    In drive mode no brake settings or hydraulic state enter any force,
    bound, initial guess, or returned quantity. Brake-mode fallbacks always
    use the actual strategy. Keys retain exact floats and every car parameter;
    no rounding, reduced sampling, or weaker feasibility checks are used.
    """
    return _solve_mode(parameters, _DRIVE_STRATEGY, speed, acceleration, curvature, gradient, road_width, 'drive')


def _solve_mode(parameters, strategy, speed, acceleration, curvature, gradient, road_width, mode):
    vehicle = parameters.wheels.vehicle
    mass, wheelbase = vehicle.vehicle_mass_kg, vehicle.wheelbase_m
    force_scale = mass * GRAVITY_MPS2
    moment_scale = force_scale * wheelbase
    yaw_rate = speed * curvature
    yaw_acceleration = acceleration * curvature + speed**2 * gradient
    hydraulic_mode = mode == 'brake' and strategy.hydraulics is not None and strategy.kind != 'adaptive'
    if strategy.hydraulics is not None:
        supply_result = calculate_hydraulic_braking(strategy.hydraulics, strategy.maximum_pushrod_force_n, strategy.front_master_force_fraction)
        supply_limits = supply_result.wheel_braking_force_demands_n
    else:
        supply_limits = None

    def evaluate(unknowns):
        amplitude = unknowns[0] * force_scale
        beta, steering = unknowns[1], unknowns[2]
        forward = speed * math.cos(beta)
        lateral = speed * math.sin(beta)
        body_ax = acceleration * math.cos(beta) - speed**2 * curvature * math.sin(beta)
        body_ay = acceleration * math.sin(beta) + speed**2 * curvature * math.cos(beta)
        contacts = calculate_four_wheel_contacts(parameters.wheels, body_ax, body_ay,
                    downforce_n=parameters.downforce_per_speed_squared_n_s2_per_m2 * speed**2,
                    aerodynamic_front_fraction=parameters.aerodynamic_front_fraction)
        pressures = ()
        bar_force = 0.0
        controller_ok = True
        if mode == 'drive':
            longitudinal = (0.0, 0.0, amplitude / 2, amplitude / 2)
        elif hydraulic_mode:
            hydraulics = calculate_hydraulic_braking(strategy.hydraulics, amplitude, strategy.front_master_force_fraction, strategy.configuration)
            longitudinal = tuple(-force for force in hydraulics.wheel_braking_force_demands_n)
            pressures = hydraulics.wheel_pressures_pa
            bar_force = amplitude
        elif strategy.kind == 'adaptive':
            # Current-signal load/force observer, deliberately without path
            # curvature, future geometry, or a perfect tire-state oracle.
            observer = []
            for index, contact in enumerate(contacts):
                axle = contacts[:2] if index < 2 else contacts[2:]
                axle_load = sum(item.vertical_load_n for item in axle)
                share = vehicle.center_of_gravity_to_rear_axle_m / wheelbase if index < 2 else vehicle.center_of_gravity_to_front_axle_m / wheelbase
                fy = mass * body_ay * share * contact.vertical_load_n / axle_load if axle_load > 0 else 0.0
                capacity = vehicle.friction_coefficient * contact.vertical_load_n
                observer.append(WheelGrip(contact.name, contact.x_m, contact.y_m, contact.vertical_load_n, fy, capacity, math.sqrt(max(0, capacity**2 - fy**2))))
            request = BrakeControlRequest(amplitude / mass, strategy.front_braking_fraction, speed, body_ay, steering)
            control = calculate_adaptive_braking(parameters.wheels, request, strategy.assistance, time_step_s=1.0, wheel_force_supply_limits_n=supply_limits, wheel_grip_override=observer)
            longitudinal = tuple(-force for force in control.allocation.braking_forces_n)
            # The observer chooses the command, but the vehicle's solved
            # tire forces decide physical feasibility. Applying the observer's
            # provisional lateral split as a second grip constraint would
            # unfairly penalize even zero assistance versus the baseline.
            controller_ok = supply_limits is None or all(abs(force) <= limit + 1e-7 for force, limit in zip(longitudinal, supply_limits))
        else:
            front_force = amplitude * strategy.front_braking_fraction
            rear_force = amplitude - front_force
            longitudinal = (-front_force / 2, -front_force / 2, -rear_force / 2, -rear_force / 2)

        if mode == 'brake' and strategy.kind == 'adaptive' and supply_limits is not None:
            fractions = [abs(force) / limit if limit > 0 else (0 if force == 0 else math.inf) for force, limit in zip(longitudinal, supply_limits)]
            bar_force = max(fractions) * strategy.maximum_pushrod_force_n
            pressures = tuple(abs(force) / gain if gain > 0 else (0 if force == 0 else math.inf) for force, gain in zip(longitudinal, strategy.hydraulics.wheel_force_gain_n_per_pa))

        auxiliary = (0.0, 0.0, 0.0, 0.0)
        if strategy.kind == 'fiddle':
            # Reserve from a current-state lateral-load observer, followed by
            # the same solved combined-tire and force/yaw checks as every mode.
            reserves, supplies = [], []
            for index, contact in enumerate(contacts):
                axle = contacts[:2] if index < 2 else contacts[2:]
                axle_load = sum(item.vertical_load_n for item in axle)
                share = vehicle.center_of_gravity_to_rear_axle_m / wheelbase if index < 2 else vehicle.center_of_gravity_to_front_axle_m / wheelbase
                fy = mass * body_ay * share * contact.vertical_load_n / axle_load if axle_load > 0 else 0.0
                capacity = vehicle.friction_coefficient * contact.vertical_load_n
                service = max(0.0, -longitudinal[index]) if mode == 'brake' else 0.0
                reserves.append(max(0.0, math.sqrt(max(0.0, capacity**2 - fy**2)) - service))
                supplies.append(max(0.0, supply_limits[index] - service))
            auxiliary = auxiliary_braking(strategy.fiddle, speed, body_ay, steering,
                                          parameters.wheels.rear_track_m, reserves, supplies)
            longitudinal = tuple(fx - extra for fx, extra in zip(longitudinal, auxiliary))
            if any(auxiliary):
                base_pressures = pressures or (0.0,) * 4
                pressures = tuple(pressure + (extra / gain if gain > 0 else 0.0) for pressure, extra, gain in zip(base_pressures, auxiliary, strategy.hydraulics.wheel_force_gain_n_per_pa))
            # Auxiliary actuator shares the per-wheel pressure ceiling; the
            # balance-bar pushrod force continues to describe the main pedal.
            controller_ok = all(max(0.0, -fx) <= limit + 1e-7 for fx, limit in zip(longitudinal, supply_limits))

        lateral_forces, vehicle_forces, utilizations = [], [], []
        for index, (contact, fx) in enumerate(zip(contacts, longitudinal)):
            angle = steering if index < 2 else 0.0
            local_forward = forward - yaw_rate * contact.y_m
            local_lateral = lateral + yaw_rate * contact.x_m
            if local_forward <= 0:
                raise InfeasibleOperatingPoint('A wheel has nonpositive forward velocity.')
            slip = angle - math.atan2(local_lateral, local_forward)
            axle_load = sum(item.vertical_load_n for item in (contacts[:2] if index < 2 else contacts[2:]))
            stiffness = parameters.axle_cornering_stiffness_n_per_rad * contact.vertical_load_n / axle_load if axle_load > 0 else 0.0
            pure_fy = calculate_lateral_tire_force(math.degrees(slip), contact.vertical_load_n, vehicle.friction_coefficient, stiffness)
            capacity = vehicle.friction_coefficient * contact.vertical_load_n
            remaining_lateral = math.sqrt(max(0.0, capacity**2 - fx**2))
            fy = max(-remaining_lateral, min(remaining_lateral, pure_fy))
            if mode == 'brake' and strategy.kind == 'adaptive':
                actual_remaining_braking = math.sqrt(max(0.0, capacity**2 - fy**2))
                controller_ok = controller_ok and abs(fx) <= actual_remaining_braking * (1 - strategy.assistance.longitudinal_capacity_margin_fraction) + 1e-7
            lateral_forces.append(fy)
            vehicle_forces.append(transform_tire_forces_to_vehicle_coordinates(fx, fy, math.degrees(angle)))
            utilizations.append(math.hypot(fx, fy) / capacity if capacity > 0 else (0 if fx == 0 and fy == 0 else math.inf))
        drag = parameters.drag_force_per_speed_squared * speed**2 + parameters.rolling_resistance_force_n
        fx_sum = sum(force[0] for force in vehicle_forces) - drag * math.cos(beta)
        fy_sum = sum(force[1] for force in vehicle_forces) - drag * math.sin(beta)
        moment = sum(contact.x_m * force[1] - contact.y_m * force[0] for contact, force in zip(contacts, vehicle_forces))
        residual = np.array([(fx_sum - mass * body_ax) / force_scale, (fy_sum - mass * body_ay) / force_scale, (moment - parameters.wheels.yaw_moment_of_inertia_kgm2 * yaw_acceleration) / moment_scale])
        return residual, (contacts, longitudinal, lateral_forces, utilizations, pressures, bar_force, controller_ok, body_ax, body_ay, moment, forward, vehicle_forces, auxiliary)

    estimated_lateral = mass * speed**2 * curvature
    rear_fy = estimated_lateral * vehicle.center_of_gravity_to_front_axle_m / wheelbase
    beta_seed = vehicle.center_of_gravity_to_rear_axle_m * curvature - rear_fy / parameters.axle_cornering_stiffness_n_per_rad
    steering_seed = wheelbase * curvature + estimated_lateral * (vehicle.center_of_gravity_to_rear_axle_m - vehicle.center_of_gravity_to_front_axle_m) / (wheelbase * parameters.axle_cornering_stiffness_n_per_rad)
    amplitude_seed = max(.001 * force_scale, mass * abs(acceleration))
    if hydraulic_mode:
        reference = calculate_hydraulic_braking(strategy.hydraulics, force_scale, strategy.front_master_force_fraction, strategy.configuration)
        gain = sum(reference.wheel_braking_force_demands_n) / force_scale
        amplitude_seed /= max(gain, 1e-6)
    unknowns = np.array([amplitude_seed / force_scale, beta_seed, steering_seed], dtype=float)
    bounds = np.array([20.0, parameters.maximum_sideslip_angle_rad, parameters.maximum_steering_angle_rad])

    def clamp(candidate):
        return np.clip(candidate, np.array([0.0, -bounds[1], -bounds[2]]), bounds)

    unknowns = clamp(unknowns)
    best = None
    for _ in range(24):
        try:
            residual, detail = evaluate(unknowns)
        except (InfeasibleOperatingPoint, ValueError):
            return OperatingPoint(False, 'Wheel contact or controller domain exceeded.', speed, acceleration, curvature)
        norm = float(np.max(np.abs(residual)))
        if best is None or norm < best[0]:
            best = (norm, unknowns.copy(), residual, detail)
        if norm <= parameters.equilibrium_tolerance:
            break
        columns = []
        for dimension in range(3):
            step = 1e-5
            derivatives = []
            for sign in (1, -1):
                shifted = unknowns.copy()
                shifted[dimension] += sign * step
                try:
                    shifted_residual, _ = evaluate(shifted)
                    derivatives.append((sign, shifted_residual))
                except (InfeasibleOperatingPoint, ValueError):
                    continue
            if len(derivatives) == 2:
                columns.append((derivatives[0][1] - derivatives[1][1]) / (2 * step))
            elif derivatives:
                sign, shifted_residual = derivatives[0]
                columns.append((shifted_residual - residual) / (sign * step))
            else:
                return OperatingPoint(False, 'No supported equilibrium derivative.', speed, acceleration, curvature)
        jacobian = np.column_stack(columns)
        try:
            change = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]
        except np.linalg.LinAlgError:
            break
        accepted = False
        for factor in (1.0, .5, .25, .125, .0625):
            trial = clamp(unknowns + factor * change)
            try:
                trial_residual, _ = evaluate(trial)
                if np.linalg.norm(trial_residual) < np.linalg.norm(residual):
                    unknowns = trial
                    accepted = True
                    break
            except (InfeasibleOperatingPoint, ValueError):
                continue
        if not accepted:
            break

    error, solved, residual, detail = best
    contacts, longitudinal, lateral_forces, utilizations, pressures, bar_force, controller_ok, body_ax, body_ay, moment, forward, vehicle_forces, auxiliary = detail
    reason = 'Converged equilibrium.'
    feasible = error <= parameters.equilibrium_tolerance
    if not feasible:
        reason = 'Force/yaw equilibrium did not converge within the steering/sideslip bounds.'
    elif max(utilizations) > 1 + 1e-7:
        feasible, reason = False, 'Tire longitudinal demand exceeds combined grip.'
    elif not controller_ok:
        feasible, reason = False, 'Assistance grip reserve or supply constraint exceeded.'
    elif mode == 'drive' and (solved[0] * force_scale > parameters.maximum_drive_force_n + 1e-7 or solved[0] * force_scale * forward > parameters.drive_power_w + 1e-6):
        feasible, reason = False, 'Drive force or power limit exceeded.'
    elif mode == 'brake' and bar_force > strategy.maximum_pushrod_force_n + 1e-7:
        feasible, reason = False, 'Master-cylinder supply demand exceeded.'
    elif not _road_footprint_fits(parameters, solved[1], solved[2], curvature, road_width):
        feasible, reason = False, 'Car/tire outline exceeds the local road corridor.'
    brake_yaw = sum(-contact.y_m * min(0.0, fx) for contact, fx in zip(contacts, longitudinal))
    if mode == 'drive' and strategy.kind == 'fiddle':
        brake_yaw = sum(contact.y_m * force for contact, force in zip(contacts, auxiliary))
    return OperatingPoint(
        feasible, reason, speed, acceleration, curvature, float(solved[2]), float(solved[1]),
        yaw_rate, yaw_acceleration, moment, brake_yaw, max(utilizations), error,
        tuple(longitudinal), tuple(lateral_forces), tuple(contact.vertical_load_n for contact in contacts),
        tuple(pressures), bar_force, float(solved[0] * force_scale) if mode == 'drive' else 0.0,
        body_ax, body_ay, (float(residual[0] * force_scale), float(residual[1] * force_scale)), float(residual[2] * moment_scale),
        auxiliary,
    )
