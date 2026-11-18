"""Quasi-static, straight-line braking with a prescribed tire-force split.

Deceleration and braking-force demands are positive magnitudes. This model
predicts the onset of tire saturation, not wheel motion after locking.
"""

from dataclasses import dataclass
import math

from vehicle_model import calculate_axle_loads_with_longitudinal_transfer


GRAVITY_MPS2 = 9.81


@dataclass(frozen=True)
class BrakingParameters:
    """Illustrative vehicle values; replace with measurements from the car."""

    vehicle_mass_kg: float = 300.0
    center_of_gravity_to_front_axle_m: float = 1.0
    center_of_gravity_to_rear_axle_m: float = 0.8
    center_of_gravity_height_m: float = 0.30
    friction_coefficient: float = 1.5

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite.")
            minimum_is_allowed = name in (
                "center_of_gravity_height_m", "friction_coefficient"
            )
            if value < 0 or (value == 0 and not minimum_is_allowed):
                raise ValueError(f"{name} must be {'nonnegative' if minimum_is_allowed else 'positive'}.")

    @property
    def wheelbase_m(self):
        return self.center_of_gravity_to_front_axle_m + self.center_of_gravity_to_rear_axle_m

    @property
    def rear_lift_deceleration_mps2(self):
        if self.center_of_gravity_height_m == 0:
            return math.inf
        return GRAVITY_MPS2 * self.center_of_gravity_to_front_axle_m / self.center_of_gravity_height_m


@dataclass(frozen=True)
class BrakingState:
    deceleration_mps2: float
    front_braking_fraction: float
    front_vertical_load_n: float
    rear_vertical_load_n: float
    front_braking_force_demand_n: float
    rear_braking_force_demand_n: float
    front_grip_capacity_n: float
    rear_grip_capacity_n: float
    front_tire_utilization: float
    rear_tire_utilization: float
    is_feasible: bool


@dataclass(frozen=True)
class BrakingLimit:
    maximum_deceleration_mps2: float
    limiting_constraint: str
    state: BrakingState


def _validate_fraction(front_braking_fraction):
    if not math.isfinite(front_braking_fraction) or not 0 <= front_braking_fraction <= 1:
        raise ValueError("front_braking_fraction must be finite and between 0 and 1.")


def _calculate_supported_axle_loads(parameters, deceleration_mps2):
    if not math.isfinite(deceleration_mps2) or deceleration_mps2 < 0:
        raise ValueError("deceleration_mps2 must be finite and nonnegative.")
    if deceleration_mps2 > parameters.rear_lift_deceleration_mps2:
        raise ValueError("Deceleration exceeds rear axle lift; the four-wheel-contact model is invalid.")
    return calculate_axle_loads_with_longitudinal_transfer(
        parameters.vehicle_mass_kg,
        parameters.center_of_gravity_to_front_axle_m,
        parameters.center_of_gravity_to_rear_axle_m,
        parameters.center_of_gravity_height_m,
        -deceleration_mps2,
    )


def calculate_ideal_front_braking_fraction(parameters, deceleration_mps2):
    """Return front tire-force fraction for equal axle utilization.

    Assumes the same friction coefficient at both axles and no cornering,
    aero, gradient, or brake-torque limits. Does not assert that the requested
    deceleration is achievable, and does not return a balance-bar setting.
    At zero deceleration this is the limiting ideal split as demand rises.
    """
    front_load_n, rear_load_n = _calculate_supported_axle_loads(parameters, deceleration_mps2)
    return front_load_n / (front_load_n + rear_load_n)


def calculate_straight_line_braking_state(parameters, deceleration_mps2, front_braking_fraction):
    """Evaluate requested force and grip at each axle, without clipping demand.

    Utilization is demand / capacity: 1 is saturation, >1 is infeasible.
    Returned demands are not achievable forces if is_feasible is False.
    """
    _validate_fraction(front_braking_fraction)
    front_load_n, rear_load_n = _calculate_supported_axle_loads(parameters, deceleration_mps2)
    total_demand_n = parameters.vehicle_mass_kg * deceleration_mps2
    front_demand_n = front_braking_fraction * total_demand_n
    rear_demand_n = (1 - front_braking_fraction) * total_demand_n
    front_capacity_n = parameters.friction_coefficient * front_load_n
    rear_capacity_n = parameters.friction_coefficient * rear_load_n

    def utilization(demand_n, capacity_n):
        if capacity_n == 0:
            return 0.0 if demand_n == 0 else math.inf
        return demand_n / capacity_n

    front_utilization = utilization(front_demand_n, front_capacity_n)
    rear_utilization = utilization(rear_demand_n, rear_capacity_n)
    return BrakingState(
        deceleration_mps2, front_braking_fraction, front_load_n, rear_load_n,
        front_demand_n, rear_demand_n, front_capacity_n, rear_capacity_n,
        front_utilization, rear_utilization,
        max(front_utilization, rear_utilization) <= 1 + 1e-10,
    )


def find_maximum_deceleration_for_brake_bias(parameters, front_braking_fraction):
    """Solve both axle grip inequalities analytically for a fixed force split.

    Stops at first saturation or rear axle lift. It does not redistribute
    braking after an axle saturates and does not simulate ABS.
    """
    _validate_fraction(front_braking_fraction)
    friction = parameters.friction_coefficient
    front_denominator_m = (
        front_braking_fraction * parameters.wheelbase_m
        - friction * parameters.center_of_gravity_height_m
    )
    rear_denominator_m = (
        (1 - front_braking_fraction) * parameters.wheelbase_m
        + friction * parameters.center_of_gravity_height_m
    )
    front_limit_mps2 = (
        friction * GRAVITY_MPS2 * parameters.center_of_gravity_to_rear_axle_m / front_denominator_m
        if front_denominator_m > 0 else math.inf
    )
    rear_limit_mps2 = (
        friction * GRAVITY_MPS2 * parameters.center_of_gravity_to_front_axle_m / rear_denominator_m
        if rear_denominator_m > 0 else math.inf
    )
    limits = {
        "front tire saturation": front_limit_mps2,
        "rear tire saturation": rear_limit_mps2,
        "rear axle lift": parameters.rear_lift_deceleration_mps2,
    }
    maximum_deceleration_mps2 = min(limits.values())
    limiting_constraints = [
        name for name, limit in limits.items()
        if math.isclose(limit, maximum_deceleration_mps2, rel_tol=1e-9, abs_tol=1e-10)
    ]
    return BrakingLimit(
        maximum_deceleration_mps2,
        " + ".join(limiting_constraints),
        calculate_straight_line_braking_state(parameters, maximum_deceleration_mps2, front_braking_fraction),
    )
