"""Four-wheel force-allocation study, not a steering or wheel-slip simulation.

Coordinates: x forward, y left, positive yaw turns left. All braking forces
are positive magnitudes; their signed vehicle-frame Fx is negative.
"""

from dataclasses import dataclass, field
import math

from brake_model import BrakingParameters, GRAVITY_MPS2


FORCE_TOLERANCE_N = 1e-7


@dataclass(frozen=True)
class FourWheelParameters:
    vehicle: BrakingParameters = field(default_factory=BrakingParameters)
    front_track_m: float = 1.20
    rear_track_m: float = 1.20
    front_roll_moment_fraction: float = 0.50
    yaw_moment_of_inertia_kgm2: float = 150.0

    def __post_init__(self):
        for name in ('front_track_m', 'rear_track_m', 'yaw_moment_of_inertia_kgm2'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive.')
        if not math.isfinite(self.front_roll_moment_fraction) or not 0 <= self.front_roll_moment_fraction <= 1:
            raise ValueError('front_roll_moment_fraction must be between 0 and 1.')


@dataclass(frozen=True)
class WheelContact:
    name: str
    x_m: float
    y_m: float
    vertical_load_n: float


@dataclass(frozen=True)
class WheelGrip:
    name: str
    x_m: float
    y_m: float
    vertical_load_n: float
    lateral_force_n: float
    friction_capacity_n: float
    maximum_braking_force_n: float


@dataclass(frozen=True)
class BrakingAllocation:
    wheels: tuple[WheelGrip, ...]
    braking_forces_n: tuple[float, ...]
    yaw_moment_nm: float
    is_feasible: bool

    @property
    def total_braking_force_n(self):
        return sum(self.braking_forces_n)

    @property
    def front_braking_fraction(self):
        if self.total_braking_force_n == 0:
            return None
        return sum(self.braking_forces_n[:2]) / self.total_braking_force_n


class InfeasibleOperatingPoint(ValueError):
    """Requested loads or lateral forces are outside this model's domain."""


def calculate_four_wheel_contacts(parameters, longitudinal_acceleration_mps2, lateral_acceleration_mps2, *, downforce_n=0.0, aerodynamic_front_fraction=.5):
    """Signed body-axis accelerations -> FL, FR, RL, RR normal loads.

    Shared by the snapshot studies and the equilibrium/lap model. Negative
    loads are rejected, retaining the original four-wheel-contact assumption.
    """
    if not all(math.isfinite(value) for value in (longitudinal_acceleration_mps2, lateral_acceleration_mps2)):
        raise ValueError('Accelerations must be finite.')
    if not math.isfinite(downforce_n) or downforce_n < 0 or not math.isfinite(aerodynamic_front_fraction) or not 0 <= aerodynamic_front_fraction <= 1:
        raise ValueError('Downforce must be nonnegative and its front fraction in [0, 1].')
    vehicle = parameters.vehicle
    mass = vehicle.vehicle_mass_kg
    front_distance = vehicle.center_of_gravity_to_front_axle_m
    rear_distance = vehicle.center_of_gravity_to_rear_axle_m
    height = vehicle.center_of_gravity_height_m
    front_load = mass * (GRAVITY_MPS2 * rear_distance - longitudinal_acceleration_mps2 * height) / vehicle.wheelbase_m
    rear_load = mass * (GRAVITY_MPS2 * front_distance + longitudinal_acceleration_mps2 * height) / vehicle.wheelbase_m
    # Aerodynamic load adds normal force, never inertial mass. This prescribed
    # axle split includes its pitch balance; drag force height is not modeled.
    front_load += downforce_n * aerodynamic_front_fraction
    rear_load += downforce_n * (1 - aerodynamic_front_fraction)
    roll_moment = mass * lateral_acceleration_mps2 * height
    contacts = []
    for axle, x, track, axle_load, transfer in (
        ('front', front_distance, parameters.front_track_m, front_load, parameters.front_roll_moment_fraction * roll_moment / parameters.front_track_m),
        ('rear', -rear_distance, parameters.rear_track_m, rear_load, (1 - parameters.front_roll_moment_fraction) * roll_moment / parameters.rear_track_m),
    ):
        for side, side_sign in (('left', 1), ('right', -1)):
            load = axle_load / 2 - side_sign * transfer
            if load < -FORCE_TOLERANCE_N:
                raise InfeasibleOperatingPoint(f'{axle} {side} wheel lift: negative normal load.')
            contacts.append(WheelContact(f'{axle}_{side}', x, side_sign * track / 2, max(0.0, load)))
    return tuple(contacts)


def calculate_four_wheel_grip(parameters, deceleration_mps2, lateral_acceleration_mps2):
    """Return FL, FR, RL, RR loads and remaining longitudinal grip.

    Front/rear lateral forces balance yaw before braking. Within each axle,
    lateral force is proportional to wheel normal load. This prescribed
    force distribution is an assumption, not a solved steering equilibrium.
    Positive lateral acceleration is a left turn, loading the right wheels.
    """
    if not math.isfinite(deceleration_mps2) or deceleration_mps2 < 0:
        raise ValueError('deceleration_mps2 must be finite and nonnegative.')
    if not math.isfinite(lateral_acceleration_mps2):
        raise ValueError('lateral_acceleration_mps2 must be finite.')
    vehicle = parameters.vehicle
    mass = vehicle.vehicle_mass_kg
    front_distance = vehicle.center_of_gravity_to_front_axle_m
    rear_distance = vehicle.center_of_gravity_to_rear_axle_m
    wheelbase = vehicle.wheelbase_m
    front_lateral_force = mass * lateral_acceleration_mps2 * rear_distance / wheelbase
    rear_lateral_force = mass * lateral_acceleration_mps2 * front_distance / wheelbase
    contacts = calculate_four_wheel_contacts(parameters, -deceleration_mps2, lateral_acceleration_mps2)
    axle_loads = (sum(item.vertical_load_n for item in contacts[:2]), sum(item.vertical_load_n for item in contacts[2:]))
    wheels = []
    for index, contact in enumerate(contacts):
        axle_load = axle_loads[0 if index < 2 else 1]
        lateral_force = front_lateral_force if index < 2 else rear_lateral_force
        load = contact.vertical_load_n
        if axle_load <= FORCE_TOLERANCE_N:
            if abs(lateral_force) > FORCE_TOLERANCE_N:
                raise InfeasibleOperatingPoint('An unloaded axle cannot provide lateral force.')
            wheel_lateral_force = 0.0
        else:
            wheel_lateral_force = lateral_force * load / axle_load
        capacity = vehicle.friction_coefficient * load
        if abs(wheel_lateral_force) > capacity + FORCE_TOLERANCE_N:
            raise InfeasibleOperatingPoint(f'{contact.name} lateral demand exceeds tire grip.')
        remaining = math.sqrt(max(0.0, capacity**2 - wheel_lateral_force**2))
        wheels.append(WheelGrip(
            contact.name, contact.x_m, contact.y_m, load,
            wheel_lateral_force, capacity, remaining,
        ))
    return tuple(wheels)


def evaluate_braking_allocation(wheels, braking_forces_n):
    """Evaluate given per-wheel demands and their total yaw moment.

    Infeasible forces remain demands; they are never presented as achieved.
    """
    forces = tuple(braking_forces_n)
    if len(wheels) != 4 or len(forces) != 4:
        raise ValueError('Provide four wheels and four braking-force demands in FL, FR, RL, RR order.')
    if any(not math.isfinite(force) or force < 0 for force in forces):
        raise ValueError('Braking-force demands must be finite and nonnegative.')
    yaw = sum(wheel.x_m * wheel.lateral_force_n + wheel.y_m * force for wheel, force in zip(wheels, forces))
    feasible = all(force <= wheel.maximum_braking_force_n + FORCE_TOLERANCE_N for wheel, force in zip(wheels, forces))
    return BrakingAllocation(tuple(wheels), forces, yaw, feasible)


def allocate_axle_balanced_braking(wheels, total_braking_force_n, front_braking_fraction=None):
    """Equal left/right forces per axle, with fixed or freely chosen front bias.

    If front_braking_fraction is None, choose a feasible front/rear split near
    the axle load fraction. Return None if no such split exists. Fixed-split
    results can be infeasible; inspect is_feasible before using them.
    """
    _validate_total_force(total_braking_force_n)
    if front_braking_fraction is None:
        front_capacity = 2 * min(wheel.maximum_braking_force_n for wheel in wheels[:2])
        rear_capacity = 2 * min(wheel.maximum_braking_force_n for wheel in wheels[2:])
        minimum_front_force = max(0, total_braking_force_n - rear_capacity)
        maximum_front_force = min(total_braking_force_n, front_capacity)
        if minimum_front_force > maximum_front_force + FORCE_TOLERANCE_N:
            return None
        load_fraction = sum(wheel.vertical_load_n for wheel in wheels[:2]) / sum(wheel.vertical_load_n for wheel in wheels)
        front_force = min(maximum_front_force, max(minimum_front_force, load_fraction * total_braking_force_n))
    else:
        if not math.isfinite(front_braking_fraction) or not 0 <= front_braking_fraction <= 1:
            raise ValueError('front_braking_fraction must be between 0 and 1.')
        front_force = front_braking_fraction * total_braking_force_n
    rear_force = max(0.0, total_braking_force_n - front_force)
    return evaluate_braking_allocation(wheels, (front_force / 2, front_force / 2, rear_force / 2, rear_force / 2))


def _validate_total_force(total_braking_force_n):
    if not math.isfinite(total_braking_force_n) or total_braking_force_n < 0:
        raise ValueError('total_braking_force_n must be finite and nonnegative.')


def _extreme_yaw_allocation(wheels, total_force, reverse):
    forces = [0.0] * 4
    remaining = total_force
    for index in sorted(range(4), key=lambda index: wheels[index].y_m, reverse=reverse):
        forces[index] = min(remaining, wheels[index].maximum_braking_force_n)
        remaining -= forces[index]
    return evaluate_braking_allocation(wheels, forces)


def calculate_braking_yaw_range(wheels, total_braking_force_n):
    """Return min/max achievable total yaw (N m), or None if force is excessive."""
    _validate_total_force(total_braking_force_n)
    if total_braking_force_n > sum(wheel.maximum_braking_force_n for wheel in wheels) + FORCE_TOLERANCE_N:
        return None
    minimum = _extreme_yaw_allocation(wheels, total_braking_force_n, reverse=False)
    maximum = _extreme_yaw_allocation(wheels, total_braking_force_n, reverse=True)
    return minimum.yaw_moment_nm, maximum.yaw_moment_nm


def allocate_independent_braking(wheels, total_braking_force_n, target_yaw_moment_nm=0.0):
    """Find a feasible independent allocation for a requested yaw, or None.

    Greedy allocations give the yaw extremes at fixed total braking force.
    Interpolating them spans every intermediate yaw while retaining the force
    sum and wheel bounds. The returned solution is feasible, not uniquely
    optimal for heat, wear, or actuator effort. No external solver is needed.
    Set target_yaw_moment_nm=None to leave yaw unrestricted and distribute
    braking in proportion to remaining capacity instead.
    """
    _validate_total_force(total_braking_force_n)
    if target_yaw_moment_nm is None:
        capacity = sum(wheel.maximum_braking_force_n for wheel in wheels)
        if total_braking_force_n > capacity + FORCE_TOLERANCE_N:
            return None
        forces = tuple(total_braking_force_n * wheel.maximum_braking_force_n / capacity if capacity > 0 else 0.0 for wheel in wheels)
        return evaluate_braking_allocation(wheels, forces)
    if not math.isfinite(target_yaw_moment_nm):
        raise ValueError('target_yaw_moment_nm must be finite.')
    yaw_range = calculate_braking_yaw_range(wheels, total_braking_force_n)
    if yaw_range is None:
        return None
    minimum_yaw, maximum_yaw = yaw_range
    if target_yaw_moment_nm < minimum_yaw - FORCE_TOLERANCE_N or target_yaw_moment_nm > maximum_yaw + FORCE_TOLERANCE_N:
        return None
    minimum = _extreme_yaw_allocation(wheels, total_braking_force_n, reverse=False)
    maximum = _extreme_yaw_allocation(wheels, total_braking_force_n, reverse=True)
    span = maximum_yaw - minimum_yaw
    fraction = min(1.0, max(0.0, (target_yaw_moment_nm - minimum_yaw) / span)) if span > 1e-12 else 0.0
    forces = tuple((1 - fraction) * low + fraction * high for low, high in zip(minimum.braking_forces_n, maximum.braking_forces_n))
    return evaluate_braking_allocation(wheels, forces)
