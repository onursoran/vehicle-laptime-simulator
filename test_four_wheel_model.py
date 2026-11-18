"""Conservation, symmetry, allocation, and contact/grip boundary checks."""

from dataclasses import replace
import math
import unittest

from brake_model import BrakingParameters, GRAVITY_MPS2, find_maximum_deceleration_for_brake_bias
from four_wheel_model import (
    FourWheelParameters, InfeasibleOperatingPoint,
    allocate_axle_balanced_braking, allocate_independent_braking,
    calculate_braking_yaw_range, calculate_four_wheel_grip,
    evaluate_braking_allocation,
)


class FourWheelModelTests(unittest.TestCase):
    def setUp(self):
        self.parameters = FourWheelParameters()

    def test_static_loads(self):
        wheels = calculate_four_wheel_grip(self.parameters, 0, 0)
        self.assertEqual([wheel.vertical_load_n for wheel in wheels], [654, 654, 817.5, 817.5])

    def test_force_and_pitch_roll_moment_conservation(self):
        for front_track, rear_track in [(1.2, 1.2), (1.1, 1.3)]:
            parameters = replace(self.parameters, front_track_m=front_track, rear_track_m=rear_track)
            mass = parameters.vehicle.vehicle_mass_kg
            height = parameters.vehicle.center_of_gravity_height_m
            for lateral in [-5, 0, 5]:
                wheels = calculate_four_wheel_grip(parameters, 7, lateral)
                self.assertAlmostEqual(sum(wheel.vertical_load_n for wheel in wheels), mass * GRAVITY_MPS2)
                self.assertAlmostEqual(sum(wheel.lateral_force_n for wheel in wheels), mass * lateral)
                self.assertAlmostEqual(sum(wheel.x_m * wheel.lateral_force_n for wheel in wheels), 0)
                self.assertAlmostEqual(sum(wheel.x_m * wheel.vertical_load_n for wheel in wheels), mass * 7 * height)
                self.assertAlmostEqual(sum(wheel.y_m * wheel.vertical_load_n for wheel in wheels), -mass * lateral * height)

    def test_left_right_turn_mirror(self):
        left = calculate_four_wheel_grip(self.parameters, 7, 5)
        right = calculate_four_wheel_grip(self.parameters, 7, -5)
        for index, mirrored in enumerate([1, 0, 3, 2]):
            self.assertAlmostEqual(left[index].vertical_load_n, right[mirrored].vertical_load_n)
            self.assertAlmostEqual(left[index].lateral_force_n, -right[mirrored].lateral_force_n)
            self.assertAlmostEqual(left[index].maximum_braking_force_n, right[mirrored].maximum_braking_force_n)
        low, high = calculate_braking_yaw_range(left, 1800)
        mirrored_low, mirrored_high = calculate_braking_yaw_range(right, 1800)
        self.assertAlmostEqual(low, -mirrored_high)
        self.assertAlmostEqual(high, -mirrored_low)

    def test_differential_braking_yaw_sign(self):
        wheels = calculate_four_wheel_grip(self.parameters, 0, 0)
        left = evaluate_braking_allocation(wheels, (100, 0, 0, 0))
        right = evaluate_braking_allocation(wheels, (0, 100, 0, 0))
        self.assertAlmostEqual(left.yaw_moment_nm, 60)
        self.assertAlmostEqual(right.yaw_moment_nm, -60)

    def test_friction_circle_remaining_capacity(self):
        wheels = calculate_four_wheel_grip(self.parameters, 7, 5)
        for wheel in wheels:
            self.assertAlmostEqual(wheel.maximum_braking_force_n**2 + wheel.lateral_force_n**2, wheel.friction_capacity_n**2)

    def test_fixed_split_recovers_straight_line_model(self):
        for fraction in [0, .5, .6, .8, 1]:
            limit = find_maximum_deceleration_for_brake_bias(self.parameters.vehicle, fraction)
            wheels = calculate_four_wheel_grip(self.parameters, limit.maximum_deceleration_mps2, 0)
            allocation = allocate_axle_balanced_braking(wheels, 300 * limit.maximum_deceleration_mps2, fraction)
            self.assertTrue(allocation.is_feasible)
            self.assertAlmostEqual(allocation.yaw_moment_nm, 0)
            wheels = calculate_four_wheel_grip(self.parameters, limit.maximum_deceleration_mps2 + .001, 0)
            self.assertFalse(allocate_axle_balanced_braking(wheels, 300 * (limit.maximum_deceleration_mps2 + .001), fraction).is_feasible)

    def test_independent_allocation_preserves_force_yaw_and_bounds(self):
        for front_track, rear_track in [(1.2, 1.2), (1.1, 1.3)]:
            parameters = replace(self.parameters, front_track_m=front_track, rear_track_m=rear_track)
            wheels = calculate_four_wheel_grip(parameters, 7, 5)
            lower, upper = calculate_braking_yaw_range(wheels, 2100)
            for target in [lower, (lower + upper) / 2, upper, 0]:
                with self.subTest(tracks=(front_track, rear_track), target=target):
                    allocation = allocate_independent_braking(wheels, 2100, target)
                    self.assertIsNotNone(allocation)
                    self.assertTrue(allocation.is_feasible)
                    self.assertAlmostEqual(allocation.total_braking_force_n, 2100)
                    self.assertAlmostEqual(allocation.yaw_moment_nm, target)
            self.assertIsNone(allocate_independent_braking(wheels, 2100, upper + 1))
            self.assertIsNone(allocate_independent_braking(wheels, 2100, lower - 1))

    def test_equal_tracks_zero_yaw_and_adjustable_axles_have_same_feasibility(self):
        # Inside capacities are smaller on both axles in this prescribed-force model.
        for lateral in [-6, 0, 6]:
            for deceleration in range(14):
                try:
                    wheels = calculate_four_wheel_grip(self.parameters, deceleration, lateral)
                except InfeasibleOperatingPoint:
                    continue
                force = 300 * deceleration
                axles = allocate_axle_balanced_braking(wheels, force)
                independent = allocate_independent_braking(wheels, force, 0)
                self.assertEqual(axles is not None and axles.is_feasible, independent is not None and independent.is_feasible)

    def test_unrestricted_yaw_does_not_imply_zero_yaw(self):
        wheels = calculate_four_wheel_grip(self.parameters, 1.1 * GRAVITY_MPS2, .6 * GRAVITY_MPS2)
        force = 300 * 1.1 * GRAVITY_MPS2
        unrestricted = allocate_independent_braking(wheels, force, None)
        self.assertIsNotNone(unrestricted)
        self.assertTrue(unrestricted.is_feasible)
        self.assertAlmostEqual(unrestricted.total_braking_force_n, force)
        self.assertLess(unrestricted.yaw_moment_nm, 0)
        self.assertIsNone(allocate_independent_braking(wheels, force, 0))

    def test_zero_and_excessive_braking_demand(self):
        wheels = calculate_four_wheel_grip(self.parameters, 0, 0)
        allocation = allocate_independent_braking(wheels, 0, 0)
        self.assertEqual(allocation.total_braking_force_n, 0)
        self.assertIsNone(allocation.front_braking_fraction)
        for yaw in [0, None]:
            self.assertIsNone(allocate_independent_braking(wheels, 5000, yaw))
        parameters = replace(self.parameters, vehicle=BrakingParameters(friction_coefficient=0))
        wheels = calculate_four_wheel_grip(parameters, 0, 0)
        self.assertTrue(allocate_independent_braking(wheels, 0, None).is_feasible)
        self.assertIsNone(allocate_independent_braking(wheels, 1, None))

    def test_infeasible_contact_and_cornering(self):
        with self.assertRaises(InfeasibleOperatingPoint):
            calculate_four_wheel_grip(self.parameters, 40, 0)
        with self.assertRaises(InfeasibleOperatingPoint):
            calculate_four_wheel_grip(self.parameters, 0, 100)
        with self.assertRaises(InfeasibleOperatingPoint):
            calculate_four_wheel_grip(self.parameters, 0, 16)

    def test_invalid_inputs(self):
        for changes in [dict(front_track_m=0), dict(rear_track_m=math.inf), dict(front_roll_moment_fraction=-.1), dict(front_roll_moment_fraction=math.nan)]:
            with self.assertRaises(ValueError):
                replace(self.parameters, **changes)
        for demand, lateral in [(-1, 0), (math.nan, 0), (0, math.inf)]:
            with self.assertRaises(ValueError):
                calculate_four_wheel_grip(self.parameters, demand, lateral)
        wheels = calculate_four_wheel_grip(self.parameters, 0, 0)
        for force in [-1, math.nan]:
            with self.assertRaises(ValueError):
                allocate_independent_braking(wheels, force)


if __name__ == '__main__':
    unittest.main()
