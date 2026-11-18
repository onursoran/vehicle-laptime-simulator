"""Physical reference cases and operating-boundary checks for braking."""

from dataclasses import replace
import math
import unittest

from brake_model import (
    BrakingParameters,
    GRAVITY_MPS2,
    calculate_ideal_front_braking_fraction,
    calculate_straight_line_braking_state,
    find_maximum_deceleration_for_brake_bias,
)


class BrakeModelTests(unittest.TestCase):
    def setUp(self):
        self.parameters = BrakingParameters()

    def test_reference_loads_and_ideal_split(self):
        for demand, front_load, rear_load in [(0, 1308, 1635), (9.81, 1798.5, 1144.5)]:
            with self.subTest(demand=demand):
                fraction = calculate_ideal_front_braking_fraction(self.parameters, demand)
                state = calculate_straight_line_braking_state(self.parameters, demand, fraction)
                self.assertAlmostEqual(state.front_vertical_load_n, front_load)
                self.assertAlmostEqual(state.rear_vertical_load_n, rear_load)
                self.assertAlmostEqual(fraction, front_load / 2943)
                self.assertAlmostEqual(state.front_tire_utilization, state.rear_tire_utilization)

    def test_ideal_split_reaches_friction_ceiling(self):
        demand = 1.5 * GRAVITY_MPS2
        fraction = calculate_ideal_front_braking_fraction(self.parameters, demand)
        self.assertAlmostEqual(fraction, 25 / 36)
        limit = find_maximum_deceleration_for_brake_bias(self.parameters, fraction)
        self.assertAlmostEqual(limit.maximum_deceleration_mps2, demand)
        self.assertAlmostEqual(limit.state.front_tire_utilization, 1)
        self.assertAlmostEqual(limit.state.rear_tire_utilization, 1)
        self.assertIn('front tire saturation', limit.limiting_constraint)
        self.assertIn('rear tire saturation', limit.limiting_constraint)

    def test_fixed_bias_limits_and_conservation(self):
        for fraction in [0, .2, .5, .6, .8, 1]:
            with self.subTest(fraction=fraction):
                limit = find_maximum_deceleration_for_brake_bias(self.parameters, fraction)
                state = limit.state
                self.assertTrue(state.is_feasible)
                self.assertLessEqual(limit.maximum_deceleration_mps2, 1.5 * GRAVITY_MPS2 + 1e-10)
                self.assertAlmostEqual(state.front_vertical_load_n + state.rear_vertical_load_n, 300 * GRAVITY_MPS2)
                self.assertAlmostEqual(state.front_braking_force_demand_n + state.rear_braking_force_demand_n, 300 * limit.maximum_deceleration_mps2)
                above = calculate_straight_line_braking_state(self.parameters, limit.maximum_deceleration_mps2 + .001, fraction)
                self.assertFalse(above.is_feasible)

    def test_limiting_axle_switches_with_bias(self):
        self.assertEqual(find_maximum_deceleration_for_brake_bias(self.parameters, .5).limiting_constraint, 'rear tire saturation')
        self.assertEqual(find_maximum_deceleration_for_brake_bias(self.parameters, .8).limiting_constraint, 'front tire saturation')

    def test_zero_height_symmetric_vehicle(self):
        parameters = replace(self.parameters, center_of_gravity_to_rear_axle_m=1, center_of_gravity_height_m=0)
        self.assertAlmostEqual(calculate_ideal_front_braking_fraction(parameters, 9.81), .5)
        for fraction, expected_g in [(.5, 1.5), (0, .75), (1, .75)]:
            self.assertAlmostEqual(find_maximum_deceleration_for_brake_bias(parameters, fraction).maximum_deceleration_mps2, expected_g * GRAVITY_MPS2)

    def test_mass_does_not_change_deceleration_limit(self):
        for fraction in [.2, .6, .9]:
            light = find_maximum_deceleration_for_brake_bias(self.parameters, fraction)
            heavy = find_maximum_deceleration_for_brake_bias(replace(self.parameters, vehicle_mass_kg=600), fraction)
            self.assertAlmostEqual(light.maximum_deceleration_mps2, heavy.maximum_deceleration_mps2)
            self.assertAlmostEqual(heavy.state.front_vertical_load_n, 2 * light.state.front_vertical_load_n)

    def test_zero_friction(self):
        for height in [0, .3]:
            parameters = replace(self.parameters, friction_coefficient=0, center_of_gravity_height_m=height)
            for fraction in [0, .5, 1]:
                self.assertEqual(find_maximum_deceleration_for_brake_bias(parameters, fraction).maximum_deceleration_mps2, 0)
                self.assertFalse(calculate_straight_line_braking_state(parameters, 1, fraction).is_feasible)

    def test_rear_lift_is_explicit_boundary(self):
        parameters = replace(self.parameters, center_of_gravity_height_m=1, friction_coefficient=2)
        limit = find_maximum_deceleration_for_brake_bias(parameters, 1)
        self.assertAlmostEqual(limit.maximum_deceleration_mps2, GRAVITY_MPS2)
        self.assertIn('rear axle lift', limit.limiting_constraint)
        with self.assertRaises(ValueError):
            calculate_straight_line_braking_state(parameters, GRAVITY_MPS2 + .01, 1)

    def test_invalid_inputs(self):
        for changes in [dict(vehicle_mass_kg=0), dict(center_of_gravity_to_front_axle_m=-1), dict(center_of_gravity_to_rear_axle_m=0), dict(center_of_gravity_height_m=-1), dict(friction_coefficient=-1), dict(friction_coefficient=math.nan)]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(self.parameters, **changes)
        for fraction in [-.1, 1.1, math.nan, math.inf]:
            with self.subTest(fraction=fraction), self.assertRaises(ValueError):
                find_maximum_deceleration_for_brake_bias(self.parameters, fraction)
        for demand in [-1, math.nan, math.inf]:
            with self.subTest(demand=demand), self.assertRaises(ValueError):
                calculate_ideal_front_braking_fraction(self.parameters, demand)


if __name__ == '__main__':
    unittest.main()
