from dataclasses import replace
import math
import unittest

from adaptive_brake_control import AdaptiveBrakeSettings
from brake_hydraulics import HydraulicConfiguration, ProportioningValve
from research_setup import make_research_strategies
from track_model import make_test_track
from vehicle_equilibrium import BrakeStrategy, PerformanceParameters, solve_operating_point


class VehicleEquilibriumTests(unittest.TestCase):
    def setUp(self):
        self.parameters = PerformanceParameters()
        self.baseline = BrakeStrategy('Ideal force baseline')

    def test_straight_braking_recovers_force_distribution(self):
        result = solve_operating_point(self.parameters, self.baseline, 10, -7, 0, road_width_m=3)
        self.assertTrue(result.is_feasible, result.reason)
        expected = (-630, -630, -420, -420)
        for actual, reference in zip(result.tire_longitudinal_forces_n, expected):
            self.assertAlmostEqual(actual, reference, places=2)
        self.assertAlmostEqual(result.steering_angle_rad, 0, places=7)
        self.assertAlmostEqual(sum(result.wheel_loads_n), 300 * 9.81)

    def test_curved_braking_satisfies_all_force_and_yaw_equations(self):
        result = solve_operating_point(self.parameters, self.baseline, 8, -4, .04, road_width_m=3)
        self.assertTrue(result.is_feasible, result.reason)
        self.assertLess(max(abs(error) for error in result.force_balance_errors_n), .02)
        self.assertLess(abs(result.yaw_balance_error_nm), .02)
        self.assertAlmostEqual(result.yaw_acceleration_radps2, -4 * .04)
        self.assertAlmostEqual(result.yaw_moment_nm, 150 * (-4 * .04), places=2)
        self.assertGreater(result.steering_angle_rad, 0)

    def test_curvature_gradient_requires_yaw_acceleration(self):
        result = solve_operating_point(self.parameters, self.baseline, 8, 0, .03, .001, 3)
        self.assertTrue(result.is_feasible, result.reason)
        self.assertAlmostEqual(result.yaw_acceleration_radps2, .064)
        self.assertLess(abs(result.yaw_balance_error_nm), .02)

    def test_left_right_symmetry_of_conventional_vehicle(self):
        left = solve_operating_point(self.parameters, self.baseline, 8, -4, .04, .001, 3)
        right = solve_operating_point(self.parameters, self.baseline, 8, -4, -.04, -.001, 3)
        self.assertTrue(left.is_feasible, left.reason)
        self.assertTrue(right.is_feasible, right.reason)
        self.assertAlmostEqual(left.steering_angle_rad, -right.steering_angle_rad, places=5)
        self.assertAlmostEqual(left.sideslip_angle_rad, -right.sideslip_angle_rad, places=5)
        self.assertAlmostEqual(left.peak_tire_utilization, right.peak_tire_utilization, places=5)

    def test_passive_asymmetry_requires_steering_even_on_straight(self):
        strategy = make_research_strategies()[3]
        result = solve_operating_point(self.parameters, strategy, 10, -6, 0, road_width_m=3)
        self.assertTrue(result.is_feasible, result.reason)
        self.assertGreater(abs(result.steering_angle_rad), .001)
        self.assertGreater(abs(result.brake_yaw_moment_nm), 1)
        self.assertLess(abs(result.yaw_moment_nm), .02)

    def test_adaptive_allocation_preserves_front_rear_preference(self):
        strategy = make_research_strategies(.65, .5)[-1]
        result = solve_operating_point(self.parameters, strategy, 8, -5, .05, road_width_m=3)
        self.assertTrue(result.is_feasible, result.reason)
        front = -sum(result.tire_longitudinal_forces_n[:2])
        total = -sum(result.tire_longitudinal_forces_n)
        self.assertAlmostEqual(front / total, .65)
        self.assertLess(abs(result.brake_yaw_moment_nm), 120 + 1e-7)
        self.assertLess(abs(result.yaw_balance_error_nm), .02)

    def test_hydraulic_supply_power_and_road_bounds_are_enforced(self):
        strategy = replace(make_research_strategies()[0], maximum_pushrod_force_n=10)
        self.assertFalse(solve_operating_point(self.parameters, strategy, 10, -6, 0, road_width_m=3).is_feasible)
        self.assertFalse(solve_operating_point(replace(self.parameters, drive_power_w=1), self.baseline, 10, 2, 0, road_width_m=3).is_feasible)
        self.assertFalse(solve_operating_point(self.parameters, self.baseline, 10, 0, 0, road_width_m=1).is_feasible)

    def test_quasi_steady_laps_reject_stateful_assistance_and_retention(self):
        with self.assertRaises(ValueError):
            BrakeStrategy('Invalid envelope', kind='adaptive', assistance=AdaptiveBrakeSettings(maximum_asymmetry_change_per_s=1))
        with self.assertRaises(ValueError):
            replace(make_research_strategies()[0], configuration=HydraulicConfiguration(wheel_faults=('retained', 'normal', 'normal', 'normal')))

    def test_smooth_technical_track_has_supported_low_speed_equilibria(self):
        track = make_test_track('technical', 2)
        for index, (curvature, gradient) in enumerate(zip(track.curvature_per_m, track.curvature_gradient_per_m2)):
            result = solve_operating_point(self.parameters, self.baseline, .5, 0, curvature, gradient, 3)
            self.assertTrue(result.is_feasible, f'Node {index}, curvature {curvature}, gradient {gradient}: {result}')


if __name__ == '__main__':
    unittest.main()
