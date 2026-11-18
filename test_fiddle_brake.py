from dataclasses import replace
import math
import unittest

from fiddle_brake import FiddleBrakeSettings, auxiliary_braking
from research_setup import make_research_strategies, make_research_vehicle, select_research_strategies
from vehicle_equilibrium import solve_operating_point, _solve_drive
from simulation_service import compute_simulation, validate_inputs


class FiddleBrakeTests(unittest.TestCase):
    def setUp(self):
        self.parameters = make_research_vehicle('illustrative')
        self.baseline, self.fiddle = select_research_strategies(make_research_strategies(profile='illustrative'), 'fiddle')

    def test_inside_rear_selection_limits_and_signal_gates(self):
        settings = FiddleBrakeSettings(strength_fraction=1)
        limits = (1000,) * 4
        left = auxiliary_braking(settings, 15, 12, .1, 1.2, limits, limits)
        right = auxiliary_braking(settings, 15, -12, -.1, 1.2, limits, limits)
        self.assertEqual(left, (0, 0, 200, 0))
        self.assertEqual(right, (0, 0, 0, 200))
        self.assertEqual(auxiliary_braking(settings, 15, 12, .1, 1.2, (0, 0, 30, 30), (0, 0, 20, 20)), (0, 0, 20, 0))
        for speed, ay, steering in [(1, 12, .1), (15, 0, .1), (15, 12, 0), (15, 12, -.1), (15, .1, .1)]:
            self.assertEqual(auxiliary_braking(settings, speed, ay, steering, 1.2, limits, limits), (0, 0, 0, 0))
        for invalid in [dict(strength_fraction=1.1), dict(maximum_yaw_moment_nm=-1), dict(minimum_speed_mps=math.nan)]:
            with self.assertRaises(ValueError):
                FiddleBrakeSettings(**invalid)

    def test_zero_strength_and_straight_line_recover_baseline(self):
        zero = replace(self.fiddle, fiddle=replace(self.fiddle.fiddle, strength_fraction=0))
        for acceleration, curvature in [(-4, .04), (2, .04), (0, .04), (-4, 0), (2, 0)]:
            expected = solve_operating_point(self.parameters, self.baseline, 8, acceleration, curvature, 0, 12)
            self.assertTrue(expected.is_feasible, expected.reason)
            self.assertEqual(solve_operating_point(self.parameters, zero, 8, acceleration, curvature, 0, 12), expected)
            if curvature == 0:
                self.assertEqual(solve_operating_point(self.parameters, self.fiddle, 8, acceleration, curvature, 0, 12), expected)

    def test_auxiliary_rear_only_mirrors_and_main_pedal_still_works(self):
        for acceleration in (-4, 0, 2):
            left = solve_operating_point(self.parameters, self.fiddle, 8, acceleration, .04, 0, 12)
            right = solve_operating_point(self.parameters, self.fiddle, 8, acceleration, -.04, 0, 12)
            for point, inside, outside in [(left, 2, 3), (right, 3, 2)]:
                self.assertTrue(point.is_feasible, point.reason)
                self.assertGreater(point.auxiliary_braking_forces_n[inside], 0)
                self.assertEqual(point.auxiliary_braking_forces_n[:2], (0, 0))
                self.assertEqual(point.auxiliary_braking_forces_n[outside], 0)
                self.assertLess(point.equilibrium_error, self.parameters.equilibrium_tolerance)
                self.assertLessEqual(point.peak_tire_utilization, 1 + 1e-7)
                self.assertLess(abs(point.brake_yaw_moment_nm), 120)
            self.assertAlmostEqual(left.brake_yaw_moment_nm, -right.brake_yaw_moment_nm, places=5)
            self.assertAlmostEqual(left.steering_angle_rad, -right.steering_angle_rad, places=5)
            if acceleration < 0:
                self.assertTrue(all(force < 0 for force in left.tire_longitudinal_forces_n))
                service = [-(fx + extra) for fx, extra in zip(left.tire_longitudinal_forces_n, left.auxiliary_braking_forces_n)]
                self.assertAlmostEqual(sum(service[:2]) / sum(service), .6)
                self.assertLess(sum(left.tire_longitudinal_forces_n[:2]) / sum(left.tire_longitudinal_forces_n), .6)
            else:
                self.assertGreater(left.drive_force_n, 0)
                self.assertGreater(left.wheel_pressures_pa[2], 0)
                self.assertEqual(left.wheel_pressures_pa[3], 0)

    def test_drive_overlap_costs_power_and_does_not_use_drive_only_cache(self):
        _solve_drive.cache_clear()
        assisted = solve_operating_point(self.parameters, self.fiddle, 8, 2, .04, 0, 12)
        self.assertEqual(_solve_drive.cache_info().misses, 0)
        baseline = solve_operating_point(self.parameters, self.baseline, 8, 2, .04, 0, 12)
        self.assertGreater(assisted.drive_force_n, baseline.drive_force_n)
        self.assertAlmostEqual(sum(assisted.tire_longitudinal_forces_n) + sum(assisted.auxiliary_braking_forces_n), assisted.drive_force_n)
        limited = replace(self.parameters, drive_power_w=1)
        self.assertFalse(solve_operating_point(limited, self.fiddle, 8, 2, .04, 0, 12).is_feasible)

    def test_lap_comparison_exports_auxiliary_forces_and_respects_constraints(self):
        self.assertEqual(validate_inputs({'configurations':'fiddle'})['configurations'], 'fiddle')
        data = compute_simulation({'configurations':'fiddle', 'track':'figure-eight', 'spacing':8, 'width':5})
        self.assertFalse(data['failures'])
        self.assertEqual([strategy['kind'] for strategy in data['strategies']], ['conventional', 'fiddle'])
        profile = data['results'][1]
        self.assertGreater(profile['lap_time_s'], 0)
        self.assertLessEqual(profile['maximum_equilibrium_error'], 2e-6)
        self.assertLessEqual(profile['peak_tire_utilization'], 1 + 1e-7)
        forces = profile['auxiliary_braking_forces_n']
        self.assertTrue(any(row[2] > 0 for row in forces))
        self.assertTrue(any(row[3] > 0 for row in forces))
        for row in forces:
            self.assertEqual(row[:2], (0, 0))
            self.assertEqual(row[2] * row[3], 0)


if __name__ == '__main__':
    unittest.main()
