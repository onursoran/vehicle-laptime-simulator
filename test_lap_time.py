from dataclasses import replace
import math
import unittest

import numpy as np

from adaptive_brake_control import AdaptiveBrakeSettings
from lap_time import LapTimeSolver
from research_setup import make_research_strategies
from track_model import make_test_track
from vehicle_equilibrium import OperatingPoint, PerformanceParameters


class AnalyticPerformanceSolver(LapTimeSolver):
    """Independent analytical envelope to verify the speed-profile algorithm."""
    def point(self, speed, acceleration, curvature, gradient, width):
        longitudinal_limit = 3 if acceleration >= 0 else 6
        use = math.hypot(acceleration / longitudinal_limit, speed**2 * curvature / 12)
        return OperatingPoint(use <= 1 + 1e-10, 'Analytic envelope', speed, acceleration, curvature, peak_tire_utilization=use, equilibrium_error=0)


class LapTimeTests(unittest.TestCase):
    def setUp(self):
        self.parameters = PerformanceParameters()
        self.strategies = make_research_strategies()

    def test_constant_radius_lap_against_analytical_solution(self):
        track = make_test_track('circle', 4, radius_m=20)
        result = AnalyticPerformanceSolver(self.parameters, self.strategies[0], speed_resolution_mps=.002).solve(track)
        expected_speed = math.sqrt(12 * 20)
        expected_time = 40 * math.pi / expected_speed
        self.assertLess(abs(result.lap_time_s / expected_time - 1), .002)
        self.assertLess(max(result.speed_mps) - min(result.speed_mps), .002)
        self.assertAlmostEqual(sum(result.segment_time_s), result.lap_time_s)

    def test_forward_backward_passes_produce_closed_feasible_profile(self):
        track = make_test_track('oval', 6)
        solver = AnalyticPerformanceSolver(self.parameters, self.strategies[0])
        result = solver.solve(track)
        self.assertLess(min(result.tangential_acceleration_mps2), -1)
        self.assertGreater(max(result.tangential_acceleration_mps2), 1)
        for index in range(len(track.distance_m)):
            feasible, points = solver._segment_points(track, index, result.speed_mps[index], result.speed_mps[(index + 1) % len(result.speed_mps)])
            self.assertTrue(feasible)
            self.assertEqual(len(points), 3)
        self.assertTrue(np.all(result.speed_mps <= self.parameters.maximum_speed_mps))

    def test_real_vehicle_lap_audits_force_and_yaw_balance(self):
        result = LapTimeSolver(self.parameters, self.strategies[0]).solve(make_test_track('oval', 8))
        self.assertLessEqual(result.maximum_equilibrium_error, self.parameters.equilibrium_tolerance)
        self.assertLessEqual(result.maximum_tire_utilization, 1 + 1e-7)
        self.assertLessEqual(result.maximum_abs_steering_rad, self.parameters.maximum_steering_angle_rad)
        self.assertTrue(all(point.is_feasible for point in result.segment_midpoints))

    def test_zero_assistance_lap_reproduces_conventional_baseline(self):
        track = make_test_track('oval', 8)
        baseline = LapTimeSolver(self.parameters, self.strategies[0]).solve(track)
        zero = replace(self.strategies[-1], assistance=AdaptiveBrakeSettings())
        independent = LapTimeSolver(self.parameters, zero).solve(track)
        self.assertLess(abs(independent.lap_time_s - baseline.lap_time_s), .01)
        np.testing.assert_allclose(independent.speed_mps, baseline.speed_mps, atol=.03)

    def test_mirrored_conventional_laps_and_swapped_valves_match(self):
        track = make_test_track('oval', 8)
        baseline = LapTimeSolver(self.parameters, self.strategies[0]).solve(track)
        mirrored = LapTimeSolver(self.parameters, self.strategies[0]).solve(track.mirrored())
        self.assertLess(abs(baseline.lap_time_s - mirrored.lap_time_s), .01)
        left = LapTimeSolver(self.parameters, self.strategies[3]).solve(track)
        right = LapTimeSolver(self.parameters, self.strategies[4]).solve(track.mirrored())
        self.assertLess(abs(left.lap_time_s - right.lap_time_s), .01)

    def test_road_narrower_than_car_is_rejected(self):
        with self.assertRaises(ValueError):
            LapTimeSolver(self.parameters, self.strategies[0]).solve(make_test_track('oval', width_m=1))

    def test_technical_track_mirror_uses_consistent_numerical_branch(self):
        track = make_test_track('technical', 2)
        original = LapTimeSolver(self.parameters, self.strategies[0]).solve(track)
        mirrored = LapTimeSolver(self.parameters, self.strategies[0]).solve(track.mirrored())
        self.assertLess(abs(original.lap_time_s - mirrored.lap_time_s), .005)

    def test_real_constant_radius_lap_has_no_assistance_gain_without_braking(self):
        track = make_test_track('circle', 8)
        baseline = LapTimeSolver(self.parameters, self.strategies[0]).solve(track)
        independent = LapTimeSolver(self.parameters, self.strategies[-1]).solve(track)
        self.assertLess(abs(baseline.lap_time_s - independent.lap_time_s), .001)
        self.assertLess(max(abs(value) for value in baseline.tangential_acceleration_mps2), .01)
        theoretical_speed_bound = math.sqrt(self.parameters.wheels.vehicle.friction_coefficient * 9.81 * 20)
        self.assertLessEqual(max(baseline.speed_mps), theoretical_speed_bound + .01)


if __name__ == '__main__':
    unittest.main()
