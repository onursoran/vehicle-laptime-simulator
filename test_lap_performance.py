import unittest

import numpy as np

from benchmark_lap_performance import PreviousGeometrySolver, assert_identical, fixed_benchmark_inputs
from lap_time import LapTimeSolver
from test_lap_time import AnalyticPerformanceSolver
from track_model import make_test_track


class CountedGeometryTrack:
    def __init__(self, track):
        self.track = track
        self.length_reads = 0
        self.gradient_reads = 0

    def __getattr__(self, name):
        return getattr(self.track, name)

    @property
    def segment_lengths_m(self):
        self.length_reads += 1
        return self.track.segment_lengths_m

    @property
    def curvature_gradient_per_m2(self):
        self.gradient_reads += 1
        return self.track.curvature_gradient_per_m2


class LapPerformanceTests(unittest.TestCase):
    def test_full_lap_geometry_is_built_once_per_solve(self):
        parameters, strategy = fixed_benchmark_inputs()
        track = CountedGeometryTrack(make_test_track('oval', 2))
        solver = AnalyticPerformanceSolver(parameters, strategy)
        result = solver.solve(track)
        self.assertEqual(track.length_reads, 1)
        self.assertEqual(track.gradient_reads, 1)
        for index in range(len(track.distance_m)):
            feasible, points = solver._segment_points(track, index, result.speed_mps[index], result.speed_mps[(index + 1) % len(result.speed_mps)])
            self.assertTrue(feasible)
            self.assertEqual(len(points), 3)
        self.assertEqual(track.gradient_reads, 1)

    def test_reused_solver_refreshes_geometry_and_preserves_results(self):
        parameters, strategy = fixed_benchmark_inputs()
        track = make_test_track('oval', 8)
        solver = AnalyticPerformanceSolver(parameters, strategy)
        solver.solve(track)
        track.curvature_per_m[:] *= .8
        reused = solver.solve(track)
        fresh = AnalyticPerformanceSolver(parameters, strategy).solve(track)
        np.testing.assert_array_equal(reused.speed_mps, fresh.speed_mps)
        self.assertEqual(reused.lap_time_s, fresh.lap_time_s)

    def test_optimized_geometry_preserves_all_physical_outputs_exactly(self):
        parameters, strategy = fixed_benchmark_inputs()
        track = make_test_track('technical', 4)
        previous = PreviousGeometrySolver(parameters, strategy).solve(track)
        optimized = LapTimeSolver(parameters, strategy).solve(track)
        assert_identical(previous, optimized)

    def test_cancellation_interrupts_initial_speed_search(self):
        parameters, strategy = fixed_benchmark_inputs()
        def cancelled():
            raise RuntimeError('Calculation cancelled.')
        solver = LapTimeSolver(parameters, strategy, cancel_check=cancelled)
        with self.assertRaisesRegex(RuntimeError, 'Calculation cancelled'):
            solver.solve(make_test_track('oval'))
        self.assertEqual(len(solver._points), 0)

    def test_cancellation_is_checked_during_segment_propagation(self):
        parameters, strategy = fixed_benchmark_inputs()
        track = make_test_track('oval', 2)
        checks = []
        def cancelled_after_initial_envelope():
            checks.append(True)
            if len(checks) > len(track.distance_m):
                raise RuntimeError('Calculation cancelled.')
        solver = AnalyticPerformanceSolver(parameters, strategy, cancel_check=cancelled_after_initial_envelope)
        with self.assertRaisesRegex(RuntimeError, 'Calculation cancelled'):
            solver.solve(track)
        self.assertEqual(len(checks), len(track.distance_m) + 1)


if __name__ == '__main__':
    unittest.main()
