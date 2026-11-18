import math
import unittest

import numpy as np

from real_tracks import PeriodicCenterline, REAL_TRACKS, make_track
from braking_analysis import analyze_braking
from lap_time import LapTimeSolver
from research_setup import make_research_strategies, make_research_vehicle


class RealTrackTests(unittest.TestCase):
    def test_periodic_spline_circle_geometry_and_closing_derivatives(self):
        theta = np.linspace(0, 2 * math.pi, 80, endpoint=False)
        spline = PeriodicCenterline(np.c_[20 * np.cos(theta), 20 * np.sin(theta)])
        distance, xy, heading, curvature = spline.sample(1)
        self.assertAlmostEqual(spline.length_m, 40 * math.pi, delta=.001)
        np.testing.assert_allclose(np.linalg.norm(xy, axis=1), 20, atol=1e-5)
        np.testing.assert_allclose(curvature, .05, atol=3e-5)
        position, first, second = spline.evaluate(np.array([1e-7, spline.knots[-1] - 1e-7]))
        for values in (position, first, second):
            np.testing.assert_allclose(values[0], values[1], atol=3e-7)

    def test_periodic_solution_satisfies_nonuniform_spline_equations(self):
        theta = np.linspace(0, 2 * math.pi, 17, endpoint=False)
        spline = PeriodicCenterline(np.c_[35 * np.cos(theta), 12 * np.sin(theta)])
        h, m, xy = spline.h, spline.second_derivatives, spline.coordinates
        slopes = (np.roll(xy, -1, axis=0) - xy) / h[:, None]
        lhs = np.roll(h, 1)[:, None] * np.roll(m, 1, axis=0) + 2 * (h + np.roll(h, 1))[:, None] * m + h[:, None] * np.roll(m, -1, axis=0)
        np.testing.assert_allclose(lhs, 6 * (slopes - np.roll(slopes, 1, axis=0)), atol=1e-12)

    def test_bundled_circuits_have_full_metric_scale_and_mixed_turns(self):
        historical_lengths = dict(monza=5793, montreal=4361, catalunya=4655)
        for name in REAL_TRACKS:
            with self.subTest(track=name):
                track = make_track(name)
                self.assertLess(abs(track.length_m / historical_lengths[name] - 1), .02)
                self.assertLess(min(track.curvature_per_m), -.01)
                self.assertGreater(max(track.curvature_per_m), .01)
                self.assertTrue(np.all(np.isfinite(track.curvature_gradient_per_m2)))
                self.assertTrue(np.all(track.width_m == 12))
                self.assertLessEqual(max(track.segment_lengths_m), 4.000001)
                self.assertEqual(track.metadata['kind'], 'real')
                self.assertEqual(len(track.metadata['source_sha256']), 64)
                self.assertIn('2020', track.name)
                self.assertFalse(track.metadata['mirrored'])
                self.assertTrue(track.mirrored().metadata['mirrored'])

    def test_refinement_preserves_geometry_and_user_width(self):
        coarse, fine = make_track('monza', 4, 4.5), make_track('monza', 1, 7)
        self.assertEqual(coarse.length_m, fine.length_m)
        self.assertTrue(np.all(coarse.width_m == 4.5))
        self.assertTrue(np.all(fine.width_m == 7))
        np.testing.assert_allclose(coarse.x_m[0], fine.x_m[0])
        np.testing.assert_array_equal(coarse.mirrored().curvature_per_m, -coarse.curvature_per_m)
        self.assertEqual(make_track('oval').width_m[0], 3)

    def test_invalid_selection_geometry_and_dimensions(self):
        for args in [('missing',), ('monza', 0), ('monza', 4, math.nan)]:
            with self.assertRaises(ValueError):
                make_track(*args)
        with self.assertRaises(ValueError):
            PeriodicCenterline(np.zeros((10, 2)))

    def test_full_size_circuits_produce_audited_light_and_hard_braking(self):
        parameters = make_research_vehicle()
        strategy = make_research_strategies()[0]
        for name in REAL_TRACKS:
            with self.subTest(track=name):
                track = make_track(name)
                result = LapTimeSolver(parameters, strategy).solve(track)
                braking = analyze_braking(track, result)
                self.assertGreater(braking['distance_m']['light'], 0)
                self.assertGreater(braking['distance_m']['hard'], 0)
                self.assertAlmostEqual(sum(braking['distance_m'].values()), track.length_m, places=7)
                self.assertAlmostEqual(sum(braking['time_s'].values()), result.lap_time_s, places=7)
                self.assertLessEqual(result.maximum_tire_utilization, 1 + 1e-7)
                self.assertLessEqual(result.maximum_equilibrium_error, parameters.equilibrium_tolerance)
                self.assertLessEqual(max(result.speed_mps), parameters.maximum_speed_mps)


if __name__ == '__main__':
    unittest.main()
