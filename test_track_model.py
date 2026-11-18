import math
import unittest
import numpy as np
from track_model import make_test_track


class TrackModelTests(unittest.TestCase):
    def test_circle_has_known_length_and_curvature(self):
        track = make_test_track('circle', 2, 3, radius_m=20)
        self.assertAlmostEqual(track.length_m, 40 * math.pi, places=7)
        np.testing.assert_allclose(track.curvature_per_m, 1 / 20, atol=1e-14)
        self.assertAlmostEqual(sum(track.segment_lengths_m), track.length_m)
        self.assertLess(max(track.segment_lengths_m), 2)

    def test_mirror_and_reversal_preserve_length_and_flip_turns(self):
        for name in ('oval', 'figure-eight', 'technical'):
            track = make_test_track(name)
            mirrored = track.mirrored()
            np.testing.assert_array_equal(mirrored.curvature_per_m, -track.curvature_per_m)
            np.testing.assert_allclose(track.reversed().reversed().x_m, track.x_m)
            self.assertAlmostEqual(track.reversed().length_m, track.length_m)

    def test_mixed_direction_tracks_and_width_control(self):
        for name in ('figure-eight', 'technical'):
            track = make_test_track(name, width_m=4.5)
            self.assertLess(min(track.curvature_per_m), 0)
            self.assertGreater(max(track.curvature_per_m), 0)
            self.assertTrue(np.all(track.width_m == 4.5))
        self.assertGreater(min(make_test_track('oval').curvature_per_m), 0)

    def test_invalid_dimensions(self):
        for kwargs in [dict(spacing_m=0), dict(width_m=-1), dict(radius_m=math.nan)]:
            with self.assertRaises(ValueError):
                make_test_track(**kwargs)


if __name__ == '__main__':
    unittest.main()
