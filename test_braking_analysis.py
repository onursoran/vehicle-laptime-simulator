import json
from types import SimpleNamespace
import unittest

from braking_analysis import analyze_braking


def fixture(decelerations_g, commands=None, lengths=None):
    count = len(decelerations_g)
    lengths = lengths or [10.0] * count
    distances = [sum(lengths[:index]) for index in range(count)]
    track = SimpleNamespace(distance_m=distances, segment_lengths_m=lengths, length_m=sum(lengths))
    result = SimpleNamespace(
        segment_time_s=[float(index + 1) for index in range(count)],
        speed_mps=[float(20 - index) for index in range(count)],
        tangential_acceleration_mps2=[-9.81 * value for value in decelerations_g],
        segment_midpoints=[SimpleNamespace(pushrod_force_n=value) for value in (commands or [100.0] * count)],
    )
    return track, result


class BrakingAnalysisTests(unittest.TestCase):
    def test_thresholds_and_partition_use_path_deceleration_and_applied_brakes(self):
        track, result = fixture([0, .05, .2, .3, .7, 1.2, -.2], lengths=[3, 4, 5, 6, 7, 8, 9])
        analysis = analyze_braking(track, result)
        self.assertEqual(analysis['segment_category'], ['none', 'none', 'light', 'medium', 'hard', 'hard', 'none'])
        self.assertEqual(analysis['distance_m'], dict(none=16.0, light=5.0, medium=6.0, hard=15.0))
        self.assertAlmostEqual(sum(analysis['distance_m'].values()), track.length_m)
        self.assertAlmostEqual(sum(analysis['time_s'].values()), sum(result.segment_time_s))
        self.assertAlmostEqual(analysis['peak_deceleration_g'], 1.2)
        self.assertEqual(len(analysis['zones']), 1)
        zone = analysis['zones'][0]
        self.assertEqual((zone['start_distance_m'], zone['end_distance_m'], zone['length_m']), (7, 33, 26))
        self.assertEqual(zone['duration_s'], 18)
        self.assertEqual(zone['category'], 'hard')
        self.assertEqual((zone['entry_speed_mps'], zone['exit_speed_mps']), (18, 14))
        self.assertFalse(zone['wraps_start_finish'])
        json.dumps(analysis, allow_nan=False)

    def test_coasting_scrub_and_tiny_commands_do_not_count_as_braking(self):
        analysis = analyze_braking(*fixture([1, 1, 0, -.1, .1], [0, 1, 500, 500, 1.001]))
        self.assertEqual(analysis['segment_category'], ['none', 'none', 'none', 'none', 'light'])
        self.assertAlmostEqual(analysis['peak_deceleration_g'], .1)
        zone = analysis['zones'][0]
        self.assertEqual(zone['end_distance_m'], 50)
        self.assertFalse(zone['wraps_start_finish'])

    def test_zones_crossing_start_finish_are_merged(self):
        analysis = analyze_braking(*fixture([.2, .4, 0, .8, 0, .1, .9, .2]))
        self.assertEqual(len(analysis['zones']), 2)
        first, wrapped = analysis['zones']
        self.assertEqual((first['start_distance_m'], first['end_distance_m']), (30, 40))
        self.assertEqual((wrapped['start_distance_m'], wrapped['end_distance_m']), (50, 20))
        self.assertEqual(wrapped['length_m'], 50)
        self.assertEqual(wrapped['duration_s'], 24)
        self.assertTrue(wrapped['wraps_start_finish'])
        self.assertEqual(wrapped['category'], 'hard')
        self.assertEqual((wrapped['entry_speed_mps'], wrapped['exit_speed_mps']), (15, 18))

    def test_no_braking_returns_empty_zones_and_zero_peak(self):
        analysis = analyze_braking(*fixture([0, -.5, .01, .02]))
        self.assertEqual(analysis['zones'], [])
        self.assertEqual(analysis['peak_deceleration_g'], 0)
        self.assertEqual(analysis['distance_m']['none'], 40)

    def test_all_braking_is_one_full_closed_zone(self):
        analysis = analyze_braking(*fixture([.1, .4, .8, .2]))
        self.assertEqual(len(analysis['zones']), 1)
        zone = analysis['zones'][0]
        self.assertEqual((zone['start_distance_m'], zone['end_distance_m']), (0, 0))
        self.assertEqual((zone['length_m'], zone['duration_s']), (40, 10))
        self.assertTrue(zone['wraps_start_finish'])
        self.assertEqual(zone['category'], 'hard')

    def test_bad_profiles_are_rejected(self):
        track, result = fixture([0, .2, .4, .8])
        result.speed_mps = result.speed_mps[:-1]
        with self.assertRaises(ValueError):
            analyze_braking(track, result)
        track, result = fixture([0, .2, .4, float('nan')])
        with self.assertRaises(ValueError):
            analyze_braking(track, result)
        track, result = fixture([0, .2, .4, .8])
        result.segment_time_s[0] = 0
        with self.assertRaises(ValueError):
            analyze_braking(track, result)


if __name__ == '__main__':
    unittest.main()
