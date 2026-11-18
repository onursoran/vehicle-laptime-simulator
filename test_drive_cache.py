from dataclasses import replace
import unittest

from car_profile import make_vehicle
from research_setup import make_research_strategies
from vehicle_equilibrium import _solve_drive, _solve_mode, solve_operating_point


class DriveCacheTests(unittest.TestCase):
    def setUp(self):
        _solve_drive.cache_clear()

    def test_cached_drive_matches_every_strategy_exactly(self):
        parameters = make_vehicle()
        for args in [(20, 0, 0, 0, 12), (15, 2, .02, .0001, 12), (33, 0, .1, .001, 3), (8, -.2, -.04, -.0003, 4)]:
            cached = _solve_drive(parameters, *args)
            for strategy in make_research_strategies(profile='illustrative'):
                expected = _solve_mode(parameters, strategy, *args, 'drive')
                self.assertEqual(cached, expected)

    def test_full_branch_selection_matches_uncached_reference(self):
        parameters = make_vehicle()
        for strategy in make_research_strategies(profile='illustrative'):
            for args in [(20, 0, 0, 0, 12), (15, -5, .02, 0, 12), (8, 0, -.06, .003, 3), (30, 0, .1, 0, 3)]:
                preferred = 'drive' if args[1] >= 0 else 'brake'
                expected = _solve_mode(parameters, strategy, *args, preferred)
                if not expected.is_feasible:
                    alternative = _solve_mode(parameters, strategy, *args, 'brake' if preferred == 'drive' else 'drive')
                    if alternative.is_feasible:
                        expected = alternative
                self.assertEqual(solve_operating_point(parameters, strategy, *args), expected)

    def test_cache_excludes_brake_strategy_but_includes_vehicle_and_width(self):
        parameters = make_vehicle()
        strategies = make_research_strategies(profile='illustrative')
        for strategy in strategies:
            self.assertTrue(solve_operating_point(parameters, strategy, 20, 0, 0, 0, 12).is_feasible)
        self.assertEqual(_solve_drive.cache_info().misses, 1)
        self.assertEqual(_solve_drive.cache_info().hits, 5)
        _solve_drive(replace(parameters, drive_power_w=35000), 20, 0, 0, 0, 12)
        _solve_drive(parameters, 20, 0, 0, 0, 4)
        self.assertEqual(_solve_drive.cache_info().misses, 3)


if __name__ == '__main__':
    unittest.main()
