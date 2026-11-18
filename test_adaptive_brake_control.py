from dataclasses import replace
import math
import unittest

from adaptive_brake_control import (
    AdaptiveBrakeSettings, BrakeControlRequest, BrakeControlState,
    calculate_adaptive_braking,
)
from four_wheel_model import FourWheelParameters, allocate_axle_balanced_braking, calculate_four_wheel_grip


class AdaptiveBrakeTests(unittest.TestCase):
    def setUp(self):
        self.parameters = FourWheelParameters()
        self.request = BrakeControlRequest(7, .6, 10, 5, .1)

    def run_control(self, settings, request=None, previous=None, **kwargs):
        return calculate_adaptive_braking(self.parameters, request or self.request, settings, previous, time_step_s=.02, **kwargs)

    def test_zero_assistance_reproduces_conventional_baseline(self):
        result = self.run_control(AdaptiveBrakeSettings(strength_fraction=0))
        wheels = calculate_four_wheel_grip(self.parameters, 7, 5)
        baseline = allocate_axle_balanced_braking(wheels, 2100, .6)
        self.assertEqual(result.allocation.braking_forces_n, baseline.braking_forces_n)
        self.assertAlmostEqual(result.allocation.yaw_moment_nm, 0)

    def test_strength_is_fraction_of_capacity_matched_differential(self):
        for strength in [0, .25, .5, 1]:
            result = self.run_control(AdaptiveBrakeSettings(strength_fraction=strength))
            for actual, peak in zip(result.applied_asymmetry_fractions, result.peak_asymmetry_fractions):
                self.assertAlmostEqual(actual, strength * peak)
            self.assertTrue(result.meets_capacity_and_supply_limits)

    def test_front_rear_preference_preserved_for_every_strength(self):
        for fraction in [0, .3, .6, .8, 1]:
            for strength in [0, .5, 1]:
                request = replace(self.request, front_braking_fraction=fraction)
                result = self.run_control(AdaptiveBrakeSettings(strength_fraction=strength), request)
                self.assertAlmostEqual(result.allocation.total_braking_force_n, 2100)
                self.assertAlmostEqual(result.allocation.front_braking_fraction, fraction)

    def test_changing_front_rear_preference_recalculates_yaw_limited_allocation(self):
        settings = AdaptiveBrakeSettings(strength_fraction=1, maximum_yaw_moment_nm=40)
        results = [self.run_control(settings, replace(self.request, front_braking_fraction=fraction)) for fraction in [.5, .8]]
        for result in results:
            self.assertLessEqual(abs(result.allocation.yaw_moment_nm), 40 + 1e-8)
            self.assertAlmostEqual(result.allocation.front_braking_fraction, result.front_braking_fraction_requested)
        self.assertNotEqual(results[0].allocation.braking_forces_n, results[1].allocation.braking_forces_n)

    def test_front_rear_modulation_selection(self):
        for axle in ['front', 'rear']:
            result = self.run_control(AdaptiveBrakeSettings(strength_fraction=1, modulated_axles=axle))
            inactive = 1 if axle == 'front' else 0
            self.assertEqual(result.applied_asymmetry_fractions[inactive], 0)
            self.assertNotEqual(result.applied_asymmetry_fractions[1 - inactive], 0)

    def test_straight_line_and_countersteer_gates(self):
        settings = AdaptiveBrakeSettings(strength_fraction=1, lateral_deadband_mps2=.1, steering_deadband_rad=.01, minimum_speed_mps=2)
        for request in [replace(self.request, lateral_acceleration_mps2=0, steering_angle_rad=0), replace(self.request, steering_angle_rad=-.1), replace(self.request, forward_speed_mps=1), replace(self.request, lateral_acceleration_mps2=.05)]:
            result = self.run_control(settings, request)
            self.assertEqual(result.applied_asymmetry_fractions, (0, 0))
            self.assertAlmostEqual(result.allocation.yaw_moment_nm, 0)

    def test_mirror_turns_and_repeatability(self):
        settings = AdaptiveBrakeSettings(strength_fraction=.5)
        left = self.run_control(settings)
        right = self.run_control(settings, replace(self.request, lateral_acceleration_mps2=-5, steering_angle_rad=-.1))
        for index, mirror in enumerate([1, 0, 3, 2]):
            self.assertAlmostEqual(left.allocation.braking_forces_n[index], right.allocation.braking_forces_n[mirror])
        self.assertAlmostEqual(left.allocation.yaw_moment_nm, -right.allocation.yaw_moment_nm)
        self.assertEqual(left, self.run_control(settings))

    def test_fraction_slew_and_lateral_filter(self):
        settings = AdaptiveBrakeSettings(strength_fraction=1, maximum_asymmetry_change_per_s=.1, lateral_filter_time_constant_s=.1)
        first = self.run_control(settings)
        second = self.run_control(settings, previous=first.state)
        self.assertGreater(first.state.filtered_lateral_acceleration_mps2, 0)
        self.assertLess(first.state.filtered_lateral_acceleration_mps2, 5)
        for q, next_q in zip(first.applied_asymmetry_fractions, second.applied_asymmetry_fractions):
            self.assertLessEqual(abs(q), .002 + 1e-12)
            self.assertLessEqual(abs(next_q - q), .002 + 1e-12)
        self.assertFalse(first.authority_limit_overrode_slew)

    def test_authority_and_yaw_limits_override_slew_explicitly(self):
        previous = BrakeControlState(5, .2, .2)
        settings = AdaptiveBrakeSettings(strength_fraction=0, maximum_asymmetry_change_per_s=.01)
        result = self.run_control(settings, previous=previous)
        self.assertEqual(result.applied_asymmetry_fractions, (0, 0))
        self.assertTrue(result.authority_limit_overrode_slew)
        result = self.run_control(AdaptiveBrakeSettings(strength_fraction=1, maximum_yaw_moment_nm=0))
        self.assertAlmostEqual(result.allocation.yaw_moment_nm, 0)

    def test_infeasibility_is_not_hidden_by_changing_bias(self):
        request = replace(self.request, deceleration_demand_mps2=12)
        result = self.run_control(AdaptiveBrakeSettings(strength_fraction=0), request)
        self.assertFalse(result.meets_capacity_and_supply_limits)
        self.assertAlmostEqual(result.allocation.front_braking_fraction, .6)
        self.assertGreater(result.peak_tire_utilization, 1)
        self.assertTrue(any('lower braking demand' in reason for reason in result.limiting_reasons))
        result = self.run_control(AdaptiveBrakeSettings(strength_fraction=1), wheel_force_supply_limits_n=(1, 1, 1, 1))
        self.assertFalse(result.meets_capacity_and_supply_limits)

    def test_wheel_lift_and_raw_lateral_limit_are_reported(self):
        settings = AdaptiveBrakeSettings(strength_fraction=1, lateral_filter_time_constant_s=100)
        for request in [replace(self.request, deceleration_demand_mps2=40), replace(self.request, lateral_acceleration_mps2=25)]:
            result = self.run_control(settings, request)
            self.assertIsNone(result.allocation)
            self.assertFalse(result.meets_capacity_and_supply_limits)

    def test_yaw_acceleration_and_lateral_reserves(self):
        result = self.run_control(AdaptiveBrakeSettings(strength_fraction=1))
        self.assertAlmostEqual(result.yaw_acceleration_radps2, result.allocation.yaw_moment_nm / 150)
        for reserve in result.lateral_force_reserves_n:
            self.assertGreaterEqual(reserve, -1e-7)
        altered = replace(self.parameters, yaw_moment_of_inertia_kgm2=300)
        other = calculate_adaptive_braking(altered, self.request, AdaptiveBrakeSettings(strength_fraction=1), time_step_s=.02)
        self.assertAlmostEqual(result.yaw_acceleration_radps2, 2 * other.yaw_acceleration_radps2)

    def test_zero_braking_and_extreme_driver_fractions(self):
        result = self.run_control(AdaptiveBrakeSettings(strength_fraction=1), replace(self.request, deceleration_demand_mps2=0))
        self.assertEqual(result.allocation.total_braking_force_n, 0)
        self.assertIsNone(result.allocation.front_braking_fraction)
        self.assertEqual(result.applied_asymmetry_fractions, (0, 0))

    def test_invalid_settings_and_requests(self):
        for changes in [dict(strength_fraction=1.1), dict(maximum_yaw_moment_nm=-1), dict(lateral_filter_time_constant_s=math.nan), dict(modulated_axles='outside'), dict(maximum_asymmetry_change_per_s=-1)]:
            with self.assertRaises(ValueError):
                AdaptiveBrakeSettings(**changes)
        for changes in [dict(front_braking_fraction=-1), dict(steering_angle_rad=math.nan), dict(deceleration_demand_mps2=-1)]:
            with self.assertRaises(ValueError):
                replace(self.request, **changes)
        with self.assertRaises(ValueError):
            calculate_adaptive_braking(self.parameters, self.request, AdaptiveBrakeSettings(), time_step_s=0)


if __name__ == '__main__':
    unittest.main()
