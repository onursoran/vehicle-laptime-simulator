from dataclasses import replace
import math
import unittest

from brake_hydraulics import HydraulicParameters, HydraulicConfiguration, ProportioningValve, calculate_hydraulic_braking, solve_master_force_fraction
from four_wheel_model import FourWheelParameters, calculate_four_wheel_grip, allocate_axle_balanced_braking


class BrakeHydraulicsTests(unittest.TestCase):
    def setUp(self):
        # Normalized exact test gains/areas, not assumed car specifications.
        self.parameters = HydraulicParameters(1, 1, (1, 1, 1, 1))
        self.valve = ProportioningValve(100, 1 / 3)

    def test_valve_knee_continuity_slopes_and_supply_bound(self):
        for inlet, expected in [(0, 0), (50, 50), (100, 100), (160, 120), (400, 200)]:
            self.assertAlmostEqual(self.valve.outlet_pressure_pa(inlet), expected)
        pressures = [self.valve.outlet_pressure_pa(inlet) for inlet in range(1001)]
        self.assertTrue(all(a <= b for a, b in zip(pressures, pressures[1:])))
        self.assertTrue(all(0 <= outlet <= inlet for inlet, outlet in enumerate(pressures)))
        self.assertAlmostEqual(self.valve.outlet_pressure_pa(100 + 1e-6), 100 + 1e-6 / 3)

    def test_application_and_release_use_same_quasi_static_curve(self):
        rising = [self.valve.outlet_pressure_pa(inlet) for inlet in [0, 50, 100, 200, 400]]
        falling = [self.valve.outlet_pressure_pa(inlet) for inlet in [400, 200, 100, 50, 0]]
        self.assertEqual(rising, list(reversed(falling)))

    def test_conventional_baseline_matches_existing_force_model(self):
        result = calculate_hydraulic_braking(self.parameters, 1500, .6)
        wheels = calculate_four_wheel_grip(FourWheelParameters(), 10, 0)
        baseline = allocate_axle_balanced_braking(wheels, 3000, .6)
        self.assertEqual(result.wheel_braking_force_demands_n, baseline.braking_forces_n)
        self.assertEqual(result.front_braking_fraction, .6)

    def test_master_areas_and_wheel_gains_affect_actual_force_bias(self):
        parameters = HydraulicParameters(2, 1, (2, 2, 1, 1))
        result = calculate_hydraulic_braking(parameters, 1000, .6)
        self.assertEqual(result.front_master_pressure_pa, 300)
        self.assertEqual(result.rear_master_pressure_pa, 400)
        self.assertEqual(result.wheel_braking_force_demands_n, (600, 600, 400, 400))

    def test_fixed_side_is_fixed_and_pressure_dependent(self):
        configuration = HydraulicConfiguration(valves=(self.valve, None, self.valve, None))
        low = calculate_hydraulic_braking(self.parameters, 100, .5, configuration)
        high = calculate_hydraulic_braking(self.parameters, 800, .5, configuration)
        self.assertEqual(low.wheel_pressures_pa, (50, 50, 50, 50))
        self.assertEqual(high.wheel_pressures_pa, (200, 400, 200, 400))

    def test_front_rear_setting_compensation_is_load_specific(self):
        configuration = HydraulicConfiguration(valves=(self.valve, None, None, None))
        uncorrected = calculate_hydraulic_braking(self.parameters, 1000, .6, configuration)
        self.assertNotAlmostEqual(uncorrected.front_braking_fraction, .6)
        setting = solve_master_force_fraction(self.parameters, 1000, .6, configuration)
        corrected = calculate_hydraulic_braking(self.parameters, 1000, setting, configuration)
        self.assertAlmostEqual(corrected.front_braking_fraction, .6)
        self.assertNotAlmostEqual(setting, .6)
        self.assertNotAlmostEqual(solve_master_force_fraction(self.parameters, 100, .6, configuration), setting)

    def test_fault_probes_and_circuit_independence(self):
        baseline = calculate_hydraulic_braking(self.parameters, 800, .5)
        for circuit, indices in [('front', (0, 1)), ('rear', (2, 3))]:
            result = calculate_hydraulic_braking(self.parameters, 800, .5, HydraulicConfiguration(lost_circuit=circuit))
            self.assertEqual(result.front_master_pressure_pa if circuit == 'front' else result.rear_master_pressure_pa, 0)
            for index in range(4):
                self.assertEqual(result.wheel_pressures_pa[index], 0 if index in indices else baseline.wheel_pressures_pa[index])
        blocked = HydraulicConfiguration(wheel_faults=('blocked', 'normal', 'normal', 'normal'))
        self.assertEqual(calculate_hydraulic_braking(self.parameters, 800, .5, blocked).wheel_pressures_pa, (0, 400, 400, 400))
        bypass = HydraulicConfiguration(valves=(self.valve, None, None, None), wheel_faults=('bypassed', 'normal', 'normal', 'normal'))
        self.assertEqual(calculate_hydraulic_braking(self.parameters, 800, .5, bypass).wheel_pressures_pa[0], 400)
        retained = HydraulicConfiguration(wheel_faults=('retained', 'normal', 'normal', 'normal'))
        trapped = calculate_hydraulic_braking(self.parameters, 0, .5, retained, (250, 250, 250, 250))
        self.assertEqual(trapped.wheel_pressures_pa, (250, 0, 0, 0))

    def test_invalid_and_unattainable_inputs(self):
        for knee, slope in [(-1, .5), (0, -1), (0, 1.1), (math.nan, .5)]:
            with self.assertRaises(ValueError):
                ProportioningValve(knee, slope)
        with self.assertRaises(ValueError):
            HydraulicParameters(0, 1, (1, 1, 1, 1))
        with self.assertRaises(ValueError):
            calculate_hydraulic_braking(self.parameters, -1, .5)
        with self.assertRaises(ValueError):
            solve_master_force_fraction(self.parameters, 0, .5)
        with self.assertRaises(ValueError):
            solve_master_force_fraction(replace(self.parameters, wheel_force_gain_n_per_pa=(0, 0, 1, 1)), 100, .5)


if __name__ == '__main__':
    unittest.main()
