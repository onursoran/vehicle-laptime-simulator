from dataclasses import replace
import unittest

from car_profile import make_vehicle
from four_wheel_model import calculate_four_wheel_contacts
from research_setup import make_research_strategies
from vehicle_equilibrium import solve_operating_point

# Generic example values (not any real car): forces at an 80 km/h reference speed.
REFERENCE_SPEED = 80 / 3.6
DOWNFORCE_N = 800.0
DRAG_N = 150.0
FRONT_FRACTION = .45


class AerodynamicLoadTests(unittest.TestCase):
    def test_downforce_adds_axle_loads_without_changing_inertial_transfer(self):
        parameters = make_vehicle()
        weight = parameters.wheels.vehicle.vehicle_mass_kg * 9.81
        base = calculate_four_wheel_contacts(parameters.wheels, -6, 4)
        aero = calculate_four_wheel_contacts(parameters.wheels, -6, 4, downforce_n=DOWNFORCE_N, aerodynamic_front_fraction=FRONT_FRACTION)
        self.assertAlmostEqual(sum(w.vertical_load_n for w in aero), weight + DOWNFORCE_N)
        for index in range(4):
            expected = DOWNFORCE_N * (FRONT_FRACTION if index < 2 else 1 - FRONT_FRACTION) / 2
            self.assertAlmostEqual(aero[index].vertical_load_n - base[index].vertical_load_n, expected)
        for wheels in (base, aero):
            self.assertGreater(wheels[1].vertical_load_n, wheels[0].vertical_load_n)
            self.assertGreater(wheels[3].vertical_load_n, wheels[2].vertical_load_n)

    def test_reference_speed_drag_and_downforce_balance(self):
        parameters = replace(
            make_vehicle(),
            drag_force_per_speed_squared=DRAG_N / REFERENCE_SPEED**2,
            downforce_per_speed_squared_n_s2_per_m2=DOWNFORCE_N / REFERENCE_SPEED**2,
            aerodynamic_front_fraction=FRONT_FRACTION,
        )
        weight = parameters.wheels.vehicle.vehicle_mass_kg * 9.81
        strategy = make_research_strategies(profile='illustrative')[0]
        point = solve_operating_point(parameters, strategy, REFERENCE_SPEED, 0, 0, road_width_m=12)
        self.assertTrue(point.is_feasible, point.reason)
        self.assertAlmostEqual(sum(point.wheel_loads_n), weight + DOWNFORCE_N, places=6)
        self.assertAlmostEqual(point.drive_force_n, DRAG_N, places=4)
        self.assertLessEqual(point.equilibrium_error, parameters.equilibrium_tolerance)

    def test_invalid_downforce_inputs_are_rejected(self):
        parameters = make_vehicle()
        for kwargs in [dict(downforce_n=-1), dict(downforce_n=float('nan')), dict(aerodynamic_front_fraction=1.1)]:
            with self.assertRaises(ValueError):
                calculate_four_wheel_contacts(parameters.wheels, 0, 0, **kwargs)
        with self.assertRaises(ValueError):
            replace(parameters, downforce_per_speed_squared_n_s2_per_m2=-1)


if __name__ == '__main__':
    unittest.main()
