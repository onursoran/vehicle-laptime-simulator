import unittest

from car_profile import CAR_PROFILE_NAMES, make_hydraulics, make_vehicle, vehicle_profile_metadata


class CarProfileTests(unittest.TestCase):
    def test_illustrative_profile_and_provenance_remain_explicit(self):
        self.assertEqual(CAR_PROFILE_NAMES, ('illustrative',))
        vehicle = make_vehicle('illustrative')
        self.assertEqual(vehicle.wheels.vehicle.vehicle_mass_kg, 300)
        self.assertEqual(vehicle.maximum_speed_mps, 25)
        self.assertEqual(make_hydraulics('illustrative').wheel_force_gain_n_per_pa, (.0002,) * 4)
        metadata = vehicle_profile_metadata()
        self.assertEqual(metadata['id'], 'illustrative')
        self.assertIsNone(metadata['source'])
        self.assertTrue(metadata['assumptions'])

    def test_unknown_profile_is_rejected(self):
        for function in (make_vehicle, make_hydraulics, vehicle_profile_metadata):
            with self.assertRaises(ValueError):
                function('unknown')


if __name__ == '__main__':
    unittest.main()
