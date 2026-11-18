from dataclasses import replace
import threading
import time
import unittest
from unittest.mock import patch

from simulation_service import (JobManager, _lap, clear_lap_cache, compute_simulation,
                                preview_result, simulation_setup, validate_inputs)
from car_profile import make_vehicle
from research_setup import make_research_strategies


class SimulationServiceTests(unittest.TestCase):
    def setUp(self):
        clear_lap_cache()

    def test_brake_strength_change_reuses_baseline_but_not_active_lap(self):
        inputs = validate_inputs({'profile':'illustrative'})
        track, parameters = simulation_setup(inputs)
        baseline, *_, active = make_research_strategies()
        with patch('simulation_service.LapTimeSolver') as solver:
            solver.return_value.solve.return_value = 'verified lap'
            self.assertFalse(_lap(track, parameters, baseline, inputs, lambda:None)[1])
            changed = {**inputs, 'strength':75, 'yaw_limit':80, 'valve_knee_bar':25}
            self.assertTrue(_lap(track, parameters, baseline, changed, lambda:None)[1])
            self.assertFalse(_lap(track, parameters, active, inputs, lambda:None)[1])
            changed_active = replace(active, assistance=replace(active.assistance, strength_fraction=.75))
            self.assertFalse(_lap(track, parameters, changed_active, changed, lambda:None)[1])
            self.assertEqual(solver.return_value.solve.call_count, 3)

    def test_preview_works_while_comparison_worker_is_blocked(self):
        entered, release = threading.Event(), threading.Event()
        def processor(inputs, **kwargs):
            entered.set()
            release.wait(timeout=2)
            return {'completed': True}
        manager = JobManager(processor)
        try:
            manager.submit({})
            self.assertTrue(entered.wait(timeout=1))
            with patch('simulation_service.LapTimeSolver', side_effect=AssertionError('No solve in preview')):
                preview = preview_result({'track':'montreal'})
            self.assertGreater(preview['track']['length_m'], 4300)
        finally:
            release.set()
            manager.close()

    def test_job_progress_partial_results_versions_and_cancellation(self):
        entered, release = threading.Event(), threading.Event()
        def processor(inputs, progress, publish, cancel_check):
            progress(0, 2, 'Conventional')
            publish({'results':[{'name':'baseline'}]}, 1, 2)
            entered.set()
            release.wait(timeout=2)
            cancel_check()
            return {'results':[{'name':'baseline'}, {'name':'active'}]}
        manager = JobManager(processor)
        try:
            submitted = manager.submit({})
            self.assertTrue(entered.wait(timeout=1))
            identifier = submitted['job_id']
            snapshot = manager.get(identifier)
            self.assertEqual(snapshot['completed'], 1)
            self.assertEqual(len(snapshot['result']['results']), 1)
            self.assertNotIn('result', manager.get(identifier, snapshot['version']))
            self.assertEqual(manager.cancel(identifier)['status'], 'cancelled')
        finally:
            release.set()
            manager.close()
        self.assertEqual(manager.get(identifier)['status'], 'cancelled')

    def test_completed_identical_job_is_reused_and_failure_is_explicit(self):
        done = threading.Event()
        calls = []
        def processor(inputs, **kwargs):
            calls.append(inputs)
            done.set()
            return {'result':'ok'}
        manager = JobManager(processor)
        try:
            first = manager.submit({})
            self.assertTrue(done.wait(timeout=1))
            deadline = time.monotonic() + 1
            while manager.get(first['job_id'])['status'] != 'complete' and time.monotonic() < deadline:
                time.sleep(.005)
            second = manager.submit({})
            self.assertEqual(second['job_id'], first['job_id'])
            self.assertEqual(second['status'], 'complete')
            self.assertEqual(len(calls), 1)
        finally:
            manager.close()
        def failing(inputs, **kwargs):
            raise ValueError('No feasible lap')
        manager = JobManager(failing)
        try:
            job = manager.submit({})
            deadline = time.monotonic() + 1
            while manager.get(job['job_id'])['status'] not in ('failed', 'complete') and time.monotonic() < deadline:
                time.sleep(.005)
            snapshot = manager.get(job['job_id'])
            self.assertEqual(snapshot['status'], 'failed')
            self.assertEqual(snapshot['error'], 'No feasible lap')
        finally:
            manager.close()

    def test_default_comparison_conserves_loads_and_independent_bias(self):
        result = compute_simulation({'spacing':8})
        self.assertEqual(len(result['results']), 2)
        self.assertFalse(result['failures'])
        self.assertEqual(result['profile_metadata']['id'], 'illustrative')
        mass = make_vehicle().wheels.vehicle.vehicle_mass_kg
        for profile in result['results']:
            self.assertLessEqual(profile['peak_tire_utilization'], 1 + 1e-7)
            self.assertLessEqual(profile['maximum_equilibrium_error'], 2e-6)
            speed = profile['speed_mps']
            for index, loads in enumerate(profile['wheel_loads_n']):
                speed_squared = (speed[index]**2 + speed[(index + 1) % len(speed)]**2) / 2
                expected = mass * 9.81 + result['vehicle']['downforce_per_speed_squared_n_s2_per_m2'] * speed_squared
                self.assertAlmostEqual(sum(loads), expected, places=6)
        independent = result['results'][-1]
        for forces in independent['tire_longitudinal_forces_n']:
            if sum(forces) < -1:
                self.assertAlmostEqual(sum(forces[:2]) / sum(forces), .6)


if __name__ == '__main__':
    unittest.main()
