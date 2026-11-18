import http.client
from http.server import ThreadingHTTPServer
import json
import threading
import unittest
from unittest.mock import patch

from simulator import LocalSimulatorServer, SimulatorHandler, validate_inputs


class SimulatorApiTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = LocalSimulatorServer(('127.0.0.1', 0), SimulatorHandler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def request(self, method, path, body=None, headers=None):
        connection = http.client.HTTPConnection('127.0.0.1', self.server.server_port, timeout=10)
        connection.request(method, path, body, headers or {})
        response = connection.getresponse()
        result = response.status, response.read(), response.getheader('Content-Type')
        connection.close()
        return result

    def test_health_and_browser_assets_are_served(self):
        status, payload, _ = self.request('GET', '/health')
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(payload)['status'], 'ok')
        self.assertIn('montreal', json.loads(payload)['tracks'])
        for path, marker in [('/', b'Left/right assistance strength'), ('/simulator.js', b'/api/jobs')]:
            status, payload, _ = self.request('GET', path)
            self.assertEqual(status, 200)
            self.assertIn(marker, payload)

    def test_second_server_cannot_silently_share_the_same_port(self):
        with self.assertRaises(OSError):
            LocalSimulatorServer(('127.0.0.1', self.server.server_port), SimulatorHandler)

    def test_only_explicit_assets_are_exposed(self):
        for path in ['/brake_model.py', '/../brake_model.py', '/outputs/lap_time/oval/comparison.json']:
            self.assertEqual(self.request('GET', path)[0], 404)

    def test_config_and_preview_do_not_require_a_lap_calculation(self):
        status, payload, _ = self.request('GET', '/api/config')
        config = json.loads(payload)
        self.assertEqual(status, 200)
        self.assertEqual(config['default_inputs']['profile'], 'illustrative')
        self.assertEqual(config['default_inputs']['configurations'], 'active')
        with patch('simulation_service.LapTimeSolver', side_effect=AssertionError('Preview invoked solver')):
            status, payload, _ = self.request('POST', '/api/preview', '{"track":"monza"}', {'Content-Type':'application/json'})
        preview = json.loads(payload)
        self.assertEqual(status, 200)
        self.assertGreater(preview['track']['length_m'], 5700)
        self.assertEqual(preview['vehicle']['wheels']['vehicle']['vehicle_mass_kg'], 300)
        self.assertNotIn('results', preview)

    def test_background_job_routes_and_unknown_job(self):
        with patch('simulator.JOBS') as manager:
            manager.submit.return_value = {'job_id':'sample', 'status':'queued'}
            status, payload, _ = self.request('POST', '/api/jobs', '{}', {'Content-Type':'application/json'})
            self.assertEqual(status, 202)
            self.assertEqual(json.loads(payload)['job_id'], 'sample')
            manager.get.return_value = {'job_id':'sample', 'status':'running', 'version':1}
            self.assertEqual(self.request('GET', '/api/jobs/sample?after=1')[0], 200)
            manager.get.assert_called_once_with('sample', 1)
            manager.cancel.return_value = {'job_id':'sample', 'status':'cancelled'}
            status, payload, _ = self.request('POST', '/api/jobs/sample/cancel', '{}', {'Content-Type':'application/json'})
            self.assertEqual(status, 200)
            self.assertEqual(json.loads(payload)['status'], 'cancelled')
            manager.get.side_effect = KeyError('missing')
            self.assertEqual(self.request('GET', '/api/jobs/missing')[0], 404)
        self.assertEqual(self.request('GET', '/api/jobs/sample?after=bad')[0], 400)

    def test_valid_inputs_reach_the_comparison_backend(self):
        with patch('simulator.simulation_result', return_value={'test': 'ok'}) as backend:
            status, payload, _ = self.request('POST', '/api/simulate', json.dumps({'width': 4, 'front_bias': 65, 'strength': 40}), {'Content-Type': 'application/json'})
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(payload), {'test': 'ok'})
        self.assertEqual(backend.call_args.args[0]['front_bias'], 65)

    def test_bad_input_and_cross_origin_posts_are_rejected(self):
        for body in ['{', '{"width": NaN}', '{"strength": 200}', '{"track": "secret"}', '{"unknown": 1}']:
            self.assertEqual(self.request('POST', '/api/simulate', body, {'Content-Type': 'application/json'})[0], 400)
        self.assertEqual(self.request('POST', '/api/simulate', '{}', {'Content-Type': 'text/plain'})[0], 415)
        self.assertEqual(self.request('POST', '/api/simulate', '{}', {'Content-Type': 'application/json', 'Origin': 'https://example.com'})[0], 403)

    def test_infeasible_configuration_is_an_explicit_error(self):
        with patch('simulator.simulation_result', side_effect=ValueError('Road too narrow')):
            status, payload, _ = self.request('POST', '/api/simulate', '{}', {'Content-Type': 'application/json'})
        self.assertEqual(status, 422)
        self.assertEqual(json.loads(payload)['error'], 'Road too narrow')

    def test_numeric_boolean_and_bad_direction_are_rejected(self):
        for data in [{'width': True}, {'mirrored': 'false'}, {'front_track': 0}, {'spacing': float('inf')}]:
            with self.assertRaises(ValueError):
                validate_inputs(data)

    def test_real_circuits_use_full_size_defaults_and_allow_width_override(self):
        for name in ('monza', 'montreal', 'catalunya'):
            inputs = validate_inputs({'track': name})
            self.assertEqual(inputs['width'], 12)
            self.assertEqual(inputs['spacing'], 4)
            self.assertEqual(validate_inputs({'track': name, 'width': 18})['width'], 18)
        self.assertEqual(validate_inputs({})['width'], 3)
        for invalid in [{'track': []}, {'track': {}}, {'width': 21}]:
            with self.assertRaises(ValueError):
                validate_inputs(invalid)


if __name__ == '__main__':
    unittest.main()
