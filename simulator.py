"""Local browser interface for the lap-time comparisons. No cloud service."""

import argparse
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import socket
from urllib.parse import parse_qs, urlsplit

from lap_time_study import MODEL_FINGERPRINT
from real_tracks import TRACK_NAMES
from simulation_service import (DEFAULT_INPUTS, JOBS, RANGES, compute_simulation,
                                configuration_data, preview_result, validate_inputs)


ROOT = Path(__file__).resolve().parent


class LocalSimulatorServer(ThreadingHTTPServer):
    # Windows otherwise allows two reused listening sockets on this port,
    # leaving requests routed unpredictably to an old Python process.
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def simulation_result(data):
    """Legacy synchronous API; the browser uses /api/jobs instead."""
    return compute_simulation(data)


class SimulatorHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Local request paths only; avoid noisy per-frame logs.
        pass

    def respond(self, status, payload, content_type='application/json; charset=utf-8'):
        content = json.dumps(payload, allow_nan=False).encode('utf-8') if isinstance(payload, (dict, list)) else payload
        compressed = len(content) > 16384 and 'gzip' in self.headers.get('Accept-Encoding', '')
        if compressed:
            content = gzip.compress(content, compresslevel=1)
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(content)))
        if compressed:
            self.send_header('Content-Encoding', 'gzip')
        self.send_header('Vary', 'Accept-Encoding')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        try:
            self.wfile.write(content)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def do_GET(self):
        parsed = urlsplit(self.path)
        route = parsed.path
        if route == '/':
            self.respond(200, (ROOT / 'simulator.html').read_bytes(), 'text/html; charset=utf-8')
        elif route == '/simulator.js':
            self.respond(200, (ROOT / 'simulator.js').read_bytes(), 'text/javascript; charset=utf-8')
        elif route == '/health':
            self.respond(200, dict(status='ok', service='local lap-time comparison', source_fingerprint=MODEL_FINGERPRINT, tracks=list(TRACK_NAMES)))
        elif route == '/api/config':
            self.respond(200, configuration_data())
        elif route.startswith('/api/jobs/'):
            identifier = route.removeprefix('/api/jobs/')
            try:
                after = int(parse_qs(parsed.query).get('after', ['-1'])[0])
                self.respond(200, JOBS.get(identifier, after))
            except KeyError:
                self.respond(404, dict(error='Comparison job not found.'))
            except ValueError:
                self.respond(400, dict(error='Invalid job version.'))
        else:
            self.respond(404, dict(error='Not found.'))

    def do_POST(self):
        cancel_route = self.path.startswith('/api/jobs/') and self.path.endswith('/cancel')
        if self.path not in ('/api/simulate', '/api/preview', '/api/jobs') and not cancel_route:
            self.respond(404, dict(error='Not found.'))
            return
        port = self.server.server_port
        if self.headers.get('Origin') not in (None, f'http://127.0.0.1:{port}', f'http://localhost:{port}'):
            self.respond(403, dict(error='This simulator accepts same-origin local requests only.'))
            return
        if self.headers.get_content_type() != 'application/json':
            self.respond(415, dict(error='Send application/json.'))
            return
        try:
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 8192:
                raise ValueError('Invalid request size.')
            data = json.loads(self.rfile.read(length))
            inputs = validate_inputs(data) if not cancel_route else data
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self.respond(400, dict(error=str(error)))
            return
        try:
            if cancel_route:
                identifier = self.path.removeprefix('/api/jobs/').removesuffix('/cancel')
                self.respond(200, JOBS.cancel(identifier))
            elif self.path == '/api/preview':
                self.respond(200, preview_result(inputs))
            elif self.path == '/api/jobs':
                self.respond(202, JOBS.submit(inputs))
            else:
                self.respond(200, simulation_result(inputs))
        except KeyError:
            self.respond(404, dict(error='Comparison job not found.'))
        except (ValueError, RuntimeError) as error:
            self.respond(422, dict(error=str(error)))
        except Exception:
            self.respond(500, dict(error='Simulation failed. Run the command-line study for diagnostic details.'))
            raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error('port must be between 0 and 65535.')
    server = LocalSimulatorServer(('127.0.0.1', args.port), SimulatorHandler)
    print(f'Open http://127.0.0.1:{server.server_port} in your browser. Ctrl+C stops the simulator.', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        JOBS.close()
        server.server_close()


if __name__ == '__main__':
    main()
