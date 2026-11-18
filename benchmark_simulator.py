"""Exercise the running local app, recording preview latency and job progress."""

import argparse
import gzip
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen


def request(base, path, body=None):
    payload = None if body is None else json.dumps(body).encode()
    headers = {'Accept-Encoding': 'gzip'}
    if payload is not None:
        headers['Content-Type'] = 'application/json'
    start = time.perf_counter()
    with urlopen(Request(base + path, payload, headers), timeout=30) as response:
        data = response.read()
        if response.headers.get('Content-Encoding') == 'gzip':
            data = gzip.decompress(data)
    return json.loads(data), time.perf_counter() - start


def run_job(base, inputs, *, probe_preview=False):
    start = time.perf_counter()
    job, post_time = request(base, '/api/jobs', inputs)
    version, first_profile, data = -1, None, None
    preview_while_running = None
    if probe_preview:
        _, preview_while_running = request(base, '/api/preview', {'track':'catalunya'})
    deadline = time.monotonic() + 300
    while True:
        if 'result' in job:
            data = job['result']
            if first_profile is None:
                first_profile = time.perf_counter() - start
        version = job['version']
        if job['status'] in ('complete', 'failed', 'cancelled'):
            break
        if time.monotonic() > deadline:
            request(base, '/api/jobs/' + job['job_id'] + '/cancel', {})
            raise TimeoutError('Comparison exceeded five minutes.')
        time.sleep(.1)
        job, _ = request(base, '/api/jobs/' + job['job_id'] + '?after=' + str(version))
    if job['status'] != 'complete' or not data or data.get('failures'):
        raise RuntimeError(str(job.get('error', data.get('failures') if data else job)))
    return dict(inputs=inputs, request_s=post_time, first_profile_s=first_profile,
                total_s=time.perf_counter() - start, preview_during_job_s=preview_while_running,
                performance=data['performance'],
                laps=[dict(name=value['name'], lap_time_s=value['lap_time_s'],
                           peak_tire_utilization=value['peak_tire_utilization'],
                           maximum_equilibrium_error=value['maximum_equilibrium_error']) for value in data['results']]), data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', default='http://127.0.0.1:8765')
    parser.add_argument('--output', type=Path, default=Path('outputs/performance/app.json'))
    args = parser.parse_args()
    health, _ = request(args.base, '/health')
    previews = []
    for name in health['tracks']:
        preview, elapsed = request(args.base, '/api/preview', {'track':name})
        assert 'results' not in preview
        previews.append(dict(track=name, seconds=elapsed, points=len(preview['track']['distance_m'])))
    runs = []
    for index, name in enumerate(('monza', 'montreal', 'catalunya')):
        run, data = run_job(args.base, {'track':name}, probe_preview=index == 0)
        runs.append(run)
        print(f'{name}: preview {next(row["seconds"] for row in previews if row["track"] == name):.3f}s, first profile {run["first_profile_s"]:.2f}s, complete {run["total_s"]:.2f}s', flush=True)
        if name == 'montreal':
            fixture = Path('outputs/lap_time/.browser-test-data.json')
            fixture.parent.mkdir(parents=True, exist_ok=True)
            fixture.write_text(json.dumps(data, allow_nan=False), encoding='utf-8')
    changed, _ = run_job(args.base, {'track':'monza', 'strength':40})
    assert changed['performance']['configurations'][0]['cached']
    assert not changed['performance']['configurations'][1]['cached']
    repeated, _ = run_job(args.base, {'track':'monza', 'strength':40})
    report = dict(source_fingerprint=health['source_fingerprint'], previews=previews,
                  circuit_runs=runs, changed_assistance=changed, repeated_request_s=repeated['total_s'],
                  note='Observed local timings; preview includes HTTP transfer/decompression. First profile time has 100 ms polling resolution. No browser layout or frame-rate measurement.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    print(f'Saved {args.output}; repeated exact request {repeated["total_s"]:.3f}s', flush=True)


if __name__ == '__main__':
    main()
