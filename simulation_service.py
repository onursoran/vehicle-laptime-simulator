"""Fast geometry previews and cancellable, cached background lap comparisons."""

from collections import OrderedDict
from dataclasses import asdict, replace
import math
import queue
import threading
import time
import uuid

from car_profile import CAR_PROFILE_NAMES, make_vehicle, vehicle_profile_metadata
from lap_time import LapTimeSolver
from lap_time_study import comparison_data
from real_tracks import REAL_TRACKS, TRACK_NAMES, make_track
from research_setup import make_research_strategies, select_research_strategies


DEFAULT_PROFILE = 'illustrative'
RANGES = dict(width=(2, 20), front_track=(.9, 1.8), rear_track=(.9, 1.8), front_bias=(0, 100), strength=(0, 100), yaw_limit=(0, 1000), friction=(.5, 2), spacing=(1, 8), valve_knee_bar=(1, 80))
COMPUTE_LOCK = threading.Lock()
_CACHE_LOCK = threading.Lock()
_LAP_CACHE = OrderedDict()


def profile_defaults(profile):
    vehicle = make_vehicle(profile)
    return dict(profile=profile, configurations='active', track='oval', mirrored=False,
                width=3.0, front_track=vehicle.wheels.front_track_m,
                rear_track=vehicle.wheels.rear_track_m, front_bias=60.0,
                strength=50.0, yaw_limit=120.0,
                friction=vehicle.wheels.vehicle.friction_coefficient, spacing=2.0,
                valve_knee_bar=20.0)


DEFAULT_INPUTS = profile_defaults(DEFAULT_PROFILE)


def configuration_data():
    profiles = {}
    for name in CAR_PROFILE_NAMES:
        metadata = vehicle_profile_metadata(name)
        vehicle = make_vehicle(name)
        description = (f'{vehicle.wheels.vehicle.vehicle_mass_kg:g} kg including the reference driver; '
                       f'{vehicle.wheels.vehicle.wheelbase_m:.2f} m wheelbase; '
                       f'{vehicle.maximum_speed_mps * 3.6:g} km/h model speed ceiling. '
                       + 'Original research placeholders.')
        profiles[name] = dict(id=name, label=metadata.get('label', name),
                              description=description,
                              default_inputs=profile_defaults(name), metadata=metadata)
    return dict(default_inputs=DEFAULT_INPUTS, profiles=profiles)


def validate_inputs(data):
    if not isinstance(data, dict) or set(data) - set(DEFAULT_INPUTS):
        raise ValueError('Unknown simulation inputs.')
    profile = data.get('profile', DEFAULT_PROFILE)
    if not isinstance(profile, str) or profile not in CAR_PROFILE_NAMES:
        raise ValueError('Unknown vehicle profile.')
    inputs = {**profile_defaults(profile), **data}
    if not isinstance(inputs['track'], str) or inputs['track'] not in TRACK_NAMES:
        raise ValueError('Unknown test track.')
    if inputs['configurations'] not in ('active', 'all', 'fiddle'):
        raise ValueError('Choose active, all, or fiddle brake configurations.')
    if inputs['track'] in REAL_TRACKS:
        for name, default in (('width', 12.0), ('spacing', 4.0)):
            if name not in data:
                inputs[name] = default
    if type(inputs['mirrored']) is not bool:
        raise ValueError('mirrored must be a boolean.')
    for name, (lower, upper) in RANGES.items():
        value = inputs[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError(f'{name} must be between {lower} and {upper}.')
    return inputs


def simulation_setup(inputs):
    parameters = make_vehicle(inputs['profile'])
    vehicle = replace(parameters.wheels.vehicle, friction_coefficient=inputs['friction'])
    parameters = replace(parameters, wheels=replace(parameters.wheels, vehicle=vehicle,
                         front_track_m=inputs['front_track'], rear_track_m=inputs['rear_track']))
    track = make_track(inputs['track'], inputs['spacing'], inputs['width'])
    if inputs['mirrored']:
        track = track.mirrored()
    return track, parameters


def track_payload(track):
    return dict(name=track.name, metadata=track.metadata, length_m=track.length_m,
                **{name: getattr(track, name).tolist() for name in
                   ('distance_m', 'x_m', 'y_m', 'heading_rad', 'curvature_per_m', 'width_m')})


def preview_result(data):
    """Only geometry and car dimensions: no lap solver, locks, or job queue."""
    inputs = validate_inputs(data)
    track, parameters = simulation_setup(inputs)
    return dict(track=track_payload(track), vehicle=asdict(parameters), inputs=inputs,
                profile_metadata=vehicle_profile_metadata(inputs['profile']))


def clear_lap_cache():
    with _CACHE_LOCK:
        _LAP_CACHE.clear()


def _lap(track, parameters, strategy, inputs, cancel_check):
    # The strategy contains only its own settings. Changing assistance does
    # not invalidate the conventional lap, or any fixed passive-valve lap.
    key = (inputs['track'], inputs['mirrored'], inputs['spacing'], inputs['width'], parameters, strategy)
    with _CACHE_LOCK:
        if key in _LAP_CACHE:
            _LAP_CACHE.move_to_end(key)
            return _LAP_CACHE[key], True
    cancel_check()
    result = LapTimeSolver(parameters, strategy, cancel_check=cancel_check).solve(track)
    cancel_check()
    with _CACHE_LOCK:
        _LAP_CACHE[key] = result
        _LAP_CACHE.move_to_end(key)
        while len(_LAP_CACHE) > 32:
            _LAP_CACHE.popitem(last=False)
    return result, False


def compute_simulation(data, *, progress=None, publish=None, cancel_check=lambda: None):
    inputs = validate_inputs(data)
    track, parameters = simulation_setup(inputs)
    strategies = make_research_strategies(inputs['front_bias'] / 100, inputs['strength'] / 100,
                                         inputs['yaw_limit'], inputs['valve_knee_bar'] * 1e5,
                                         profile=inputs['profile'])
    strategies = select_research_strategies(strategies, inputs['configurations'])
    while not COMPUTE_LOCK.acquire(timeout=.1):
        cancel_check()
    try:
        results, successful, failures, timings = [], [], [], []
        started = time.perf_counter()
        for index, strategy in enumerate(strategies):
            cancel_check()
            if progress:
                progress(index, len(strategies), strategy.name)
            start = time.perf_counter()
            try:
                lap, cached = _lap(track, parameters, strategy, inputs, cancel_check)
            except (ValueError, RuntimeError) as error:
                cancel_check()
                if index == 0:
                    raise
                failures.append(dict(name=strategy.name, error=str(error)))
            else:
                results.append(lap)
                successful.append(strategy)
                timings.append(dict(name=strategy.name, cached=cached, calculation_s=time.perf_counter() - start))
            output = comparison_data(track, parameters, successful, results)
            output.update(inputs=inputs, profile_metadata=vehicle_profile_metadata(inputs['profile']),
                          failures=list(failures), performance=dict(configurations=timings.copy(), elapsed_s=time.perf_counter() - started))
            if publish:
                publish(output, index + 1, len(strategies))
        return output
    finally:
        COMPUTE_LOCK.release()


class JobManager:
    """One compute worker; preview/config/health requests remain independent."""

    def __init__(self, processor=compute_simulation):
        self.processor = processor
        self.lock = threading.Lock()
        self.jobs = OrderedDict()
        self.queue = queue.Queue(maxsize=4)
        self.worker = None
        self.stopped = threading.Event()

    def submit(self, inputs):
        inputs = validate_inputs(inputs)
        with self.lock:
            # Reuse exact completed/in-flight requests, avoiding duplicate tabs.
            for job in reversed(self.jobs.values()):
                if job['inputs'] == inputs and job['status'] in ('queued', 'running', 'complete'):
                    return self._snapshot(job, -1)
            if self.queue.full():
                raise RuntimeError('Calculation queue is full. Cancel an older comparison first.')
            while len(self.jobs) >= 8:
                finished = next((key for key, value in self.jobs.items() if value['status'] in ('complete', 'failed', 'cancelled')), None)
                if finished is None:
                    raise RuntimeError('Too many active comparisons.')
                self.jobs.pop(finished)
            identifier = uuid.uuid4().hex
            job = dict(job_id=identifier, inputs=inputs, status='queued', completed=0,
                       total=6 if inputs['configurations'] == 'all' else 2,
                       phase='Waiting for calculation', version=0, created=time.perf_counter(),
                       cancel=threading.Event())
            self.jobs[identifier] = job
            self.queue.put_nowait(identifier)
            if self.worker is None:
                self.worker = threading.Thread(target=self._work, daemon=True, name='lap-comparison')
                self.worker.start()
            return self._snapshot(job, -1)

    @staticmethod
    def _snapshot(job, after):
        snapshot = {name: job[name] for name in ('job_id', 'status', 'completed', 'total', 'phase', 'version')}
        snapshot['elapsed_s'] = job.get('finished', time.perf_counter()) - job['created']
        if 'error' in job:
            snapshot['error'] = job['error']
        if job['version'] > after and 'result' in job:
            snapshot['result'] = job['result']
        return snapshot

    def get(self, identifier, after=-1):
        with self.lock:
            if identifier not in self.jobs:
                raise KeyError(identifier)
            return self._snapshot(self.jobs[identifier], after)

    def cancel(self, identifier):
        with self.lock:
            job = self.jobs[identifier]
            if job['status'] in ('queued', 'running'):
                job['cancel'].set()
                job.update(status='cancelled', phase='Cancelled', finished=time.perf_counter())
            return self._snapshot(job, -1)

    def _work(self):
        while not self.stopped.is_set():
            try:
                identifier = self.queue.get(timeout=.2)
            except queue.Empty:
                continue
            with self.lock:
                job = self.jobs.get(identifier)
                if job is None or job['cancel'].is_set():
                    self.queue.task_done()
                    continue
                job.update(status='running', phase='Preparing vehicle and track')
            def cancel_check():
                if job['cancel'].is_set() or self.stopped.is_set():
                    raise RuntimeError('Calculation cancelled.')
            def progress(completed, total, phase):
                with self.lock:
                    job.update(completed=completed, total=total, phase=phase)
            def publish(result, completed, total):
                cancel_check()
                with self.lock:
                    job.update(result=result, completed=completed, total=total, version=job['version'] + 1)
            try:
                output = self.processor(job['inputs'], progress=progress, publish=publish, cancel_check=cancel_check)
                cancel_check()
                with self.lock:
                    cancel_check()
                    job.update(status='complete', phase='Complete', result=output,
                               completed=job['total'], finished=time.perf_counter(), version=job['version'] + 1)
            except Exception as error:
                with self.lock:
                    cancelled = job['cancel'].is_set() or self.stopped.is_set()
                    job.update(status='cancelled' if cancelled else 'failed',
                               phase='Cancelled' if cancelled else 'Calculation failed',
                               error=str(error), finished=time.perf_counter())
            finally:
                self.queue.task_done()

    def close(self):
        self.stopped.set()
        with self.lock:
            for job in self.jobs.values():
                job['cancel'].set()
        if self.worker is not None:
            self.worker.join(timeout=2)


JOBS = JobManager()
