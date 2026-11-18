"""Reproducible geometry-cache benchmark against the previous evaluation path.

Uses fixed illustrative parameters, independent of the selected car profile.
No result is accepted unless complete numeric lap outputs match exactly.
"""

import argparse
import cProfile
from dataclasses import asdict
import json
import math
from pathlib import Path
import statistics
import time

import numpy as np

from brake_hydraulics import HydraulicParameters, solve_master_force_fraction
from lap_time import LapTimeSolver
from real_tracks import make_track
from vehicle_equilibrium import BrakeStrategy, PerformanceParameters, _solve_drive


class PreviousGeometrySolver(LapTimeSolver):
    """Previous _segment_points implementation, kept only for the benchmark."""

    def _segment_points(self, track, index, speed_start, speed_end):
        next_index = (index + 1) % len(track.distance_m)
        ds = track.segment_lengths_m[index]
        acceleration = (speed_end**2 - speed_start**2) / (2 * ds)
        midpoint_speed = math.sqrt((speed_start**2 + speed_end**2) / 2)
        gradients = track.curvature_gradient_per_m2
        values = (
            (speed_start, track.curvature_per_m[index], gradients[index], track.width_m[index]),
            (midpoint_speed, (track.curvature_per_m[index] + track.curvature_per_m[next_index]) / 2, (gradients[index] + gradients[next_index]) / 2, min(track.width_m[index], track.width_m[next_index])),
            (speed_end, track.curvature_per_m[next_index], gradients[next_index], track.width_m[next_index]),
        )
        points = []
        for speed, curvature, gradient, width in values:
            point = self.point(speed, acceleration, curvature, gradient, width)
            points.append(point)
            if not point.is_feasible:
                return False, tuple(points)
        return True, tuple(points)


def fixed_benchmark_inputs():
    parameters = PerformanceParameters()
    area = math.pi * .01905**2 / 4
    hydraulics = HydraulicParameters(area, area, (.0002,) * 4)
    strategy = BrakeStrategy(
        'Conventional front/rear', front_braking_fraction=.6, hydraulics=hydraulics,
        front_master_force_fraction=solve_master_force_fraction(hydraulics, 2000, .6),
        maximum_pushrod_force_n=4000,
    )
    return parameters, strategy


def assert_identical(reference, candidate):
    for name in ('speed_mps', 'segment_time_s', 'tangential_acceleration_mps2'):
        np.testing.assert_array_equal(getattr(reference, name), getattr(candidate, name))
    for name in ('lap_time_s', 'iterations', 'maximum_equilibrium_error', 'maximum_tire_utilization', 'maximum_abs_steering_rad', 'segment_midpoints'):
        if getattr(reference, name) != getattr(candidate, name):
            raise AssertionError(f'Changed lap result: {name}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', choices=('monza', 'montreal'), default='monza')
    parser.add_argument('--spacing', type=float, default=4)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('outputs/performance/solver-benchmark.json'))
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('At least one timing repeat is required.')
    parameters, strategy = fixed_benchmark_inputs()
    track = make_track(args.track, args.spacing)
    timings = {'previous': [], 'optimized': []}
    reference = None
    for iteration in range(args.repeats):
        # Alternate ordering to reduce systematic warm-up/CPU bias.
        variants = [('previous', PreviousGeometrySolver), ('optimized', LapTimeSolver)]
        if iteration % 2:
            variants.reverse()
        for label, solver_type in variants:
            _solve_drive.cache_clear()
            start = time.perf_counter()
            result = solver_type(parameters, strategy).solve(track)
            elapsed = time.perf_counter() - start
            timings[label].append(elapsed)
            print(f'{label}: {elapsed:.3f} s, lap {result.lap_time_s:.12f} s', flush=True)
            if reference is None:
                reference = result
            else:
                assert_identical(reference, result)
    medians = {label: statistics.median(values) for label, values in timings.items()}
    report = dict(track=track.name, spacing_m=args.spacing, samples=len(track.distance_m),
                  timings_s=timings, median_s=medians,
                  speedup=medians['previous'] / medians['optimized'],
                  exact_result_match=True, lap_time_s=reference.lap_time_s,
                  vehicle=asdict(parameters), strategy=asdict(strategy))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')
    if args.profile:
        for label, solver_type in (('previous', PreviousGeometrySolver), ('optimized', LapTimeSolver)):
            _solve_drive.cache_clear()
            profiler = cProfile.Profile()
            profiler.runcall(solver_type(parameters, strategy).solve, track)
            profiler.dump_stats(str(args.output.with_suffix(f'.{label}.prof')))
    print(json.dumps({key: report[key] for key in ('median_s', 'speedup', 'exact_result_match')}, indent=2))


if __name__ == '__main__':
    main()
