"""Compare brake systems on fixed driving lines with explicit car profiles."""

import argparse
import csv
import hashlib
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import re
import time

import numpy as np

from lap_time import LapTimeSolver
from braking_analysis import analyze_braking
from car_profile import CAR_PROFILE_NAMES, vehicle_profile_metadata
from research_setup import make_research_strategies, make_research_vehicle, select_research_strategies
from real_tracks import DATA_DIRECTORY, REAL_TRACKS, TRACK_NAMES, make_track


def source_fingerprint():
    digest = hashlib.sha256()
    for name in ('adaptive_brake_control.py', 'fiddle_brake.py', 'brake_hydraulics.py', 'brake_model.py', 'four_wheel_model.py', 'tire_model.py', 'vehicle_model.py', 'vehicle_equilibrium.py', 'lap_time.py', 'track_model.py', 'real_tracks.py', 'braking_analysis.py', 'research_setup.py', 'car_profile.py'):
        digest.update(name.encode('utf-8'))
        digest.update((Path(__file__).parent / name).read_bytes())
    for path in sorted(DATA_DIRECTORY.glob('*.csv')):
        digest.update(path.name.encode('utf-8'))
        digest.update(path.read_bytes())
    return digest.hexdigest()[:20]


MODEL_FINGERPRINT = source_fingerprint()


def slug(value):
    return re.sub('[^a-z0-9]+', '-', value.lower()).strip('-')


def compare_track(track, parameters, strategies, progress=None):
    results = []
    for strategy in strategies:
        if progress:
            progress(f'{track.name}: {strategy.name}')
        start = time.perf_counter()
        result = LapTimeSolver(parameters, strategy).solve(track)
        results.append(result)
        if progress:
            progress(f'  {result.lap_time_s:.3f} s; {result.iterations} passes; computed in {time.perf_counter() - start:.1f} s')
    return results


def comparison_data(track, parameters, strategies, results):
    baseline = results[0].lap_time_s
    return dict(
        model='Quasi-steady, fixed centerline; source values and remaining assumptions are exported; no transient-stability or validated real-car performance claim.',
        source_fingerprint=MODEL_FINGERPRINT, numpy_version=np.__version__,
        track=dict(name=track.name, metadata=track.metadata, length_m=track.length_m, distance_m=track.distance_m.tolist(), x_m=track.x_m.tolist(), y_m=track.y_m.tolist(), heading_rad=track.heading_rad.tolist(), curvature_per_m=track.curvature_per_m.tolist(), width_m=track.width_m.tolist()),
        vehicle=asdict(parameters), strategies=[asdict(strategy) for strategy in strategies],
        results=[dict(
            name=result.strategy_name, lap_time_s=result.lap_time_s, delta_to_baseline_s=result.lap_time_s - baseline,
            speed_mps=result.speed_mps.tolist(), segment_time_s=result.segment_time_s.tolist(), acceleration_mps2=result.tangential_acceleration_mps2.tolist(),
            steering_angle_rad=[point.steering_angle_rad for point in result.segment_midpoints],
            sideslip_angle_rad=[point.sideslip_angle_rad for point in result.segment_midpoints],
            wheel_loads_n=[point.wheel_loads_n for point in result.segment_midpoints],
            tire_longitudinal_forces_n=[point.tire_longitudinal_forces_n for point in result.segment_midpoints],
            auxiliary_braking_forces_n=[point.auxiliary_braking_forces_n for point in result.segment_midpoints],
            tire_lateral_forces_n=[point.tire_lateral_forces_n for point in result.segment_midpoints],
            wheel_pressures_pa=[point.wheel_pressures_pa for point in result.segment_midpoints],
            pushrod_force_n=[point.pushrod_force_n for point in result.segment_midpoints],
            drive_force_n=[point.drive_force_n for point in result.segment_midpoints],
            body_longitudinal_acceleration_mps2=[point.body_longitudinal_acceleration_mps2 for point in result.segment_midpoints],
            body_lateral_acceleration_mps2=[point.body_lateral_acceleration_mps2 for point in result.segment_midpoints],
            yaw_acceleration_radps2=[point.yaw_acceleration_radps2 for point in result.segment_midpoints],
            peak_tire_utilization=result.maximum_tire_utilization,
            maximum_equilibrium_error=result.maximum_equilibrium_error,
            maximum_abs_steering_rad=result.maximum_abs_steering_rad,
            braking=analyze_braking(track, result),
        ) for result in results],
    )


def save_comparison(directory, track, parameters, strategies, results, *, plots=True, profile=None):
    directory.mkdir(parents=True, exist_ok=True)
    data = comparison_data(track, parameters, strategies, results)
    if profile is not None:
        data['profile_metadata'] = vehicle_profile_metadata(profile)
    (directory / 'comparison.json').write_text(json.dumps(data, indent=2, allow_nan=False), encoding='utf-8')
    summary = []
    for result in results:
        braking = analyze_braking(track, result)
        row = dict(strategy=result.strategy_name, lap_time_s=result.lap_time_s, delta_to_baseline_s=result.lap_time_s - results[0].lap_time_s, peak_speed_kph=float(max(result.speed_mps) * 3.6), maximum_steering_deg=math.degrees(result.maximum_abs_steering_rad), maximum_tire_utilization=result.maximum_tire_utilization, maximum_equilibrium_error=result.maximum_equilibrium_error, solver_passes=result.iterations)
        summary.append(row)
        row.update(peak_braking_deceleration_g=braking['peak_deceleration_g'], braking_zones=len(braking['zones']))
        row.update({f'{category}_braking_distance_m': braking['distance_m'][category] for category in ('light', 'medium', 'hard')})
        (directory / (slug(result.strategy_name) + '-braking.json')).write_text(json.dumps(braking, indent=2, allow_nan=False), encoding='utf-8')
        with (directory / (slug(result.strategy_name) + '.csv')).open('w', newline='', encoding='utf-8') as output:
            fields = ['distance_start_m', 'segment_time_s', 'speed_start_mps', 'speed_end_mps', 'tangential_acceleration_mps2', 'curvature_start_per_m', 'width_m', 'steering_midpoint_deg', 'sideslip_midpoint_deg', 'yaw_moment_midpoint_nm', 'yaw_acceleration_midpoint_radps2', 'equilibrium_error_midpoint', 'tire_utilization_midpoint']
            fields += ['braking_category'] + [f'{wheel}_{quantity}' for wheel in ('fl', 'fr', 'rl', 'rr') for quantity in ('load_n', 'longitudinal_force_n', 'lateral_force_n', 'auxiliary_braking_force_n')]
            writer = csv.DictWriter(output, fieldnames=fields)
            writer.writeheader()
            for index, point in enumerate(result.segment_midpoints):
                row = dict(distance_start_m=float(track.distance_m[index]), segment_time_s=float(result.segment_time_s[index]), speed_start_mps=float(result.speed_mps[index]), speed_end_mps=float(result.speed_mps[(index + 1) % len(result.speed_mps)]), tangential_acceleration_mps2=float(result.tangential_acceleration_mps2[index]), curvature_start_per_m=float(track.curvature_per_m[index]), width_m=float(track.width_m[index]), steering_midpoint_deg=math.degrees(point.steering_angle_rad), sideslip_midpoint_deg=math.degrees(point.sideslip_angle_rad), yaw_moment_midpoint_nm=point.yaw_moment_nm, yaw_acceleration_midpoint_radps2=point.yaw_acceleration_radps2, equilibrium_error_midpoint=point.equilibrium_error, tire_utilization_midpoint=point.peak_tire_utilization)
                for wheel_index, wheel in enumerate(('fl', 'fr', 'rl', 'rr')):
                    row[f'{wheel}_load_n'] = point.wheel_loads_n[wheel_index]
                    row[f'{wheel}_longitudinal_force_n'] = point.tire_longitudinal_forces_n[wheel_index]
                    row[f'{wheel}_lateral_force_n'] = point.tire_lateral_forces_n[wheel_index]
                    row[f'{wheel}_auxiliary_braking_force_n'] = point.auxiliary_braking_forces_n[wheel_index]
                row['braking_category'] = braking['segment_category'][index]
                writer.writerow(row)
    with (directory / 'summary.csv').open('w', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    if plots:
        plot_comparison(directory, track, results, profile=profile)
    return data


def plot_comparison(directory, track, results, *, profile=None):
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(13, 8), layout='constrained')
    left_x = track.x_m - np.sin(track.heading_rad) * track.width_m / 2
    left_y = track.y_m + np.cos(track.heading_rad) * track.width_m / 2
    right_x = track.x_m + np.sin(track.heading_rad) * track.width_m / 2
    right_y = track.y_m - np.cos(track.heading_rad) * track.width_m / 2
    for x, y in ((left_x, left_y), (right_x, right_y)):
        axes[0, 0].plot(np.r_[x, x[0]], np.r_[y, y[0]], color='gray', linewidth=.8)
    axes[0, 0].plot(np.r_[track.x_m, track.x_m[0]], np.r_[track.y_m, track.y_m[0]], linestyle='--', linewidth=.8, label='Fixed driving line')
    axes[0, 0].set(xlabel='x (m)', ylabel='y (m)', title=f'{track.name}: {track.length_m:.1f} m, {track.width_m[0]:.1f} m road')
    axes[0, 0].set_aspect('equal', adjustable='datalim')
    distance = np.r_[track.distance_m, track.length_m]
    baseline_time = np.r_[0, np.cumsum(results[0].segment_time_s)]
    for result in results:
        label = f'{result.strategy_name}: {result.lap_time_s:.3f} s'
        axes[0, 1].plot(distance, np.r_[result.speed_mps, result.speed_mps[0]] * 3.6, label=label)
        axes[1, 0].plot(distance, np.r_[0, np.cumsum(result.segment_time_s)] - baseline_time, label=result.strategy_name)
        axes[1, 1].plot(track.distance_m, [math.degrees(point.steering_angle_rad) for point in result.segment_midpoints], label=result.strategy_name)
    axes[0, 1].set(xlabel='Distance (m)', ylabel='Speed (km/h)', title='Audited closed-lap speed profiles')
    axes[1, 0].set(xlabel='Distance (m)', ylabel='Time difference vs baseline (s)', title='Positive means slower than conventional')
    axes[1, 1].set(xlabel='Distance (m)', ylabel='Road-wheel steering (degrees)', title='Quasi-steady steering demand')
    for axis in axes.flat:
        axis.grid(True, alpha=.25)
        axis.legend(fontsize=7)
    vehicle_label = 'Illustrative vehicle'
    figure.suptitle(vehicle_label + ' | equilibrium and hydraulic limits | idealized offline driver')
    figure.savefig(directory / 'lap_comparison.png', dpi=160)
    plt.close(figure)
    # Actual model braking demand, not a hand-painted circuit braking map.
    from matplotlib.collections import LineCollection
    figure, axes = plt.subplots(1, 2, figsize=(13, 5), layout='constrained')
    colors = dict(none='#9ca3af', light='#d6a31a', medium='#e87525', hard='#cc3347')
    result = results[0]
    braking = analyze_braking(track, result)
    xy = np.c_[track.x_m, track.y_m]
    segments = np.stack((xy, np.roll(xy, -1, axis=0)), axis=1)
    axes[0].add_collection(LineCollection(segments, colors=[colors[c] for c in braking['segment_category']], linewidths=2))
    axes[0].autoscale()
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].set(title='Conventional braking intensity', xlabel='x (m)', ylabel='y (m)')
    for category in ('none', 'light', 'medium', 'hard'):
        axes[0].plot([], [], color=colors[category], label=f'{category}: {braking["distance_m"][category]:.0f} m')
    axes[0].legend(fontsize=8)
    for result in results:
        axes[1].plot(track.distance_m, -result.tangential_acceleration_mps2 / 9.81, linewidth=.9, label=result.strategy_name)
    axes[1].axhline(.3, color=colors['medium'], linestyle=':', linewidth=.7)
    axes[1].axhline(.7, color=colors['hard'], linestyle=':', linewidth=.7)
    axes[1].set(title='Calculated path deceleration (negative = acceleration)', xlabel='Distance (m)', ylabel='Deceleration (g)')
    axes[1].legend(fontsize=7)
    figure.suptitle(track.name + ' | ' + vehicle_label)
    figure.savefig(directory / 'braking_zones.png', dpi=160)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--track', choices=(*TRACK_NAMES, 'all', 'real'), default='oval', help='all: three synthetic comparison tracks; real: all bundled circuits')
    parser.add_argument('--profile', choices=CAR_PROFILE_NAMES, default='illustrative')
    parser.add_argument('--width', type=float, default=None, help='Uniform corridor override, default 3 m synthetic / 12 m real')
    parser.add_argument('--spacing', type=float, default=None, help='Requested spacing, default 2 m synthetic / 4 m real')
    parser.add_argument('--front-track', type=float, default=1.2)
    parser.add_argument('--rear-track', type=float, default=1.2)
    parser.add_argument('--front-bias', type=float, default=60.0)
    parser.add_argument('--strength', type=float, default=50.0)
    parser.add_argument('--yaw-limit', type=float, default=120.0)
    parser.add_argument('--valve-knee-bar', type=float, default=20.0)
    parser.add_argument('--include-mirrors', action='store_true')
    parser.add_argument('--configurations', choices=('all', 'baseline', 'active', 'fiddle'), default='all')
    parser.add_argument('--output-dir', type=Path, default=Path('outputs/lap_time'))
    args = parser.parse_args()
    for name in ('width', 'spacing', 'front_track', 'rear_track', 'valve_knee_bar'):
        if getattr(args, name) is not None and (not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0):
            parser.error(f'{name} must be positive.')
    if not 0 <= args.front_bias <= 100 or not 0 <= args.strength <= 100:
        parser.error('Bias and assistance percentages must be between 0 and 100.')
    import matplotlib
    matplotlib.use('Agg')
    parameters = make_research_vehicle(args.profile)
    parameters = replace(parameters, wheels=replace(parameters.wheels, front_track_m=args.front_track, rear_track_m=args.rear_track))
    strategies = make_research_strategies(args.front_bias / 100, args.strength / 100, args.yaw_limit, args.valve_knee_bar * 1e5, profile=args.profile)
    strategies = select_research_strategies(strategies, args.configurations)
    names = ('oval', 'figure-eight', 'technical') if args.track == 'all' else tuple(REAL_TRACKS) if args.track == 'real' else (args.track,)
    for name in names:
        track = make_track(name, args.spacing, args.width)
        for current in (track, track.mirrored()) if args.include_mirrors else (track,):
            results = compare_track(current, parameters, strategies, lambda message: print(message, flush=True))
            directory = args.output_dir / slug(current.name)
            save_comparison(directory, current, parameters, strategies, results, profile=args.profile)
            print(f'Saved {directory.resolve()}', flush=True)


if __name__ == '__main__':
    main()
