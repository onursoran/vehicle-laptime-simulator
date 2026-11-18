"""Audit saved comparisons against current source and declared constraints."""

import argparse
import json
import math
from pathlib import Path

from lap_time_study import source_fingerprint


def audit_comparison(path):
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('source_fingerprint') != source_fingerprint():
        raise ValueError(f'{path}: comparison predates the current physical model; regenerate it.')
    parameters = data['vehicle']
    if len(data['results']) != len(data['strategies']):
        raise ValueError('Results and strategy definitions do not match.')
    count = len(data['track']['distance_m'])
    for result, strategy in zip(data['results'], data['strategies']):
        if not math.isfinite(result['lap_time_s']) or result['lap_time_s'] <= 0:
            raise ValueError('Invalid lap time.')
        if abs(sum(result['segment_time_s']) - result['lap_time_s']) > 1e-8:
            raise ValueError('Segment times do not sum to lap time.')
        for name in ('speed_mps', 'segment_time_s', 'acceleration_mps2', 'steering_angle_rad', 'sideslip_angle_rad', 'wheel_loads_n', 'tire_longitudinal_forces_n', 'tire_lateral_forces_n'):
            if len(result[name]) != count:
                raise ValueError(f'{name} does not match track sampling.')
        if result['maximum_equilibrium_error'] > parameters['equilibrium_tolerance']:
            raise ValueError('Unconverged equilibrium was reported as a lap.')
        if result['peak_tire_utilization'] > 1 + 1e-7:
            raise ValueError('Tire grip exceeded.')
        if result['maximum_abs_steering_rad'] > parameters['maximum_steering_angle_rad'] + 1e-9:
            raise ValueError('Steering limit exceeded.')
        if min(result['speed_mps']) <= 0 or max(result['speed_mps']) > parameters['maximum_speed_mps'] + 1e-8:
            raise ValueError('Invalid speed.')
        for index, loads in enumerate(result['wheel_loads_n']):
            if min(loads) < 0:
                raise ValueError('Negative normal load accepted.')
            expected = parameters['wheels']['vehicle']['vehicle_mass_kg'] * 9.81
            midpoint_speed_squared = (result['speed_mps'][index]**2 + result['speed_mps'][(index + 1) % count]**2) / 2
            expected += parameters.get('downforce_per_speed_squared_n_s2_per_m2', 0) * midpoint_speed_squared
            if abs(sum(loads) - expected) > 1e-5:
                raise ValueError('Vertical load conservation failed.')
        if max(result['pushrod_force_n']) > strategy['maximum_pushrod_force_n'] + 1e-6:
            raise ValueError('Hydraulic source bound exceeded.')
        if strategy['kind'] == 'adaptive':
            for forces in result['tire_longitudinal_forces_n']:
                if min(forces) < 0:
                    actual = sum(forces[:2]) / sum(forces)
                    if abs(actual - strategy['front_braking_fraction']) > 1e-8:
                        raise ValueError('Adaptive allocation changed front/rear preference.')
    return dict(track=data['track']['name'], configurations=len(data['results']), samples=count,
                baseline_s=data['results'][0]['lap_time_s'],
                independent_delta_s=data['results'][-1]['delta_to_baseline_s'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, default=Path('outputs/lap_time'))
    parser.add_argument('--comparison', type=Path, nargs='+', help='Audit specific comparison.json files, including real circuits, instead of the synthetic suite')
    args = parser.parse_args()
    if args.comparison:
        rows = [audit_comparison(path) for path in args.comparison]
        print(json.dumps(dict(source_fingerprint=source_fingerprint(), audited_laps=sum(row['configurations'] for row in rows), comparisons=rows), indent=2))
        return
    rows = []
    for name in ('oval', 'oval-mirrored', 'figure-eight', 'figure-eight-mirrored', 'technical', 'technical-mirrored'):
        path = args.directory / name / 'comparison.json'
        if not path.is_file():
            raise ValueError(f'Missing comparison: {path}')
        rows.append(audit_comparison(path))
    if any(row['configurations'] != 6 for row in rows):
        raise ValueError('Full comparison audit requires all six configurations on each track/direction.')
    refinements = []
    for spacing in ('2', '1', '0-5'):
        path = args.directory / f'refinement-{spacing}' / 'oval' / 'comparison.json'
        if path.is_file():
            refinements.append(dict(spacing_m=float(spacing.replace('-', '.')), **audit_comparison(path)))
    report = dict(source_fingerprint=source_fingerprint(), audited_laps=sum(row['configurations'] for row in rows), comparisons=rows, oval_refinements=refinements)
    (args.directory / 'validation_report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
