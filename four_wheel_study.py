"""Compare idealized four-wheel braking strategies with assumed parameters."""

import argparse
import csv
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np

from brake_model import GRAVITY_MPS2
from four_wheel_model import (
    FourWheelParameters,
    InfeasibleOperatingPoint,
    allocate_axle_balanced_braking,
    allocate_independent_braking,
    calculate_braking_yaw_range,
    calculate_four_wheel_grip,
)


STRATEGIES = (
    'Fixed 60% front, equal left/right',
    'Adjustable front/rear, equal left/right',
    'Independent wheels, zero added yaw',
    'Independent wheels, unrestricted yaw',
)


def run_envelope_study(parameters, lateral_targets, deceleration_targets):
    """Return highest feasible sampled demands, not certified global maxima.

    Scan the full grid rather than assuming feasibility is monotonic in
    deceleration. NaN means no feasible demand was found at that target.
    """
    limits = np.full((len(STRATEGIES), len(lateral_targets)), np.nan)
    yaw_at_limit = np.full_like(limits, np.nan)
    for lateral_index, lateral_acceleration in enumerate(lateral_targets):
        for deceleration in deceleration_targets:
            try:
                wheels = calculate_four_wheel_grip(parameters, deceleration, lateral_acceleration)
            except InfeasibleOperatingPoint:
                continue
            force = parameters.vehicle.vehicle_mass_kg * deceleration
            results = (
                allocate_axle_balanced_braking(wheels, force, .60),
                allocate_axle_balanced_braking(wheels, force),
                allocate_independent_braking(wheels, force, 0),
                allocate_independent_braking(wheels, force, None),
            )
            for strategy_index, allocation in enumerate(results):
                if allocation is not None and allocation.is_feasible:
                    limits[strategy_index, lateral_index] = deceleration
                    yaw_at_limit[strategy_index, lateral_index] = allocation.yaw_moment_nm
    return limits, yaw_at_limit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=Path('outputs/four_wheel'))
    args = parser.parse_args()
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parameters = FourWheelParameters()
    lateral_targets = np.linspace(0, 1.2 * GRAVITY_MPS2, 61)
    deceleration_targets = np.linspace(0, parameters.vehicle.friction_coefficient * GRAVITY_MPS2, 601)
    limits, yaw_at_limit = run_envelope_study(parameters, lateral_targets, deceleration_targets)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / 'assumed_parameters.json').write_text(json.dumps(asdict(parameters), indent=2), encoding='utf-8')
    with (args.output_dir / 'four_wheel_limits.csv').open('w', newline='', encoding='utf-8') as output:
        writer = csv.writer(output)
        writer.writerow(['strategy', 'lateral_acceleration_mps2', 'highest_feasible_sampled_deceleration_mps2', 'yaw_moment_at_sample_nm'])
        for strategy_index, strategy in enumerate(STRATEGIES):
            for lateral_index, lateral in enumerate(lateral_targets):
                writer.writerow([strategy, lateral, limits[strategy_index, lateral_index], yaw_at_limit[strategy_index, lateral_index]])

    figure, axes = plt.subplots(1, 3, figsize=(16, 5.5), layout='constrained')
    styles = ('-', '-', '--', ':')
    for index, strategy in enumerate(STRATEGIES):
        axes[0].plot(lateral_targets / GRAVITY_MPS2, limits[index] / GRAVITY_MPS2, linestyle=styles[index], label=strategy)
    axes[0].set(xlabel='Lateral acceleration demand (g)', ylabel='Highest feasible sampled deceleration (g)', title='Braking during a turn')
    axes[0].legend(fontsize=8)

    example_lateral = .6 * GRAVITY_MPS2
    example_deceleration = .8 * GRAVITY_MPS2
    example_wheels = calculate_four_wheel_grip(parameters, example_deceleration, example_lateral)
    labels = ['FL (inside)', 'FR (outside)', 'RL (inside)', 'RR (outside)']
    axes[1].bar(labels, [wheel.vertical_load_n for wheel in example_wheels], color=['tab:blue', 'tab:orange'] * 2)
    axes[1].set(ylabel='Wheel normal load (N)', title='Left turn: 0.6 g lateral, 0.8 g braking')
    axes[1].tick_params(axis='x', labelsize=8)

    yaw_ranges = []
    for deceleration in deceleration_targets:
        try:
            wheels = calculate_four_wheel_grip(parameters, deceleration, example_lateral)
            yaw_range = calculate_braking_yaw_range(wheels, parameters.vehicle.vehicle_mass_kg * deceleration)
        except InfeasibleOperatingPoint:
            yaw_range = None
        yaw_ranges.append(yaw_range if yaw_range is not None else (np.nan, np.nan))
    yaw_ranges = np.array(yaw_ranges)
    axes[2].fill_between(deceleration_targets / GRAVITY_MPS2, yaw_ranges[:, 0], yaw_ranges[:, 1], alpha=.25, label='Achievable yaw range')
    axes[2].plot(deceleration_targets / GRAVITY_MPS2, yaw_ranges[:, 0], color='tab:blue')
    axes[2].plot(deceleration_targets / GRAVITY_MPS2, yaw_ranges[:, 1], color='tab:blue')
    axes[2].axhline(0, color='black', linestyle='--', label='Zero added yaw')
    axes[2].set(xlabel='Deceleration demand (g)', ylabel='Yaw moment (N m; positive turns left)', title='Yaw authority at 0.6 g lateral')
    axes[2].legend(fontsize=8)
    for axis in axes:
        axis.grid(True, alpha=.3)
        axis.set_axisbelow(True)
    figure.suptitle('Assumed vehicle | prescribed lateral forces | constant friction | ideal force allocation')
    figure.savefig(args.output_dir / 'four_wheel_study.png', dpi=160)

    print('ASSUMED PARAMETERS (not measured car data):')
    print(json.dumps(asdict(parameters), indent=2))
    print('Highest feasible sampled deceleration (g), grid spacing 0.0025 g:')
    for lateral_g in [0, .6, 1.0]:
        index = int(np.argmin(abs(lateral_targets / GRAVITY_MPS2 - lateral_g)))
        print(f'  Lateral {lateral_g:.1f} g:')
        for strategy_index, strategy in enumerate(STRATEGIES):
            print(f'    {strategy}: {limits[strategy_index, index] / GRAVITY_MPS2:.4f} g; yaw {yaw_at_limit[strategy_index, index]:.1f} N m')
    print('Unrestricted-yaw results can rotate the car; they are not yaw-balanced braking improvements.')
    print(f'Results saved to {args.output_dir.resolve()}')
    if not args.no_show:
        plt.show()
    plt.close(figure)


if __name__ == '__main__':
    main()
