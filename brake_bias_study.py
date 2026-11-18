"""Run a fast brake-bias study independently of the G-G grid search."""

import argparse
import csv
from pathlib import Path

import numpy as np

from brake_model import (
    BrakingParameters,
    GRAVITY_MPS2,
    calculate_ideal_front_braking_fraction,
    calculate_straight_line_braking_state,
    find_maximum_deceleration_for_brake_bias,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-show', action='store_true', help='Save plots without opening windows.')
    parser.add_argument('--output-dir', type=Path, default=Path('outputs/brake_bias'))
    args = parser.parse_args()
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parameters = BrakingParameters()
    grip_ceiling_mps2 = parameters.friction_coefficient * GRAVITY_MPS2
    contact_ceiling_mps2 = min(grip_ceiling_mps2, parameters.rear_lift_deceleration_mps2)
    ideal_peak_fraction = calculate_ideal_front_braking_fraction(parameters, contact_ceiling_mps2)
    fractions = np.unique(np.append(np.linspace(0, 1, 201), ideal_peak_fraction))
    limits = [find_maximum_deceleration_for_brake_bias(parameters, fraction) for fraction in fractions]
    args.output_dir.mkdir(parents=True, exist_ok=True)

    with (args.output_dir / 'brake_bias_limits.csv').open('w', newline='', encoding='utf-8') as output:
        writer = csv.writer(output)
        writer.writerow(['front_braking_fraction', 'maximum_deceleration_mps2', 'maximum_deceleration_g', 'limiting_constraint'])
        for fraction, limit in zip(fractions, limits):
            writer.writerow([fraction, limit.maximum_deceleration_mps2, limit.maximum_deceleration_mps2 / GRAVITY_MPS2, limit.limiting_constraint])

    decelerations = np.linspace(0, contact_ceiling_mps2, 151)
    ideal_fractions = [calculate_ideal_front_braking_fraction(parameters, demand) for demand in decelerations]
    figure, axes = plt.subplots(1, 3, figsize=(15, 4.8), layout='constrained')
    axes[0].plot(decelerations / GRAVITY_MPS2, np.array(ideal_fractions) * 100)
    axes[0].set(xlabel='Deceleration demand (g)', ylabel='Ideal front braking force (%)', title='Equal axle utilization')

    axes[1].plot(fractions * 100, [limit.maximum_deceleration_mps2 / GRAVITY_MPS2 for limit in limits])
    axes[1].axhline(contact_ceiling_mps2 / GRAVITY_MPS2, linestyle='--', color='black', label='Model ceiling')
    axes[1].scatter([ideal_peak_fraction * 100], [contact_ceiling_mps2 / GRAVITY_MPS2], color='tab:red', zorder=3)
    axes[1].set(xlabel='Fixed front braking force (%)', ylabel='Maximum deceleration (g)', title='First saturation or axle lift')
    axes[1].legend()

    example_fraction = 0.60
    states = [calculate_straight_line_braking_state(parameters, demand, example_fraction) for demand in decelerations]
    axes[2].plot(decelerations / GRAVITY_MPS2, [state.front_tire_utilization for state in states], label='Front axle')
    axes[2].plot(decelerations / GRAVITY_MPS2, [state.rear_tire_utilization for state in states], label='Rear axle')
    axes[2].axhline(1, color='black', linestyle='--', label='Grip limit')
    axes[2].set(xlabel='Deceleration demand (g)', ylabel='Demand / grip capacity', title='60% front force: requested utilization')
    axes[2].legend()
    for axis in axes:
        axis.grid(True, alpha=0.3)
    figure.suptitle('Straight-line brake bias | illustrative parameters | no aero or brake-torque limits')
    figure.savefig(args.output_dir / 'brake_bias_study.png', dpi=160)

    print('Illustrative parameters: 300 kg, wheelbase 1.8 m, CG height 0.30 m, friction coefficient 1.5')
    print('Front tire-force split | Maximum deceleration | Limiting constraint')
    for fraction in [0.50, 0.60, ideal_peak_fraction, 0.80]:
        limit = find_maximum_deceleration_for_brake_bias(parameters, fraction)
        print(f'{fraction:>20.2%} | {limit.maximum_deceleration_mps2 / GRAVITY_MPS2:>18.3f} g | {limit.limiting_constraint}')
    print(f'Results saved to {args.output_dir.resolve()}')
    if not args.no_show:
        plt.show()
    plt.close(figure)


if __name__ == '__main__':
    main()
