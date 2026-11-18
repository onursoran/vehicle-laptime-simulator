"""Repeatable input-signal studies of assistance, yaw limits, and valve curves.

These are prescribed maneuver inputs, not integrated trajectories or laps.
All controller thresholds below are explicit research settings, not validated
vehicle safety limits or measured actuator characteristics.
"""

import argparse
import csv
from dataclasses import asdict, replace
import json
import math
from pathlib import Path

import numpy as np

from adaptive_brake_control import AdaptiveBrakeSettings, BrakeControlRequest, calculate_adaptive_braking
from brake_hydraulics import ProportioningValve
from brake_model import GRAVITY_MPS2
from four_wheel_model import FourWheelParameters


RESEARCH_SETTINGS = AdaptiveBrakeSettings(
    maximum_asymmetry_fraction=.4,
    maximum_yaw_moment_nm=120,
    maximum_asymmetry_change_per_s=.8,
    lateral_filter_time_constant_s=.1,
    lateral_deadband_mps2=.1,
    steering_deadband_rad=.005,
    minimum_speed_mps=2,
    longitudinal_capacity_margin_fraction=.05,
)


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as output:
        writer = csv.DictWriter(output, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_maneuver(parameters, settings, times, fixed_front_fraction=None):
    lateral_g = np.interp(times, [0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 12], [0, 0, .6, .6, .4, 0, -.6, -.6, -.4, 0, 0])
    braking_g = np.interp(times, np.arange(13), [0, .8, .8, .7, .5, .1, 0, .65, .65, .3, 0, .7, 0])
    front_fraction = np.interp(times, [0, 4, 5, 8, 9, 12], [.6, .6, .68, .68, .58, .58])
    if fixed_front_fraction is not None:
        front_fraction = np.full_like(times, fixed_front_fraction)
    state = None
    rows = []
    for index, time in enumerate(times):
        lateral = lateral_g[index] * GRAVITY_MPS2
        request = BrakeControlRequest(
            braking_g[index] * GRAVITY_MPS2, front_fraction[index], 10,
            lateral, math.atan(parameters.vehicle.wheelbase_m * lateral / 10**2),
        )
        result = calculate_adaptive_braking(parameters, request, settings, state, time_step_s=float(times[1] - times[0]))
        state = result.state
        allocation = result.allocation
        row = dict(
            strength_fraction=settings.strength_fraction, time_s=float(time),
            lateral_acceleration_g=float(lateral_g[index]), braking_demand_g=float(braking_g[index]),
            front_fraction_requested=float(front_fraction[index]),
            front_fraction_delivered=allocation.front_braking_fraction if allocation else None,
            front_peak_asymmetry=result.peak_asymmetry_fractions[0],
            rear_peak_asymmetry=result.peak_asymmetry_fractions[1],
            front_applied_asymmetry=result.applied_asymmetry_fractions[0],
            rear_applied_asymmetry=result.applied_asymmetry_fractions[1],
            yaw_moment_nm=allocation.yaw_moment_nm if allocation else None,
            yaw_acceleration_radps2=result.yaw_acceleration_radps2,
            peak_combined_tire_utilization=result.peak_tire_utilization,
            meets_limits=result.meets_capacity_and_supply_limits,
            authority_overrode_slew=result.authority_limit_overrode_slew,
            limiting_reasons='; '.join(result.limiting_reasons),
        )
        for wheel_index, name in enumerate(('fl', 'fr', 'rl', 'rr')):
            row[f'{name}_braking_force_demand_n'] = allocation.braking_forces_n[wheel_index] if allocation else None
            row[f'{name}_lateral_force_reserve_n'] = result.lateral_force_reserves_n[wheel_index] if allocation else None
        previous_row = rows[-1] if rows else None
        previous_yaw = previous_row['yaw_moment_nm'] if previous_row else None
        row['yaw_moment_change_nm_per_s'] = (
            (row['yaw_moment_nm'] - previous_yaw) / float(times[1] - times[0])
            if previous_yaw is not None and row['yaw_moment_nm'] is not None else None
        )
        rows.append(row)
    return rows


def run_sensitivity_study(parameters, base_settings=RESEARCH_SETTINGS):
    """Illustrative sensitivity probes, not a distribution of real conditions."""
    rows = []
    for mass in (parameters.vehicle.vehicle_mass_kg, parameters.vehicle.vehicle_mass_kg * 1.2):
        for friction in (.9, 1.2, 1.5):
            vehicle = replace(parameters.vehicle, vehicle_mass_kg=mass, friction_coefficient=friction)
            changed = replace(parameters, vehicle=vehicle)
            for axles in ('front', 'rear', 'both'):
                for lateral_g in (-.6, 0, .6):
                    for braking_g in (.3, .6, .9):
                        for front_fraction in (.55, .65):
                            for strength in (0, .25, .5, 1):
                                settings = replace(base_settings, strength_fraction=strength, modulated_axles=axles, maximum_asymmetry_change_per_s=None, lateral_filter_time_constant_s=0)
                                request = BrakeControlRequest(braking_g * GRAVITY_MPS2, front_fraction, 10, lateral_g * GRAVITY_MPS2, math.atan(changed.vehicle.wheelbase_m * lateral_g * GRAVITY_MPS2 / 100))
                                result = calculate_adaptive_braking(changed, request, settings, time_step_s=.02)
                                rows.append(dict(
                                    mass_kg=mass, friction_coefficient=friction, modulated_axles=axles,
                                    lateral_g=lateral_g, braking_demand_g=braking_g, front_fraction=front_fraction,
                                    strength_fraction=strength, meets_limits=result.meets_capacity_and_supply_limits,
                                    yaw_moment_nm=result.allocation.yaw_moment_nm if result.allocation else None,
                                    peak_combined_tire_utilization=result.peak_tire_utilization,
                                ))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--output-dir', type=Path, default=Path('outputs/brake_control'))
    parser.add_argument('--front-bias', type=float, help='Fixed front tire-force percentage (0-100); default tests a varying driver preference.')
    parser.add_argument('--strength', type=float, help='Left/right assistance percentage (0-100), compared with assistance off.')
    parser.add_argument('--modulated-axles', choices=('front', 'rear', 'both'), default='both')
    parser.add_argument('--yaw-limit', type=float, default=RESEARCH_SETTINGS.maximum_yaw_moment_nm, help='Research yaw-moment ceiling in N m; not a validated safety limit.')
    args = parser.parse_args()
    for name in ('front_bias', 'strength'):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or not 0 <= value <= 100):
            parser.error(f'--{name.replace("_", "-")} must be between 0 and 100.')
    if not math.isfinite(args.yaw_limit) or args.yaw_limit < 0:
        parser.error('--yaw-limit must be finite and nonnegative.')
    import matplotlib
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    parameters = FourWheelParameters()
    times = np.linspace(0, 12, 601)
    strengths = (0, .25, .5, 1) if args.strength is None else tuple(sorted({0, args.strength / 100}))
    base_settings = replace(RESEARCH_SETTINGS, modulated_axles=args.modulated_axles, maximum_yaw_moment_nm=args.yaw_limit)
    fixed_front_fraction = None if args.front_bias is None else args.front_bias / 100
    studies = [run_maneuver(parameters, replace(base_settings, strength_fraction=strength), times, fixed_front_fraction) for strength in strengths]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    assumptions = dict(
        vehicle=asdict(parameters), research_controller_settings=asdict(base_settings),
        strength_fractions=strengths, time_step_s=.02,
        fixed_front_braking_fraction=fixed_front_fraction,
        maneuver='Prescribed 12 s brake application/release and left/right transitions; speed signal 10 m/s. No integrated trajectory.',
        peak_definition='Capacity-matched axle imbalance, not a lap-time or handling optimum.',
        sensitivity_mass_factors=[1, 1.2], sensitivity_friction_coefficients=[.9, 1.2, 1.5],
        valve_example='Tilton 98-1261 example: 650 psi knee, incremental slope 1/3. Not identification of the installed component.',
    )
    (args.output_dir / 'research_inputs.json').write_text(json.dumps(assumptions, indent=2), encoding='utf-8')
    write_csv(args.output_dir / 'maneuver_comparison.csv', [row for study in studies for row in study])
    sensitivity = run_sensitivity_study(parameters, base_settings)
    write_csv(args.output_dir / 'sensitivity_comparison.csv', sensitivity)

    figure, axes = plt.subplots(2, 2, figsize=(12, 8), layout='constrained')
    first = studies[0]
    axes[0, 0].plot(times, [row['braking_demand_g'] for row in first], label='Braking demand')
    axes[0, 0].plot(times, [row['lateral_acceleration_g'] for row in first], label='Lateral demand (+ left)')
    axes[0, 0].set(ylabel='Acceleration demand (g)', title='Prescribed inputs: application, release, opposite turns')
    for strength, study in zip(strengths, studies):
        label = f'{strength:.0%} assistance'
        axes[0, 1].plot(times, [row['front_applied_asymmetry'] for row in study], label=label)
        axes[1, 0].plot(times, [row['yaw_acceleration_radps2'] for row in study], label=label)
        axes[1, 1].plot(times, [row['peak_combined_tire_utilization'] for row in study], label=label)
    axes[0, 1].set(ylabel='Front (right-left) / axle braking force', title='Applied asymmetry after limits')
    axes[1, 0].set(ylabel='Snapshot yaw acceleration (rad/s²)', title='Residual moment / inertia; no driver correction')
    axes[1, 0].axhline(args.yaw_limit / parameters.yaw_moment_of_inertia_kgm2, color='black', linestyle=':', alpha=.5)
    axes[1, 0].axhline(-args.yaw_limit / parameters.yaw_moment_of_inertia_kgm2, color='black', linestyle=':', alpha=.5)
    axes[1, 1].axhline(1, color='black', linestyle=':', label='Tire limit')
    axes[1, 1].set(ylabel='Maximum combined force / friction capacity', title='Tire utilization; above 1 is infeasible')
    for axis in axes.flat:
        axis.set_xlabel('Time (s)')
        axis.grid(True, alpha=.3)
        axis.legend(fontsize=8)
    figure.suptitle('Active-control benchmark | explicit research limits | front/rear preference preserved')
    figure.savefig(args.output_dir / 'adaptive_brake_control.png', dpi=160)

    valve = ProportioningValve(650 * 6894.757293168, 1 / 3)
    inlet = np.linspace(0, 8e6, 321)
    output = np.array([valve.outlet_pressure_pa(float(value)) for value in inlet])
    valve_figure, valve_axis = plt.subplots(figsize=(7, 5), layout='constrained')
    valve_axis.plot(inlet / 1e5, inlet / 1e5, linestyle='--', label='Bypass / below-knee reference')
    valve_axis.plot(inlet / 1e5, output / 1e5, label='Fixed valve setting: application and release')
    valve_axis.axvline(valve.knee_pressure_pa / 1e5, linestyle=':', color='black', label='650 psi example knee')
    valve_axis.set(xlabel='Inlet pressure (bar)', ylabel='Outlet pressure (bar)', title='Passive valve: nonlinear pressure ratio, no corner sensing')
    valve_axis.grid(True, alpha=.3)
    valve_axis.legend(fontsize=8)
    valve_figure.savefig(args.output_dir / 'passive_valve_curve.png', dpi=160)
    write_csv(args.output_dir / 'passive_valve_curve.csv', [dict(inlet_pressure_pa=float(x), outlet_pressure_pa=float(y)) for x, y in zip(inlet, output)])

    print('Research input-signal study; no lap-time, path-following, or driver-confidence claim.')
    for strength, study in zip(strengths, studies):
        max_yaw = max(abs(row['yaw_moment_nm'] or 0) for row in study)
        peak_use = max(row['peak_combined_tire_utilization'] or 0 for row in study)
        failures = sum(not row['meets_limits'] for row in study)
        bias_errors = [abs(row['front_fraction_requested'] - row['front_fraction_delivered']) for row in study if row['front_fraction_delivered'] is not None]
        print(f'{strength:.0%} assistance: peak |yaw| {max_yaw:.2f} N m; peak tire use {peak_use:.3f}; {failures}/{len(study)} samples outside limits; maximum front/rear fraction error {max(bias_errors):.2e}')
    print(f'Saved {len(sensitivity)} sensitivity cases and plots to {args.output_dir.resolve()}')
    if not args.no_show:
        plt.show()
    plt.close(figure)
    plt.close(valve_figure)


if __name__ == '__main__':
    main()
