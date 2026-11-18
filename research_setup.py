"""Explicit illustrative inputs for runnable comparisons, not installed parts.

Replace these with measured component data before interpreting actual-car
performance. The program exports every value with each comparison.
"""

from dataclasses import replace
import math

from adaptive_brake_control import AdaptiveBrakeSettings
from brake_hydraulics import HydraulicConfiguration, HydraulicParameters, ProportioningValve, solve_master_force_fraction
from vehicle_equilibrium import BrakeStrategy, PerformanceParameters
from fiddle_brake import FiddleBrakeSettings


def select_research_strategies(strategies, configurations):
    """Keep the legacy passive study separate from the auxiliary-brake test."""
    if configurations == 'all':
        return strategies
    if configurations == 'baseline':
        return strategies[:1]
    if configurations == 'active':
        return [strategies[0], strategies[-1]]
    if configurations == 'fiddle':
        settings = strategies[-1].assistance
        fiddle = FiddleBrakeSettings(strength_fraction=settings.strength_fraction,
                                     maximum_yaw_moment_nm=settings.maximum_yaw_moment_nm)
        return [strategies[0], replace(strategies[0], name=f'Inside-rear fiddle brake {fiddle.strength_fraction:.0%}',
                                      kind='fiddle', fiddle=fiddle)]
    raise ValueError('Unknown brake comparison.')


def make_research_strategies(front_fraction=.6, assistance_strength=.5, yaw_limit_nm=120.0, knee_pressure_pa=2e6, *, profile='illustrative'):
    # Research-only 3/4-inch master bores and calibrated force-gain placeholders.
    from car_profile import make_hydraulics
    hydraulics = make_hydraulics(profile)
    bar_setting = solve_master_force_fraction(hydraulics, 2000.0, front_fraction)
    common = dict(front_braking_fraction=front_fraction, hydraulics=hydraulics, front_master_force_fraction=bar_setting, maximum_pushrod_force_n=4000.0)
    valve = ProportioningValve(knee_pressure_pa, 1 / 3)
    baseline = BrakeStrategy('Conventional front/rear', **common)
    strategies = [baseline]
    for label, valves in (
        ('Rear-left valve', (None, None, valve, None)),
        ('Rear-right valve', (None, None, None, valve)),
        ('Fixed-left valves', (valve, None, valve, None)),
        ('Fixed-right valves', (None, valve, None, valve)),
    ):
        strategies.append(replace(baseline, name=label, kind='passive', configuration=HydraulicConfiguration(valves=valves)))
    settings = AdaptiveBrakeSettings(strength_fraction=assistance_strength, maximum_yaw_moment_nm=yaw_limit_nm, maximum_asymmetry_fraction=.4)
    strategies.append(replace(baseline, name=f'Independent {assistance_strength:.0%} assistance', kind='adaptive', assistance=settings))
    return strategies


def make_research_vehicle(profile='illustrative'):
    from car_profile import make_vehicle
    return make_vehicle(profile)
