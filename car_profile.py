"""Car input profiles.

This public version contains only the original illustrative research vehicle.
Team-specific car data is kept out of the public repository.
"""

import math

from brake_hydraulics import HydraulicParameters
from vehicle_equilibrium import PerformanceParameters


CAR_PROFILE_NAMES = ('illustrative',)
DEFAULT_PROFILE = 'illustrative'


def _check_profile(profile):
    if profile not in CAR_PROFILE_NAMES:
        raise ValueError(f'Unknown car profile: {profile}.')


def make_vehicle(profile=DEFAULT_PROFILE):
    """Return a complete runnable vehicle profile."""
    _check_profile(profile)
    return PerformanceParameters()


def make_hydraulics(profile=DEFAULT_PROFILE):
    """Two equal 19.05 mm master bores and research per-wheel force gains."""
    _check_profile(profile)
    area = math.pi * .01905**2 / 4
    return HydraulicParameters(area, area, (.0002,) * 4)


def vehicle_profile_metadata(profile=DEFAULT_PROFILE):
    """JSON-ready provenance for the selected profile."""
    _check_profile(profile)
    return dict(id=profile, label='Original illustrative vehicle', source=None,
                measured_values={}, assumptions=['Original research placeholders retained for comparison.'])
