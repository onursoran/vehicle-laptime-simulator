"""Offline, full-size historical circuit centerlines with periodic geometry.

The bundled source coordinates are unchanged. A C2 periodic cubic spline
through those already-smoothed coordinates supplies continuous curvature.
There is no length scaling, invented braking map, or racing-line optimization.
"""

from functools import lru_cache
import hashlib
import math
from pathlib import Path

import numpy as np

from track_model import Track, make_test_track


DATA_DIRECTORY = Path(__file__).resolve().parent / 'data' / 'tracks' / 'tumftm'
SOURCE_REVISION = 'e59595d1f3573b30d1ded6a08984935b957688e0'
REAL_TRACKS = {
    'monza': ('Monza', 'Monza (2020 dataset)'),
    'montreal': ('Montreal', 'Montreal - Circuit Gilles-Villeneuve (2020 dataset)'),
    'catalunya': ('Catalunya', 'Barcelona-Catalunya (2020 layout with final chicane)'),
}
SYNTHETIC_TRACKS = ('oval', 'figure-eight', 'technical', 'circle')
TRACK_NAMES = (*SYNTHETIC_TRACKS, *REAL_TRACKS)


def _tridiagonal_solve(lower, diagonal, upper, rhs):
    """Thomas elimination for the strictly diagonally dominant spline system."""
    diagonal, rhs = diagonal.copy(), rhs.copy()
    for index in range(1, len(diagonal)):
        ratio = lower[index - 1] / diagonal[index - 1]
        diagonal[index] -= ratio * upper[index - 1]
        rhs[index] -= ratio * rhs[index - 1]
    answer = rhs / diagonal[:, None]
    for index in range(len(diagonal) - 2, -1, -1):
        answer[index] -= upper[index] * answer[index + 1] / diagonal[index]
    return answer


class PeriodicCenterline:
    """Arc-parameterized interpolation with a continuous closing seam."""

    def __init__(self, coordinates):
        coordinates = np.asarray(coordinates, dtype=float)
        if coordinates.ndim != 2 or coordinates.shape[1] != 2 or len(coordinates) < 8 or not np.all(np.isfinite(coordinates)):
            raise ValueError('Centerline needs at least eight finite x/y samples.')
        if np.linalg.norm(coordinates[-1] - coordinates[0]) < 1e-6:
            coordinates = coordinates[:-1]
        h = np.linalg.norm(np.roll(coordinates, -1, axis=0) - coordinates, axis=1)
        if np.any(h < 1e-6):
            raise ValueError('Centerline contains repeated consecutive points.')
        self.coordinates = coordinates
        self.h = h
        self.knots = np.r_[0.0, np.cumsum(h)]
        slopes = (np.roll(coordinates, -1, axis=0) - coordinates) / h[:, None]
        rhs = 6 * (slopes - np.roll(slopes, 1, axis=0))
        diagonal = 2 * (h + np.roll(h, 1))
        # Two corner terms turn a tridiagonal matrix into a cyclic matrix.
        # Solve with a rank-two Woodbury correction: O(n), no SciPy dependency.
        updates = np.zeros((len(h), 2))
        updates[0, 0] = h[-1]
        updates[-1, 1] = h[-1]
        solved = _tridiagonal_solve(h[:-1], diagonal, h[:-1], np.c_[rhs, updates])
        base, correction = solved[:, :2], solved[:, 2:]
        adjustment = np.linalg.solve(np.eye(2) + correction[[-1, 0]], base[[-1, 0]])
        self.second_derivatives = base - correction @ adjustment
        # Numerically integrate spline arc length on <=0.5 m parameter steps,
        # independent of requested solver spacing, so refinement keeps geometry.
        fine = np.linspace(0, self.knots[-1], math.ceil(self.knots[-1] / .5) + 1)
        _, first, _ = self.evaluate(fine)
        speed = np.linalg.norm(first, axis=1)
        arc = np.r_[0.0, np.cumsum((speed[:-1] + speed[1:]) * .5 * np.diff(fine))]
        self.length_m = float(arc[-1])
        self.arc_parameters = fine
        self.arc_distances = arc

    def evaluate(self, parameter):
        parameter = np.asarray(parameter) % self.knots[-1]
        index = np.searchsorted(self.knots, parameter, side='right') - 1
        following = (index + 1) % len(self.h)
        h = self.h[index, None]
        fraction = ((parameter - self.knots[index]) / self.h[index])[:, None]
        a, b = 1 - fraction, fraction
        p, q = self.coordinates[index], self.coordinates[following]
        m, n = self.second_derivatives[index], self.second_derivatives[following]
        position = a * p + b * q + ((a**3 - a) * m + (b**3 - b) * n) * h**2 / 6
        first = (q - p) / h + ((1 - 3 * a**2) * m + (3 * b**2 - 1) * n) * h / 6
        second = a * m + b * n
        return position, first, second

    def sample(self, spacing_m):
        count = max(16, math.ceil(self.length_m / spacing_m))
        distance = np.linspace(0, self.length_m, count, endpoint=False)
        parameter = np.interp(distance, self.arc_distances, self.arc_parameters)
        position, first, second = self.evaluate(parameter)
        curvature = (first[:, 0] * second[:, 1] - first[:, 1] * second[:, 0]) / np.linalg.norm(first, axis=1)**3
        return distance, position, np.unwrap(np.arctan2(first[:, 1], first[:, 0])), curvature


@lru_cache(maxsize=3)
def _source_geometry(name):
    filename = REAL_TRACKS[name][0] + '.csv'
    path = DATA_DIRECTORY / filename
    raw = np.loadtxt(path, delimiter=',', comments='#')
    if raw.ndim != 2 or raw.shape[1] != 4 or not np.all(np.isfinite(raw)) or np.any(raw[:, 2:] <= 0):
        raise ValueError(f'Invalid circuit data in {filename}.')
    return PeriodicCenterline(raw[:, :2]), hashlib.sha256(path.read_bytes()).hexdigest()


def make_track(name='oval', spacing_m=None, width_m=None):
    """Select a synthetic test or a bundled real circuit, without networking.

    Real default: 4 m samples, uniform 12 m experimental corridor. Synthetic
    default: 2 m samples, 3 m corridor. Width is an explicit user override,
    not a claim of measured track boundaries. Existing car parameters stay put.
    """
    if name not in TRACK_NAMES:
        raise ValueError(f'Unknown track: {name}.')
    real = name in REAL_TRACKS
    spacing_m = (4.0 if real else 2.0) if spacing_m is None else spacing_m
    width_m = (12.0 if real else 3.0) if width_m is None else width_m
    if not all(math.isfinite(value) and value > 0 for value in (spacing_m, width_m)):
        raise ValueError('Track spacing and width must be positive and finite.')
    if not real:
        return make_test_track(name, spacing_m, width_m)
    spline, checksum = _source_geometry(name)
    distance, xy, heading, curvature = spline.sample(spacing_m)
    filename, label = REAL_TRACKS[name]
    metadata = dict(
        kind='real', track_id=name, source='TUMFTM racetrack-database; OpenStreetMap contributors',
        source_url=f'https://github.com/TUMFTM/racetrack-database/blob/{SOURCE_REVISION}/tracks/{filename}.csv',
        source_revision=SOURCE_REVISION, source_sha256=checksum,
        layout_note='Historical 2020 dataset; approximate smoothed centerline, flat surface, no banking or elevation. Not a surveyed current circuit.',
        width_note='Uniform user-selected corridor; source satellite-estimated left/right widths are not used.',
        geometry_method='Periodic C2 cubic interpolation, numerical arc-length resampling; original metric scale and source travel direction.',
        mirrored=False, reversed=False,
    )
    return Track(label, distance, xy[:, 0], xy[:, 1], heading, curvature, np.full(len(distance), width_m), spline.length_m, metadata)
