"""Smooth, closed test tracks sampled at approximately uniform arc length."""

from dataclasses import dataclass, field
import math

import numpy as np


@dataclass(frozen=True)
class Track:
    name: str
    distance_m: np.ndarray
    x_m: np.ndarray
    y_m: np.ndarray
    heading_rad: np.ndarray
    curvature_per_m: np.ndarray
    width_m: np.ndarray
    length_m: float
    metadata: dict = field(default_factory=dict)

    def __post_init__(self):
        arrays = (self.distance_m, self.x_m, self.y_m, self.heading_rad, self.curvature_per_m, self.width_m)
        count = len(self.distance_m)
        if count < 8 or any(np.asarray(values).shape != (count,) or not np.all(np.isfinite(values)) for values in arrays):
            raise ValueError('Track requires at least eight matching finite one-dimensional arrays.')
        if not math.isfinite(self.length_m) or self.length_m <= 0 or self.distance_m[0] != 0 or self.distance_m[-1] >= self.length_m:
            raise ValueError('Track distances must start at zero and stop before the positive closing length.')
        if np.any(np.diff(self.distance_m) <= 0) or np.any(self.width_m <= 0):
            raise ValueError('Track distances must increase and widths must be positive.')

    @property
    def segment_lengths_m(self):
        return np.diff(np.r_[self.distance_m, self.length_m])

    @property
    def curvature_gradient_per_m2(self):
        previous = np.roll(self.segment_lengths_m, 1)
        return (np.roll(self.curvature_per_m, -1) - np.roll(self.curvature_per_m, 1)) / (previous + self.segment_lengths_m)

    def mirrored(self):
        return Track(self.name + ' mirrored', self.distance_m.copy(), self.x_m.copy(), -self.y_m, -self.heading_rad, -self.curvature_per_m, self.width_m.copy(), self.length_m, {**self.metadata, 'mirrored': True})

    def reversed(self):
        indices = np.r_[0, np.arange(len(self.distance_m) - 1, 0, -1)]
        distances = np.r_[0.0, self.length_m - self.distance_m[indices[1:]]]
        return Track(self.name + ' reversed', distances, self.x_m[indices], self.y_m[indices], np.unwrap(self.heading_rad[indices] + math.pi), -self.curvature_per_m[indices], self.width_m[indices], self.length_m, {**self.metadata, 'reversed': True})


def make_test_track(name='oval', spacing_m=4.0, width_m=3.0, *, radius_m=20.0):
    """Synthetic geometry, not a reconstruction of a competition circuit.

    Figure-eight is a controlled crossing test; intersection traffic and
    grade separation are not modeled. Width bounds the fixed centerline
    corridor and does not secretly optimize the racing line.
    """
    if not all(math.isfinite(value) and value > 0 for value in (spacing_m, width_m, radius_m)):
        raise ValueError('Spacing, width and radius must be positive.')
    if name not in ('oval', 'figure-eight', 'technical', 'circle'):
        raise ValueError('Choose oval, figure-eight, technical, or circle.')

    def geometry(theta):
        if name in ('oval', 'circle'):
            a, b = (45.0, 22.0) if name == 'oval' else (radius_m, radius_m)
            x, y = a * np.sin(theta), b * (1 - np.cos(theta))
            dx, dy = a * np.cos(theta), b * np.sin(theta)
            ddx, ddy = -a * np.sin(theta), b * np.cos(theta)
        elif name == 'figure-eight':
            x, y = 35 * np.sin(theta), 15 * np.sin(2 * theta)
            dx, dy = 35 * np.cos(theta), 30 * np.cos(2 * theta)
            ddx, ddy = -35 * np.sin(theta), -60 * np.sin(2 * theta)
        else:
            radius = 30 * (1 + .25 * np.cos(3 * theta))
            derivative = -22.5 * np.sin(3 * theta)
            second_derivative = -67.5 * np.cos(3 * theta)
            x, y = radius * np.sin(theta), 37.5 - radius * np.cos(theta)
            dx, dy = derivative * np.sin(theta) + radius * np.cos(theta), -derivative * np.cos(theta) + radius * np.sin(theta)
            ddx = second_derivative * np.sin(theta) + 2 * derivative * np.cos(theta) - radius * np.sin(theta)
            ddy = -second_derivative * np.cos(theta) + 2 * derivative * np.sin(theta) + radius * np.cos(theta)
        return x, y, dx, dy, ddx, ddy

    fine_theta = np.linspace(0, 2 * math.pi, 20001)
    _, _, dx, dy, _, _ = geometry(fine_theta)
    speed = np.hypot(dx, dy)
    fine_s = np.r_[0.0, np.cumsum((speed[:-1] + speed[1:]) / 2 * np.diff(fine_theta))]
    length = float(fine_s[-1])
    count = max(16, math.ceil(length / spacing_m))
    distance = np.linspace(0, length, count, endpoint=False)
    theta = np.interp(distance, fine_s, fine_theta)
    x, y, dx, dy, ddx, ddy = geometry(theta)
    curvature = (dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    return Track(name, distance, x, y, np.unwrap(np.arctan2(dy, dx)), curvature, np.full(count, width_m), length)
