"""Closed-lap forward/backward speed profiles with audited equilibrium.

The offline planner knows track geometry. That geometry is never passed to
the independent-brake controller. This is a fixed-line quasi-steady estimate,
not a transient race driver or a proof of minimum achievable real lap time.
"""

from dataclasses import dataclass
import math

import numpy as np

from vehicle_equilibrium import solve_operating_point


@dataclass(frozen=True)
class LapResult:
    track_name: str
    strategy_name: str
    lap_time_s: float
    speed_mps: np.ndarray
    segment_time_s: np.ndarray
    tangential_acceleration_mps2: np.ndarray
    segment_midpoints: tuple
    maximum_equilibrium_error: float
    maximum_tire_utilization: float
    maximum_abs_steering_rad: float
    iterations: int


class LapTimeSolver:
    def __init__(self, parameters, strategy, *, speed_resolution_mps=.015, maximum_passes=25, cancel_check=None):
        self.parameters = parameters
        self.strategy = strategy
        self.speed_resolution_mps = speed_resolution_mps
        self.maximum_passes = maximum_passes
        self._points = {}
        self._geometry_track = None
        self.cancel_check = cancel_check
        self._segment_probes = 0
        if cancel_check is not None and not callable(cancel_check):
            raise ValueError('cancel_check must be callable when supplied.')
        if speed_resolution_mps <= 0 or not math.isfinite(speed_resolution_mps) or maximum_passes < 1:
            raise ValueError('Positive finite speed resolution and pass count are required.')

    def point(self, speed, acceleration, curvature, gradient, width):
        # Exact-key memoization; no quantization of physical constraints.
        key = (float(speed), float(acceleration), float(curvature), float(gradient), float(width))
        if key not in self._points:
            self._points[key] = solve_operating_point(self.parameters, self.strategy, *key)
        return self._points[key]

    def _speed_limit(self, curvature, gradient, width):
        if self.cancel_check is not None:
            self.cancel_check()
        low = .5
        if not self.point(low, 0, curvature, gradient, width).is_feasible:
            raise ValueError('Track cannot be followed at low speed within the configured steering, footprint, and contact constraints.')
        high = self.parameters.maximum_speed_mps
        if self.point(high, 0, curvature, gradient, width).is_feasible:
            return high
        while high - low > self.speed_resolution_mps:
            candidate = (low + high) / 2
            if self.point(candidate, 0, curvature, gradient, width).is_feasible:
                low = candidate
            else:
                high = candidate
        # A small numerical margin leaves room for acceleration-induced load
        # transfer and avoids treating the nonlinear boundary as exact data.
        return low * .999

    def _prepare_track(self, track):
        """Build invariant geometry once, without rounding or changing samples.

        Previously every edge feasibility probe rebuilt full-lap distance and
        curvature-gradient arrays. That made geometry work quadratic in the
        number of samples. Each solve refreshes these snapshots so reusing a
        solver after editing a track still observes the new geometry.
        """
        self._segment_lengths = track.segment_lengths_m
        self._curvature_gradients = track.curvature_gradient_per_m2
        curvature, gradients, width = track.curvature_per_m, self._curvature_gradients, track.width_m
        following_curvature, following_gradients, following_width = (
            np.roll(values, -1) for values in (curvature, gradients, width)
        )
        start = np.column_stack((curvature, gradients, width))
        midpoint = np.column_stack(((curvature + following_curvature) / 2,
                                    (gradients + following_gradients) / 2,
                                    np.minimum(width, following_width)))
        end = np.column_stack((following_curvature, following_gradients, following_width))
        self._segment_geometry = np.stack((start, midpoint, end), axis=1)
        self._geometry_track = track

    def _segment_points(self, track, index, speed_start, speed_end):
        self._segment_probes += 1
        if self.cancel_check is not None and self._segment_probes % 32 == 0:
            self.cancel_check()
        if self._geometry_track is not track:
            self._prepare_track(track)
        ds = self._segment_lengths[index]
        acceleration = (speed_end**2 - speed_start**2) / (2 * ds)
        midpoint_speed = math.sqrt((speed_start**2 + speed_end**2) / 2)
        points = []
        for speed, (curvature, gradient, width) in zip((speed_start, midpoint_speed, speed_end), self._segment_geometry[index]):
            point = self.point(speed, acceleration, curvature, gradient, width)
            points.append(point)
            if not point.is_feasible:
                return False, tuple(points)
        return True, tuple(points)

    def solve(self, track):
        self._prepare_track(track)
        count = len(track.distance_m)
        gradients = self._curvature_gradients
        speeds = np.array([self._speed_limit(curvature, gradient, width) for curvature, gradient, width in zip(track.curvature_per_m, gradients, track.width_m)])
        for iteration in range(1, self.maximum_passes + 1):
            previous = speeds.copy()
            for forward in (False, True):
                indices = range(count) if forward else range(count - 1, -1, -1)
                for index in indices:
                    next_index = (index + 1) % count
                    if forward and speeds[next_index] < speeds[index]:
                        continue
                    if not forward and speeds[index] < speeds[next_index]:
                        continue
                    feasible, _ = self._segment_points(track, index, speeds[index], speeds[next_index])
                    if feasible:
                        continue
                    variable_index = next_index if forward else index
                    fixed_index = index if forward else next_index
                    high = speeds[variable_index]
                    low = min(high, speeds[fixed_index])
                    start = speeds[index] if forward else low
                    end = low if forward else speeds[next_index]
                    low_feasible, _ = self._segment_points(track, index, start, end)
                    if not low_feasible:
                        # Changes in curvature/yaw demand can invalidate even
                        # equal endpoint speeds. Reduce both and propagate on
                        # the next pass; never accept an unsupported segment.
                        speeds[index] *= .97
                        speeds[next_index] *= .97
                        continue
                    while high - low > self.speed_resolution_mps:
                        candidate = (low + high) / 2
                        start = speeds[index] if forward else candidate
                        end = candidate if forward else speeds[next_index]
                        candidate_feasible, _ = self._segment_points(track, index, start, end)
                        if candidate_feasible:
                            low = candidate
                        else:
                            high = candidate
                    speeds[variable_index] = low
            if np.any(speeds < .2):
                raise RuntimeError('No usable closed-lap profile found; inspect track and vehicle limits.')
            if np.max(previous - speeds) <= self.speed_resolution_mps:
                audit = [self._segment_points(track, index, speeds[index], speeds[(index + 1) % count]) for index in range(count)]
                if all(feasible for feasible, _ in audit):
                    break
        else:
            raise RuntimeError('Speed profile did not converge to an audited feasible closed lap.')

        # Fresh full-lap evidence at start/midpoint/end of every segment.
        all_points = [point for _, points in audit for point in points]
        midpoint_points = tuple(points[1] for _, points in audit)
        next_speeds = np.roll(speeds, -1)
        times = 2 * self._segment_lengths / (speeds + next_speeds)
        accelerations = (next_speeds**2 - speeds**2) / (2 * self._segment_lengths)
        return LapResult(
            track.name, self.strategy.name, float(sum(times)), speeds, times,
            accelerations, midpoint_points,
            max(point.equilibrium_error for point in all_points),
            max(point.peak_tire_utilization for point in all_points),
            max(abs(point.steering_angle_rad) for point in all_points), iteration,
        )
