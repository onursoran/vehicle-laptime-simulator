"""Describe simulated braking intensity without prescribing brake commands.

These labels describe the solved car profile, not real-world telemetry or
fixed track properties. Braking requires both a positive hydraulic command
and meaningful path deceleration; tire scrub/coasting alone is not braking.
"""

import math


GRAVITY_MPS2 = 9.81
CATEGORIES = ('none', 'light', 'medium', 'hard')
MINIMUM_PUSHROD_FORCE_N = 1.0
MINIMUM_DECELERATION_G = .05
MEDIUM_DECELERATION_G = .3
HARD_DECELERATION_G = .7


def analyze_braking(track, result):
    """Return JSON-ready per-segment labels, totals and contiguous brake zones.

    Zones merge across the closing seam. A wrapped zone ends after distance
    zero on the next traversal; a nonwrapped zone can end at ``length_m``.
    A profile braking over every segment is represented as one full-lap,
    wrapped zone with both start and end at zero.
    """
    count = len(track.distance_m)
    distances = [float(value) for value in track.distance_m]
    lengths = [float(value) for value in track.segment_lengths_m]
    times = [float(value) for value in result.segment_time_s]
    speeds = [float(value) for value in result.speed_mps]
    accelerations = [float(value) for value in result.tangential_acceleration_mps2]
    commands = [float(point.pushrod_force_n) for point in result.segment_midpoints]
    arrays = (distances, lengths, times, speeds, accelerations, commands)
    if not count or any(len(values) != count or not all(math.isfinite(value) for value in values) for values in arrays):
        raise ValueError('Braking analysis requires matching finite track and profile arrays.')
    if any(value <= 0 for values in (lengths, times, speeds) for value in values):
        raise ValueError('Segment lengths, times and speeds must be positive.')

    deceleration_g = [max(0.0, -value / GRAVITY_MPS2) for value in accelerations]
    ranks = []
    for deceleration, command in zip(deceleration_g, commands):
        if command <= MINIMUM_PUSHROD_FORCE_N or deceleration <= MINIMUM_DECELERATION_G:
            ranks.append(0)
        elif deceleration < MEDIUM_DECELERATION_G:
            ranks.append(1)
        elif deceleration < HARD_DECELERATION_G:
            ranks.append(2)
        else:
            ranks.append(3)
    categories = [CATEGORIES[rank] for rank in ranks]
    distance_totals = {category: 0.0 for category in CATEGORIES}
    time_totals = {category: 0.0 for category in CATEGORIES}
    for category, length, duration in zip(categories, lengths, times):
        distance_totals[category] += length
        time_totals[category] += duration

    braking = [rank > 0 for rank in ranks]
    all_braking = all(braking)
    starts = [0] if all_braking else [index for index in range(count) if braking[index] and not braking[index - 1]]
    zones = []
    for start in starts:
        indices = []
        index = start
        while braking[index] and len(indices) < count:
            indices.append(index)
            index = (index + 1) % count
        wrapped = all_braking or any(right < left for left, right in zip(indices, indices[1:]))
        end_distance = 0.0 if all_braking else float(track.length_m) if index == 0 else distances[index]
        zones.append(dict(
            start_distance_m=distances[start], end_distance_m=end_distance,
            wraps_start_finish=wrapped,
            length_m=sum(lengths[index] for index in indices),
            duration_s=sum(times[index] for index in indices),
            entry_speed_mps=speeds[start], exit_speed_mps=speeds[(indices[-1] + 1) % count],
            peak_deceleration_g=max(deceleration_g[index] for index in indices),
            category=CATEGORIES[max(ranks[index] for index in indices)],
        ))

    return dict(
        segment_category=categories,
        peak_deceleration_g=max((deceleration for deceleration, active in zip(deceleration_g, braking) if active), default=0.0),
        distance_m=distance_totals, time_s=time_totals, zones=zones,
        thresholds=dict(
            gravity_mps2=GRAVITY_MPS2,
            minimum_pushrod_force_n_exclusive=MINIMUM_PUSHROD_FORCE_N,
            minimum_deceleration_g_exclusive=MINIMUM_DECELERATION_G,
            medium_deceleration_g_inclusive=MEDIUM_DECELERATION_G,
            hard_deceleration_g_inclusive=HARD_DECELERATION_G,
        ),
        interpretation='Intensity of the simulated car profile; not measured circuit braking zones. Zone category is its peak intensity.',
    )
