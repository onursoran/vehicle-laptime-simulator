import math

def calculate_lateral_tire_force (slip_angle_deg, vertical_load_n, friction_coefficient, cornering_stiffness_n_per_rad):
    slip_angle_rad = math.radians(slip_angle_deg)
    maximum_lateral_force_n = vertical_load_n * friction_coefficient
    unsaturated_lateral_force_n = slip_angle_rad * cornering_stiffness_n_per_rad
    actual_lateral_force_n = max( -maximum_lateral_force_n,min(unsaturated_lateral_force_n, maximum_lateral_force_n))
    return actual_lateral_force_n

def calculate_longitudinal_tire_force (slip_ratio, vertical_load_n, friction_coefficient, longitudinal_stiffness_n):
    maximum_longitudinal_tire_force = vertical_load_n * friction_coefficient
    unsaturated_longitudinal_force_n = slip_ratio * longitudinal_stiffness_n
    actual_longitudinal_force_n = max( -maximum_longitudinal_tire_force, min(unsaturated_longitudinal_force_n, maximum_longitudinal_tire_force))
    return actual_longitudinal_force_n

def apply_friction_circle(req_longitudinal_force_n, req_lateral_force_n, maximum_tire_force_n):
    combined_force_magnitude_n = math.hypot(req_lateral_force_n, req_longitudinal_force_n)
    if combined_force_magnitude_n <= maximum_tire_force_n:
        return req_longitudinal_force_n, req_lateral_force_n

    scale_factor = maximum_tire_force_n / combined_force_magnitude_n

    actual_longitudinal_force_n = (req_longitudinal_force_n * scale_factor)
    actual_lateral_force_n = (req_lateral_force_n * scale_factor)

    return actual_longitudinal_force_n, actual_lateral_force_n


def calculate_combined_tire_forces(slip_angle_deg, slip_ratio, vertical_load_n, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness):
    slip_angle_rad = math.radians(slip_angle_deg)
    unsaturated_lateral_force_n = slip_angle_rad * cornering_stiffness_n_per_rad
    unsaturated_longitudal_force_n = slip_ratio * longitudinal_stiffness
    maximum_tire_force_n = vertical_load_n * friction_coefficient
    return apply_friction_circle(unsaturated_longitudal_force_n, unsaturated_lateral_force_n, maximum_tire_force_n)