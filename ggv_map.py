import numpy as np
from optimizer import find_longitudinal_acceleration_limits_at_target_lateral_acceleration


def generate_gg_diagram_at_speed(vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, maximum_lateral_acceleration_mps2, number_of_lateral_acceleration_targets, maximum_steering_angle_deg, number_of_steering_test_points, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points, maximum_absolute_slip_ratio, number_of_slip_ratio_test_points, lateral_acceleration_tolerance_mps2, yaw_acceleration_tolerance_radps2, center_of_gravity_height_m=0.0):
    positive_lateral_acceleration_targets_mps2 = np.linspace(0, maximum_lateral_acceleration_mps2, number_of_lateral_acceleration_targets)

    positive_lateral_accelerations_mps2 = []
    maximum_longitudinal_accelerations_mps2 = []
    minimum_longitudinal_accelerations_mps2 = []

    for target_lateral_acceleration_mps2 in positive_lateral_acceleration_targets_mps2:
        maximum_longitudinal_acceleration_mps2, minimum_longitudinal_acceleration_mps2 = find_longitudinal_acceleration_limits_at_target_lateral_acceleration(vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, target_lateral_acceleration_mps2, maximum_steering_angle_deg, number_of_steering_test_points, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points, maximum_absolute_slip_ratio, number_of_slip_ratio_test_points, lateral_acceleration_tolerance_mps2, yaw_acceleration_tolerance_radps2, center_of_gravity_height_m)

        if maximum_longitudinal_acceleration_mps2 is not None and minimum_longitudinal_acceleration_mps2 is not None:
            positive_lateral_accelerations_mps2.append(target_lateral_acceleration_mps2)
            maximum_longitudinal_accelerations_mps2.append(maximum_longitudinal_acceleration_mps2)
            minimum_longitudinal_accelerations_mps2.append(minimum_longitudinal_acceleration_mps2)

    # Exclude zero from mirroring only when it was actually feasible.
    mirror_start_index = 1 if positive_lateral_accelerations_mps2 and positive_lateral_accelerations_mps2[0] == 0.0 else 0
    negative_lateral_accelerations_mps2 = [-value for value in reversed(positive_lateral_accelerations_mps2[mirror_start_index:])]
    negative_side_maximum_longitudinal_accelerations_mps2 = list(reversed(maximum_longitudinal_accelerations_mps2[mirror_start_index:]))
    negative_side_minimum_longitudinal_accelerations_mps2 = list(reversed(minimum_longitudinal_accelerations_mps2[mirror_start_index:]))

    lateral_accelerations_mps2 = np.array(negative_lateral_accelerations_mps2 + positive_lateral_accelerations_mps2)
    complete_maximum_longitudinal_accelerations_mps2 = np.array(negative_side_maximum_longitudinal_accelerations_mps2 + maximum_longitudinal_accelerations_mps2)
    complete_minimum_longitudinal_accelerations_mps2 = np.array(negative_side_minimum_longitudinal_accelerations_mps2 + minimum_longitudinal_accelerations_mps2)

    return lateral_accelerations_mps2, complete_maximum_longitudinal_accelerations_mps2, complete_minimum_longitudinal_accelerations_mps2


def generate_ggv_map(vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, maximum_lateral_acceleration_mps2, number_of_lateral_acceleration_targets, maximum_steering_angle_deg, number_of_steering_test_points, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points, maximum_absolute_slip_ratio, number_of_slip_ratio_test_points, lateral_acceleration_tolerance_mps2, yaw_acceleration_tolerance_radps2, minimum_forward_velocity, maximum_forward_velocity, number_of_velocity_test_points):
    candidate_forward_velocities
