import numpy as np
from vehicle_model import calculate_bicycle_model_accelerations

def find_maximum_straight_line_acceleration (vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_intertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, maximum_slip_ratio, number_of_test_points, center_of_gravity_height_m=0.0):
    candidate_slip_ratios = np.linspace(0, maximum_slip_ratio, number_of_test_points + 1)

    best_slip_ratio = 0.0
    maximum_longitudinal_acceleration_mps2 = 0.0

    for candidate_slip_ratio in candidate_slip_ratios:
        longitudinal_acceleration_mps2, lateral_acceleration_mps2, yaw_accelerationradps2 = calculate_bicycle_model_accelerations(vehicle_mass_kg, 0, forward_velocity_mps, 0, 0, candidate_slip_ratio, candidate_slip_ratio, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_intertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, center_of_gravity_height_m)

        if longitudinal_acceleration_mps2 > maximum_longitudinal_acceleration_mps2:
            maximum_longitudinal_acceleration_mps2 = longitudinal_acceleration_mps2
            best_slip_ratio = candidate_slip_ratio

    return best_slip_ratio, maximum_longitudinal_acceleration_mps2

def find_maximum_straight_line_braking (vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, maximum_braking_slip_ratio, number_of_test_points, center_of_gravity_height_m=0.0):
    candidate_slip_ratios = np.linspace(0, -maximum_braking_slip_ratio, number_of_test_points)
    best_braking_slip_ratio = 0.0
    minimum_longitudinal_acceleration_mps2 = 0.0

    for candidate_slip_ratio in candidate_slip_ratios:
        longitudinal_acceleration_mps2, lateral_acceleration_mps2, yaw_acceleration_radps2 = calculate_bicycle_model_accelerations(vehicle_mass_kg, 0, forward_velocity_mps, 0, 0, candidate_slip_ratio, candidate_slip_ratio, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, center_of_gravity_height_m)

        if longitudinal_acceleration_mps2 < minimum_longitudinal_acceleration_mps2:
            minimum_longitudinal_acceleration_mps2 = longitudinal_acceleration_mps2
            best_braking_slip_ratio = candidate_slip_ratio
    return best_braking_slip_ratio, minimum_longitudinal_acceleration_mps2

def calculate_steady_state_cornering_errors (forward_velocity_mps, yaw_rate_radps, lateral_acceleration_mps2, yaw_acceleration_radps2):
    required_lateral_acceleration_mps2 = forward_velocity_mps * yaw_rate_radps

    lateral_acceleration_error_mps2 = lateral_acceleration_mps2 - required_lateral_acceleration_mps2
    yaw_acceleration_error_radps2 = yaw_acceleration_radps2

    return lateral_acceleration_error_mps2, yaw_acceleration_error_radps2

def find_maximum_steady_state_cornering_acceleration(vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, maximum_steering_angle_deg, number_of_steering_test_points, maximum_yaw_rate_radps, number_of_yaw_rate_test_points, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points, lateral_acceleration_tolerance_mps2, yaw_acceleration_tolerance_radps2, center_of_gravity_height_m=0.0):
    candidate_steering_angles_deg = np.linspace(0, maximum_steering_angle_deg, number_of_steering_test_points)
    candidate_yaw_rates_radps = np.linspace(0, maximum_yaw_rate_radps, number_of_yaw_rate_test_points)
    candidate_lateral_velocities_mps = np.linspace(-maximum_lateral_velocity_mps, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points)

    best_steering_angle_deg = 0.0
    best_lateral_velocity_mps = 0.0
    best_yaw_rate_radps = 0.0
    maximum_lateral_acceleration_mps2 = 0.0

    for candidate_steering_angle_deg in candidate_steering_angles_deg:
        for candidate_yaw_rate_radps in candidate_yaw_rates_radps:
            for candidate_lateral_velocity_mps in candidate_lateral_velocities_mps:
                _, lateral_acceleration_mps2, yaw_acceleration_radps2 = calculate_bicycle_model_accelerations(vehicle_mass_kg, candidate_steering_angle_deg, forward_velocity_mps, candidate_lateral_velocity_mps, candidate_yaw_rate_radps, 0, 0, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, center_of_gravity_height_m)

                lateral_acceleration_error_mps2, yaw_acceleration_error_radps2 = calculate_steady_state_cornering_errors(forward_velocity_mps, candidate_yaw_rate_radps, lateral_acceleration_mps2, yaw_acceleration_radps2)

                lateral_condition_is_valid = abs(lateral_acceleration_error_mps2) <= lateral_acceleration_tolerance_mps2
                yaw_condition_is_valid = abs(yaw_acceleration_error_radps2) <= yaw_acceleration_tolerance_radps2

                if lateral_condition_is_valid and yaw_condition_is_valid:
                    if lateral_acceleration_mps2 > maximum_lateral_acceleration_mps2:
                        maximum_lateral_acceleration_mps2 = lateral_acceleration_mps2
                        best_steering_angle_deg = candidate_steering_angle_deg
                        best_lateral_velocity_mps = candidate_lateral_velocity_mps
                        best_yaw_rate_radps = candidate_yaw_rate_radps

    return best_steering_angle_deg, best_lateral_velocity_mps, best_yaw_rate_radps, maximum_lateral_acceleration_mps2

def find_longitudinal_acceleration_limits_at_target_lateral_acceleration(vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, target_lateral_acceleration_mps2, maximum_steering_angle_deg, number_of_steering_test_points, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points, maximum_absolute_slip_ratio, number_of_slip_ratio_test_points, lateral_acceleration_tolerance_mps2, yaw_acceleration_tolerance_radps2, center_of_gravity_height_m=0.0):
    if forward_velocity_mps <= 0:
        raise ValueError("Forward velocity must be greater than zero.")

    target_yaw_rate_radps = target_lateral_acceleration_mps2 / forward_velocity_mps

    if target_lateral_acceleration_mps2 >= 0:
        candidate_steering_angles_deg = np.linspace(0, maximum_steering_angle_deg, number_of_steering_test_points)
    else:
        candidate_steering_angles_deg = np.linspace(-maximum_steering_angle_deg, 0, number_of_steering_test_points)

    candidate_lateral_velocities_mps = np.linspace(-maximum_lateral_velocity_mps, maximum_lateral_velocity_mps, number_of_lateral_velocity_test_points)
    candidate_slip_ratios = np.linspace(-maximum_absolute_slip_ratio, maximum_absolute_slip_ratio, number_of_slip_ratio_test_points)

    maximum_longitudinal_acceleration_mps2 = -float("inf")
    minimum_longitudinal_acceleration_mps2 = float("inf")
    valid_candidate_was_found = False

    for candidate_steering_angle_deg in candidate_steering_angles_deg:
        for candidate_lateral_velocity_mps in candidate_lateral_velocities_mps:
            for candidate_slip_ratio in candidate_slip_ratios:
                longitudinal_acceleration_mps2, lateral_acceleration_mps2, yaw_acceleration_radps2 = calculate_bicycle_model_accelerations(vehicle_mass_kg, candidate_steering_angle_deg, forward_velocity_mps, candidate_lateral_velocity_mps, target_yaw_rate_radps, candidate_slip_ratio, candidate_slip_ratio, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, center_of_gravity_height_m)

                lateral_acceleration_error_mps2 = lateral_acceleration_mps2 - target_lateral_acceleration_mps2
                yaw_acceleration_error_radps2 = yaw_acceleration_radps2

                lateral_condition_is_valid = abs(lateral_acceleration_error_mps2) <= lateral_acceleration_tolerance_mps2
                yaw_condition_is_valid = abs(yaw_acceleration_error_radps2) <= yaw_acceleration_tolerance_radps2

                if lateral_condition_is_valid and yaw_condition_is_valid:
                    valid_candidate_was_found = True

                    if longitudinal_acceleration_mps2 > maximum_longitudinal_acceleration_mps2:
                        maximum_longitudinal_acceleration_mps2 = longitudinal_acceleration_mps2

                    if longitudinal_acceleration_mps2 < minimum_longitudinal_acceleration_mps2:
                        minimum_longitudinal_acceleration_mps2 = longitudinal_acceleration_mps2

    if not valid_candidate_was_found:
        return None, None

    return maximum_longitudinal_acceleration_mps2, minimum_longitudinal_acceleration_mps2