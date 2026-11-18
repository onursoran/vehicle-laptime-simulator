import math
from tire_model import calculate_combined_tire_forces


def calculate_vehicle_accelerations(vehicle_mass_kg, front_longitudinal_force_n, rear_longitudinal_force_n, front_lateral_force_n, rear_lateral_force_n, CG_to_front_distance_m, CG_to_rear_distance_m, yaw_moi_kgm2):
    total_longitudinal_forces_n = front_longitudinal_force_n + rear_longitudinal_force_n
    total_lateral_forces_n = front_lateral_force_n + rear_lateral_force_n

    longitudinal_acceleration_mps2 = total_longitudinal_forces_n / vehicle_mass_kg
    lateral_acceleration_mps2 = total_lateral_forces_n / vehicle_mass_kg

    front_yaw_moment_nm = CG_to_front_distance_m * front_lateral_force_n
    rear_yaw_moment_nm = CG_to_rear_distance_m * rear_lateral_force_n
    net_yaw_moment_nm = front_yaw_moment_nm - rear_yaw_moment_nm

    yaw_acceleration_radps2 = net_yaw_moment_nm / yaw_moi_kgm2

    return longitudinal_acceleration_mps2, lateral_acceleration_mps2, yaw_acceleration_radps2

def calculate_static_axle_loads (vehicle_mass_kg, CG_to_front_distance_m, CG_to_rear_distance_m):
    wheelbase_length_m = CG_to_front_distance_m + CG_to_rear_distance_m
    vehicle_weight_n = vehicle_mass_kg * 9.81
    front_axle_vertical_load_n = (vehicle_weight_n * CG_to_rear_distance_m)/wheelbase_length_m
    rear_axle_vertical_load_n = (vehicle_weight_n * CG_to_front_distance_m)/wheelbase_length_m

    return front_axle_vertical_load_n, rear_axle_vertical_load_n

def calculate_axle_loads_with_longitudinal_transfer(vehicle_mass_kg, CG_to_front_distance_m, CG_to_rear_distance_m, center_of_gravity_height_m, longitudinal_acceleration_mps2):
    front_axle_vertical_load_n, rear_axle_vertical_load_n = calculate_static_axle_loads(vehicle_mass_kg, CG_to_front_distance_m, CG_to_rear_distance_m)
    wheelbase_length_m = CG_to_front_distance_m + CG_to_rear_distance_m
    longitudinal_load_transfer_n = (vehicle_mass_kg * longitudinal_acceleration_mps2 * center_of_gravity_height_m) / wheelbase_length_m

    front_axle_vertical_load_n = front_axle_vertical_load_n - longitudinal_load_transfer_n
    rear_axle_vertical_load_n = rear_axle_vertical_load_n + longitudinal_load_transfer_n

    vehicle_weight_n = vehicle_mass_kg * 9.81
    if front_axle_vertical_load_n < 0.0:
        front_axle_vertical_load_n = 0.0
        rear_axle_vertical_load_n = vehicle_weight_n
    elif rear_axle_vertical_load_n < 0.0:
        rear_axle_vertical_load_n = 0.0
        front_axle_vertical_load_n = vehicle_weight_n

    return front_axle_vertical_load_n, rear_axle_vertical_load_n

def calculate_axle_slip_angles(steering_angle_deg, forward_velocity_mps, lateral_velocity_mps, yaw_rate_radps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m):
    steering_angle_rad = math.radians(steering_angle_deg)
    
    front_axle_lateral_velocity_mps = lateral_velocity_mps + yaw_rate_radps * center_of_gravity_to_front_axle_m
    rear_axle_lateral_velocity_mps = lateral_velocity_mps - yaw_rate_radps * center_of_gravity_to_rear_axle_m

    front_axle_travel_direction_rad = math.atan2(front_axle_lateral_velocity_mps, forward_velocity_mps)
    rear_axle_travel_direction_rad = math.atan2(rear_axle_lateral_velocity_mps, forward_velocity_mps)

    front_slip_angle_rad = steering_angle_rad - front_axle_travel_direction_rad
    rear_slip_angle_rad = -rear_axle_travel_direction_rad

    front_slip_angle_deg = math.degrees(front_slip_angle_rad)
    rear_slip_angle_deg = math.degrees(rear_slip_angle_rad)

    return front_slip_angle_deg, rear_slip_angle_deg

def calculate_bicycle_model_accelerations(vehicle_mass_kg, steering_angle_deg, forward_velocity_mps, lateral_velocity_mps, yaw_rate_radps, front_slip_ratio, rear_slip_ratio, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, center_of_gravity_height_m=0.0, load_transfer_iterations=3):
    front_slip_angle_deg, rear_slip_angle_deg = calculate_axle_slip_angles(steering_angle_deg, forward_velocity_mps, lateral_velocity_mps, yaw_rate_radps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m)

    longitudinal_acceleration_mps2 = 0.0
    lateral_acceleration_mps2 = 0.0
    yaw_acceleration_radps2 = 0.0

    for _ in range(load_transfer_iterations):
        front_axle_vertical_load_n, rear_axle_vertical_load_n = calculate_axle_loads_with_longitudinal_transfer(vehicle_mass_kg, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, center_of_gravity_height_m, longitudinal_acceleration_mps2)

        front_tire_longitudinal_force_n, front_tire_lateral_force_n = calculate_combined_tire_forces(front_slip_angle_deg, front_slip_ratio, front_axle_vertical_load_n, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n)
        front_vehicle_longitudinal_force_n, front_vehicle_lateral_force_n = transform_tire_forces_to_vehicle_coordinates(front_tire_longitudinal_force_n, front_tire_lateral_force_n, steering_angle_deg)

        rear_vehicle_longitudinal_force_n, rear_vehicle_lateral_force_n = calculate_combined_tire_forces(rear_slip_angle_deg, rear_slip_ratio, rear_axle_vertical_load_n, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n)

        longitudinal_acceleration_mps2, lateral_acceleration_mps2, yaw_acceleration_radps2 = calculate_vehicle_accelerations(vehicle_mass_kg, front_vehicle_longitudinal_force_n, rear_vehicle_longitudinal_force_n, front_vehicle_lateral_force_n, rear_vehicle_lateral_force_n, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2)

    return longitudinal_acceleration_mps2, lateral_acceleration_mps2, yaw_acceleration_radps2

def transform_tire_forces_to_vehicle_coordinates(tire_longitudinal_force_n, tire_lateral_force_n, steering_angle_deg):
    steering_angle_rad = math.radians(steering_angle_deg)
    vehicle_longitudinal_force_n = tire_longitudinal_force_n * math.cos(steering_angle_rad) - tire_lateral_force_n * math.sin(steering_angle_rad)
    vehicle_lateral_force_n = tire_longitudinal_force_n * math.sin(steering_angle_rad) + tire_lateral_force_n * math.cos(steering_angle_rad)
    return vehicle_longitudinal_force_n, vehicle_lateral_force_n

