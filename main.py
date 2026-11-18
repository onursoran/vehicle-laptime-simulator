import math
import matplotlib.pyplot as plt
import numpy as np
from tire_model import calculate_lateral_tire_force, calculate_longitudinal_tire_force, apply_friction_circle, calculate_combined_tire_forces
from vehicle_model import calculate_vehicle_accelerations, calculate_static_axle_loads, calculate_axle_loads_with_longitudinal_transfer, calculate_axle_slip_angles, calculate_bicycle_model_accelerations, transform_tire_forces_to_vehicle_coordinates
from optimizer import find_maximum_straight_line_acceleration, find_maximum_straight_line_braking, calculate_steady_state_cornering_errors, find_maximum_steady_state_cornering_acceleration, find_longitudinal_acceleration_limits_at_target_lateral_acceleration
from ggv_map import generate_gg_diagram_at_speed


vertical_load_n = 700
cornering_stiffness_n_per_rad = 20000
friction_coefficient = 1.5

slip_angle_range_deg = np.linspace(-10, 10, 201)

lateral_force_range_n = []

for current_slip_angle_deg in slip_angle_range_deg:
    calculated_lateral_force_n = calculate_lateral_tire_force(current_slip_angle_deg, vertical_load_n, friction_coefficient, cornering_stiffness_n_per_rad)
    lateral_force_range_n.append(calculated_lateral_force_n)

plt.figure()
plt.plot(slip_angle_range_deg, lateral_force_range_n)
plt.xlabel("Slip Angle (Degrees)")
plt.ylabel("Lateral Force (N)")
plt.title("Simplified Tire Model")
plt.grid(True)


slip_ratio_range = np.linspace(-0.2, 0.2, 201)

longitudinal_force_range_n = []

for current_slip_ratio in slip_ratio_range:
    calculated_longitudinal_force_n = calculate_longitudinal_tire_force(current_slip_ratio, vertical_load_n, friction_coefficient, cornering_stiffness_n_per_rad)
    longitudinal_force_range_n.append(calculated_longitudinal_force_n)

plt.figure()
plt.plot(slip_ratio_range, longitudinal_force_range_n)
plt.xlabel("Slip Ratio")
plt.ylabel("Longitudinal Force (N)")
plt.title("Simplified Longitudinal Tire Model")
plt.grid(True)

maximum_tire_force_n = vertical_load_n * friction_coefficient

combined_longitudinal_force_n, combined_lateral_force_n = (apply_friction_circle(1000,1000,maximum_tire_force_n))

friction_circle_boundary = np.linspace(0, 2 * np.pi, 361)
longitudinal_coordinates = maximum_tire_force_n * np.cos(friction_circle_boundary)
lateral_coordinates = maximum_tire_force_n * np.sin(friction_circle_boundary)

plt.figure()
plt.plot(longitudinal_coordinates, lateral_coordinates)
plt.xlabel("Longitudinal force (N)")
plt.ylabel("Lateral force (N)")
plt.title("Tire Friction Circle")
plt.grid(True)
plt.axis("equal")

plt.scatter(1000, 1000, color="red", label="Requested force")
plt.scatter(combined_longitudinal_force_n, combined_lateral_force_n, color="green", label="Corrected Values")
plt.legend()


center_of_gravity_height_m = 0.30
vehicle_mass_kg = 300
center_of_gravity_to_front_axle_m = 1.0
center_of_gravity_to_rear_axle_m = 0.8
forward_velocity_mps = 10
yaw_moment_of_inertia_kgm2 = 150
longitudinal_stiffness_n = 10000

slip_ratio_sweep = np.linspace(-0.3, 0.3, 121)
acceleration_without_load_transfer_mps2 = []
acceleration_with_load_transfer_mps2 = []
front_axle_load_with_transfer_n = []
rear_axle_load_with_transfer_n = []

for current_slip_ratio in slip_ratio_sweep:
    longitudinal_acceleration_without_transfer_mps2, _, _ = calculate_bicycle_model_accelerations(vehicle_mass_kg, 0, forward_velocity_mps, 0, 0, current_slip_ratio, current_slip_ratio, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, 0.0)
    longitudinal_acceleration_with_transfer_mps2, _, _ = calculate_bicycle_model_accelerations(vehicle_mass_kg, 0, forward_velocity_mps, 0, 0, current_slip_ratio, current_slip_ratio, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, center_of_gravity_height_m)

    front_axle_load_n, rear_axle_load_n = calculate_axle_loads_with_longitudinal_transfer(vehicle_mass_kg, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, center_of_gravity_height_m, longitudinal_acceleration_with_transfer_mps2)

    acceleration_without_load_transfer_mps2.append(longitudinal_acceleration_without_transfer_mps2)
    acceleration_with_load_transfer_mps2.append(longitudinal_acceleration_with_transfer_mps2)
    front_axle_load_with_transfer_n.append(front_axle_load_n)
    rear_axle_load_with_transfer_n.append(rear_axle_load_n)

plt.figure()
plt.plot(slip_ratio_sweep, acceleration_without_load_transfer_mps2, label="No load transfer")
plt.plot(slip_ratio_sweep, acceleration_with_load_transfer_mps2, label="With load transfer")
plt.xlabel("Slip Ratio")
plt.ylabel("Longitudinal Acceleration (m/s2)")
plt.title("Straight-Line Acceleration With Longitudinal Load Transfer")
plt.grid(True)
plt.legend()

plt.figure()
plt.plot(acceleration_with_load_transfer_mps2, front_axle_load_with_transfer_n, label="Front axle")
plt.plot(acceleration_with_load_transfer_mps2, rear_axle_load_with_transfer_n, label="Rear axle")
plt.xlabel("Longitudinal Acceleration (m/s2)")
plt.ylabel("Vertical Load (N)")
plt.title("Axle Loads vs Longitudinal Acceleration")
plt.grid(True)
plt.legend()

lateral_acceleration_mps2, maximum_longitudinal_acceleration_mps2, minimum_longitudinal_accelerations_mps2 = generate_gg_diagram_at_speed(vehicle_mass_kg, forward_velocity_mps, center_of_gravity_to_front_axle_m, center_of_gravity_to_rear_axle_m, yaw_moment_of_inertia_kgm2, friction_coefficient, cornering_stiffness_n_per_rad, longitudinal_stiffness_n, 14.29, 8, 15, 61, 1.0, 81, 0.30, 121, 0.1, 0.1, center_of_gravity_height_m)
plt.figure()
plt.plot(lateral_acceleration_mps2, maximum_longitudinal_acceleration_mps2)
plt.xlabel("Lateral Acceleration (m/s2)")
plt.ylabel("Maximum Longitudinal Acceleration (m/s2)")
plt.title("lateral vs max longitudinal acceleration")
plt.grid(True)
plt.axis("equal")

plt.figure()
plt.plot(lateral_acceleration_mps2, minimum_longitudinal_accelerations_mps2)
plt.xlabel("Lateral Acceleration (m/s2)")
plt.ylabel("Minimum Longitudinal Acceleration (m/s2)")
plt.title("lateral vs min longitudinal acceleration")
plt.grid(True)
plt.axis("equal")

plt.show()