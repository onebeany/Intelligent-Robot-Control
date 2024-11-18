import numpy as np


# Function to create rotation matrices for Z and Y axes
def rotation_matrix_z(angle):
    return np.array([
        [np.cos(angle), -np.sin(angle), 0],
        [np.sin(angle), np.cos(angle), 0],
        [0, 0, 1]
    ])


def rotation_matrix_y(angle):
    return np.array([
        [np.cos(angle), 0, np.sin(angle)],
        [0, 1, 0],
        [-np.sin(angle), 0, np.cos(angle)]
    ])


# Main function to calculate position based on four angles
def calculate_position(psi_1_deg, theta_1_deg, theta_2_deg, theta_3_deg):
    # Define
    r_1 = 7     # [cm]
    r_2 = 13    # [cm]
    r_3 = 12.5  # [cm]
    r_4 = 7     # [cm]

    # Convert degrees to radians
    psi_1   = np.deg2rad(psi_1_deg)
    theta_1 = np.deg2rad(theta_1_deg)
    theta_2 = np.deg2rad(theta_2_deg)
    theta_3 = np.deg2rad(theta_3_deg)

    # Compute combined rotation matrices based on Euler angles (assuming ZYX order)
    R_1 =       rotation_matrix_z(psi_1)
    R_2 = R_1 @ rotation_matrix_y(theta_1)
    R_3 = R_1 @ rotation_matrix_y(theta_2 + theta_1)
    R_4 = R_1 @ rotation_matrix_y(theta_3 + theta_2 + theta_1)

    # Position vectors
    x_1 = np.array([0, 0, r_1])
    x_2 = np.array([0, 0, r_2])
    x_3 = np.array([0, 0, r_3])
    x_4 = np.array([r_4, 0, 0])

    # Calculate final position
    position = R_4 @ x_4 + R_3 @ x_3 + R_2 @ x_2 + R_1 @ x_1

    return position


# Example usage
psi_1 =   0  # [deg]
theta_1 = 0  # [deg]
theta_2 = 0  # [deg]
theta_3 = 0  # [deg]
position = calculate_position(psi_1, theta_1, theta_2, theta_3)
orientation = [0, psi_1, theta_1 + theta_2 + theta_3]
print("Position (x, y, z):", position)
