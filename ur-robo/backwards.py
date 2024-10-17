import numpy as np
import math
from forwards import calculate_forward_kinematics

def derivation_approximation(theta, theta_i, coordinate_i):
    theta = theta.copy()

    base_matrix = calculate_forward_kinematics(theta)
    base_coordinates = get_eueler_angles(base_matrix)

    h = 0.001
    theta[theta_i] += h

    new_matrix = calculate_forward_kinematics(theta)
    new_coordinates = get_eueler_angles(new_matrix)

    return (new_coordinates[coordinate_i] - base_coordinates[coordinate_i]) / h


def jacobian_matrix(theta):
    return np.array([
        [
            derivation_approximation(theta, theta_i, coordinate_i) for theta_i in range(6)
        ] for coordinate_i in range(6)
    ])


def get_eueler_angles(m):
        x = m[0, 3]
        y = m[1, 3]
        z = m[2, 3]


        # wikipedia

        pitch = math.atan2(m[2, 0], m[2, 1])
        roll = math.acos(m[2, 2])
        yaw = -math.atan2(m[0, 2], m[1, 2])

        """
        stackoverflow
        yaw = math.atan2(m[1, 0], m[0, 0])
        pitch = math.atan2(-m[2, 0], np.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2))
        roll = math.atan2(m[2, 1], m[2, 2])

        chatgpt
        pitch = math.atan2(-m[2][0], np.sqrt(m[0][0] ** 2 + m[1][0] ** 2))
        yaw = math.atan2(m[1][0] / np.cos(pitch), m[0][0] / np.cos(pitch))
        roll = math.atan2(m[2][1] / np.cos(pitch), m[2][2] / np.cos(pitch)) 
        """
        return np.array([x, y, z, pitch, yaw, roll])


def calculate_inverse_kinematics(target_coordinates, theta):
    """
    target_coordinates: [x, y, z, pitch, yaw, roll] 
    """
    x_target = target_coordinates
    theta = theta.copy()

    f_theta = 10
    i = 0
    while np.linalg.norm(f_theta) > 0.1 and i < 100:
        m_theta = calculate_forward_kinematics(theta)

        x_theta = get_eueler_angles(m_theta)
        print(x_theta)
        f_theta = x_theta - x_target

        j_theta = jacobian_matrix(theta)
        j_theta_inv = np.linalg.pinv(j_theta)

        theta = theta - j_theta_inv @ f_theta
        i += 1

    return theta
