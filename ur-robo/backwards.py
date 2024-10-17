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


        """
        # wikipedia
        pitch = math.atan2(m[2, 0], m[2, 1])
        roll = math.acos(m[2, 2])
        yaw = -math.atan2(m[0, 2], m[1, 2])

        stackoverflow
        pitch = math.atan2(-m[2, 0], np.sqrt(m[0, 0] ** 2 + m[1, 0] ** 2))
        yaw = math.atan2(m[1, 0], m[0, 0])
        roll = math.atan2(m[2, 1], m[2, 2])

        chatgpt
        pitch = math.atan2(-m[2][0], np.sqrt(m[0][0] ** 2 + m[1][0] ** 2))
        yaw = math.atan2(m[1][0] / np.cos(pitch), m[0][0] / np.cos(pitch))
        roll = math.atan2(m[2][1] / np.cos(pitch), m[2][2] / np.cos(pitch)) 
        """


        #https://eecs.qmul.ac.uk/~gslabaugh/publications/euler.pdf
        if not np.isclose(m[2, 0], 1) and not np.isclose(m[2, 0], -1):
            pitch_1 = -math.asin(m[2, 0])
            pitch_2 = math.pi - pitch_1
            roll_1 = math.atan2(m[2, 1] / math.cos(pitch_1), m[2, 2] / math.cos(pitch_1))
            roll_2 = math.atan2(m[2, 1] / math.cos(pitch_2), m[2, 2] / math.cos(pitch_2))
            yaw_1 = math.atan2(m[1, 0] / math.cos(pitch_1), m[0, 0] / math.cos(pitch_1))
            yaw_2 = math.atan2(m[1, 0] / math.cos(pitch_2), m[0, 0] / math.cos(pitch_2))
            pitch = pitch_1
            roll = roll_1
            yaw = yaw_1
        else:
            roll = 0
            if np.isclose(m[2, 0], -1):
                pitch = math.pi / 2
                yaw = roll + math.atan2(m[0, 1], m[1, 1])
            else:
                pitch = -math.pi / 2
                yaw = -roll + math.atan2(-m[0, 1], -m[1, 1])

        return np.array([x, y, z, pitch, yaw, roll])


def calculate_inverse_kinematics(target_coordinates, theta):
    """
    target_coordinates: [x, y, z, x axis rotation, yaw, roll] 
    """
    x_target = target_coordinates
    theta = theta.copy()

    f_theta = 10
    i = 0
    while np.linalg.norm(f_theta) > 0.1 and i < 100:
        m_theta = calculate_forward_kinematics(theta)

        x_theta = get_eueler_angles(m_theta)
        f_theta = x_theta - x_target

        j_theta = jacobian_matrix(theta)
        j_theta_inv = np.linalg.pinv(j_theta)

        theta = theta - j_theta_inv @ f_theta
        i += 1

    return theta
