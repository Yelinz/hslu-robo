
import numpy as np


def get_transformation_matrix(theta_i, i):
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    alpha = [3.14 / 2, 0, 0, 3.14 / 2, (-3.14) / 2, 0]

    return np.array([
        [
            np.cos(theta_i),
            -np.sin(theta_i) * np.cos(alpha[i]),
            np.sin(theta_i) * np.sin(alpha[i]),
            a[i] * np.cos(theta_i),
        ],
        [
            np.sin(theta_i),
            np.cos(theta_i) * np.cos(alpha[i]),
            -np.cos(theta_i) * np.sin(alpha[i]),
            a[i] * np.sin(theta_i),
        ],
        [0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]],
        [0, 0, 0, 1],
    ])


def derivation_approximation(theta, theta_i, coordinate_i):
    theta = theta.copy()

    base_matrix = calculate_end_orientation(theta)
    base_coordinates = get_end_position(base_matrix)

    h = 0.001
    theta[theta_i] += h

    new_matrix = calculate_end_orientation(theta)
    new_coordinates = get_end_position(new_matrix)

    return (new_coordinates[coordinate_i] - base_coordinates[coordinate_i]) / h


def jacobian_matrix(theta):
    return np.array([
        [
            derivation_approximation(theta, theta_i, coordinate_i) for theta_i in range(6)
        ] for coordinate_i in range(3)
    ])


def calculate_end_orientation(theta):
    m = np.identity(4)

    for i, theta_i in enumerate(theta):
        t = get_transformation_matrix(theta_i, i)
        m = m @ t

    return m


def get_end_position(m):
    return np.array([m[0, 3], m[1, 3], m[2, 3]])


def get_theta(target_coordinates, theta):
    # theta_target = [3.14 / 2, -3.14 / 2, 0, 0, 0, 0]
    # m_target = calculate_end_orientation(theta_target)
    #
    # x_target = np.array([0.2227859, -0.08589111, 0.6087095])
    x_target = target_coordinates
    theta = theta.copy()

    for i in range(50):
        m_theta = calculate_end_orientation(theta)

        x_theta = get_end_position(m_theta)
        f_theta = x_theta - x_target

        j_theta = jacobian_matrix(theta)
        j_theta_inv = np.linalg.pinv(j_theta)

        theta = theta - j_theta_inv @ f_theta

    return theta
