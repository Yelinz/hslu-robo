import numpy as np

def calculate_forward_kinematics(joint_positions):
    pose = np.eye(4)

    for i, joint_angle in enumerate(joint_positions):
        pose = pose @ get_transformation_matrix(joint_angle, i)

    return pose

def get_transformation_matrix(theta, n):
    a = [0, -0.24355, -0.2132, 0, 0, 0]
    d = [0.15185, 0, 0, 0.13105, 0.08535, 0.0921]
    alpha = [3.14 / 2, 0, 0, 3.14 / 2, (-3.14) / 2, 0]

    M = [
        [
            np.cos(theta),
            -np.sin(theta) * np.cos(alpha[n]),
            np.sin(theta) * np.sin(alpha[n]),
            a[n] * np.cos(theta),
        ],
        [
            np.sin(theta),
            np.cos(theta) * np.cos(alpha[n]),
            -np.cos(theta) * np.sin(alpha[n]),
            a[n] * np.sin(theta),
        ],
        [0, np.sin(alpha[n]), np.cos(alpha[n]), d[n]],
        [0, 0, 0, 1],
    ]

    return M
