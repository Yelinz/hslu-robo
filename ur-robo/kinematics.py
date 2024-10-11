#! /usr/bin/env python3

import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from urdf_parser_py.urdf import URDF
from backwards import get_theta


class ForwardKinematics:
    def __init__(self):
        # we create two publishers in our ROS node
        # one to publish the angles of the joints (joint states)
        # and another for visualizing a target pose (to see whether your calculations are correct)
        self.robot_name = "beta"
        self.joint_state_publisher = rospy.Publisher(self.robot_name+'_joint_states', JointState, queue_size=1) 
        self.pose_publisher = rospy.Publisher(self.robot_name+'_pose', PoseStamped, queue_size=1)
        rospy.init_node(self.robot_name+'_joint_state_publisher')
        self.rate = rospy.Rate(0.1)
        rospy.sleep(3.0)

    def run(self):
        # to get the kinematic chain with the joints and the corresponding parameters, we use the urdf parser
        # documentation of urdf_parser_py see http://wiki.ros.org/urdfdom_py
        robot = URDF.from_parameter_server()
        root = robot.get_root()
        tip = "beta_tool0"
        joint_names = robot.get_chain(root, tip, joints=True, links=False, fixed=False)
        # the properties of a given joint / link can be obtained with the joint_map
        # see http://wiki.ros.org/urdf/XML/joint

        joint_angles = [3.14/2, -3.14/2, 0, 0, 0, 0]  # in radians

        # create the joint state messages
        js = JointState()
        js.name = joint_names
        js.position = joint_angles

        end_effector_pose = self.calculate_forward_kinematics(
            joint_angles
        )

        #print(end_effector_pose @ joint_angles)
        if end_effector_pose is not None:
            target_pose_message = self.get_pose_message_from_matrix(end_effector_pose)
        else:
            rospy.logerr(
                "error, no target pose calculated, use identity matrix instead"
            )
            target_pose_message = self.get_pose_message_from_matrix(np.identity(4))


        # publish the joint state values and the target pose
        i=0
        #iinverse_target = self.calculate_inverse_kinematics(joint_angles, [0.5, 0.1, 0.5])
        #print(inverse_target)
        while not rospy.is_shutdown():
            """
            angles = [i, 0, 0, 0, 0, 0]
            js = JointState()
            js.name = joint_names
            js.position = angles
            end_effector_pose = self.calculate_forward_kinematics(
                angles, joint_names, robot
            )
            target_pose_message = self.get_pose_message_from_matrix(end_effector_pose)
            """
            i += 0.01
            target_angles = get_theta([0.5, 0.1+i, 0.5], joint_angles)
            js = JointState()
            js.name = joint_names
            js.position = target_angles
            end_effector_pose = self.calculate_forward_kinematics(target_angles)
            target_pose_message = self.get_pose_message_from_matrix(end_effector_pose)

            self.joint_state_publisher.publish(js)
            self.pose_publisher.publish(target_pose_message)
            rospy.sleep(0.5)

    def calculate_forward_kinematics(self, joint_positions):
        pose = np.eye(4)

        for i, joint_angle in enumerate(joint_positions):
            pose = pose @ self.get_transformation_matrix(joint_angle, i)

        return pose

    def get_transformation_matrix(self, theta, n):
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

    def get_eueler_angles(self, angles):
        end_effector_pose = self.calculate_forward_kinematics(angles)
        x = end_effector_pose[0][3]
        y = end_effector_pose[1][3]
        z = end_effector_pose[2][3]
        pitch = math.atan2(-end_effector_pose[2][0], math.sqrt(end_effector_pose[0][0] ** 2 + end_effector_pose[1][0] ** 2))
        yaw = math.atan2(end_effector_pose[1][0] / math.cos(pitch), end_effector_pose[0][0] / math.cos(pitch))
        roll = math.atan2(end_effector_pose[2][1] / math.cos(pitch), end_effector_pose[2][2] / math.cos(pitch)) 
        return np.array([x, y, z, pitch, yaw, roll])
    
    def calculate_inverse_kinematics(self, start_angles, end_coordinates):
        """
        start_pose = self.calculate_forward_kinematics(start_angles)
        end_pose = np.eye(4)
        end_pose[0][3] = end_coordinates[0]
        end_pose[1][3] = end_coordinates[1]
        end_pose[2][3] = end_coordinates[2]
        """

        theta_max_step = 0.2
        start_pose = self.calculate_forward_kinematics(start_angles)
        print(f'start pose: {start_pose}')

        # current_positions = np.array([start_pose[0][3], start_pose[1][3], start_pose[2][3]])
        current_positions = self.get_eueler_angles(start_angles)
        end_coordinates = np.array([0.5, 0.1, 0.5, 0, 0, 0])
        delta_p = end_coordinates - current_positions
        step = 0
        max_steps = 1000
        while np.linalg.norm(delta_p) > 0.01 and step < max_steps:
            print(f"start inverse kinematics: {current_positions}, {np.linalg.norm(delta_p)}")
            # Get the jacobian matrix given the current joint angles
            J_j = self.jacobian(start_angles)

            # Calculate the pseudo-inverse of the Jacobian matrix
            J_invj = np.linalg.pinv(J_j)

            # Multiply the two matrices together
            v_Q = np.matmul(J_invj, delta_p)

            # Move the joints to new angles
            # We use the np.clip method here so that the joint doesn't move too much. We
            # just want the joints to move a tiny amount at each time step because
            # the full motion of the end effector is nonlinear, and we're approximating the
            # big nonlinear motion of the end effector as a bunch of tiny linear motions.
            clipped = np.clip(
                v_Q, -1 * theta_max_step, theta_max_step
            )
            start_angles += clipped 

            # Get the current position of the end-effector in the global frame
            # p_j = self.position(Q_j, p_i=p_eff_N)
            end_pose = self.calculate_forward_kinematics(start_angles)
            current_positions = np.array([end_pose[0][3], end_pose[1][3], end_pose[2][3]])
            current_positions = self.get_eueler_angles(start_angles)

            # Increment the time step
            step += 1

            # Determine the difference between the new position and the desired end position
            delta_p = end_coordinates - current_positions
        rospy.loginfo_throttle(0.1, f"finish inverse kinematics: {current_positions}, {np.linalg.norm(delta_p)}")

        return end_pose

    def jacobian(self, angles):
        # Position of the end effector in global frame
        start_positions = self.get_eueler_angles(angles)
        for i in range(6):
            joint_angle = angles.copy()
            joint_angle[i] += 0.000001
            new_positions = self.get_eueler_angles(joint_angle)
            delta = start_positions - new_positions
            joint_jacobian = np.array([[delta[0]], [delta[1]], [delta[2]]])
            if i == 0:
                jacobian_matrix = joint_jacobian
            else:
                jacobian_matrix = np.concatenate(
                    (jacobian_matrix, joint_jacobian), axis=1
                )

        return jacobian_matrix


    # the following function creates a PoseStamped message from a homogeneous matrix
    def get_pose_message_from_matrix(self, matrix):
        """Return pose msgs from homogen matrix
        matrix : homogeneous matrix 4x4
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base"
        pose = pose_stamped.pose
        pose.position.x = matrix[0][3]
        pose.position.y = matrix[1][3]
        pose.position.z = matrix[2][3]

        q = self.get_quaternion_from_matrix(matrix)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose_stamped

    # the ROS message type PoseStamped uses quaternions for the orientation
    def get_quaternion_from_matrix(self, matrix):
        """Return quaternion from homogeneous matrix
        matrix : homogeneous matrix 4x4
        """
        q = np.empty((4,), dtype=np.float64)
        M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
        t = np.trace(M)
        if t > M[3, 3]:
            q[3] = t
            q[2] = M[1, 0] - M[0, 1]
            q[1] = M[0, 2] - M[2, 0]
            q[0] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 0, 1, 2
            if M[1, 1] > M[0, 0]:
                i, j, k = 1, 2, 0
            if M[2, 2] > M[i, i]:
                i, j, k = 2, 0, 1
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
        return q


if __name__ == "__main__":
    fk = ForwardKinematics()
    fk.run()
