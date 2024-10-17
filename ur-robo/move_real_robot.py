#! /usr/bin/env python3

import rospy
import socket
from backwards import calculate_inverse_kinematics


class RealRobotArm:

    def __init__(self):

        host = rospy.get_param("robot_ip")
        port_ur = 30002
        port_gripper = 63352

        rospy.init_node('my_real_robot')
        rospy.sleep(3.0)        
        # Create socket connection to robot arm and gripper
        self.socket_ur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_ur.connect((host, port_ur))
        self.socket_gripper = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_gripper.connect((host, port_gripper))
        # activate the gripper
        self.socket_gripper.sendall(b'SET ACT 1\n')


    def send_joint_command(self, joint_angles):
        values = ', '.join(['{:.2f}'.format(i) if type(i) == float else str(i) for i in joint_angles])
        self.socket_ur.send (str.encode("movej(["+ values + "])\n"))

    def send_gripper_command(self, value):
        if (value >= 0 and value <= 255):
            command = 'SET POS ' + str(value) + '\n'
            self.socket_gripper.send(str.encode(command))
            # make the gripper move
            self.socket_gripper.send(b'SET GTO 1\n')


    def close_connection(self):
        self.socket_ur.close()
        self.socket_gripper.close()

if __name__ == '__main__':
    robot = RealRobotArm()

    sleep_const = 3

    # send joint angles
    joint_angles = [0, -1.57, 0, 0, 0, 0] # upright position
    robot.send_joint_command(joint_angles)
    rospy.sleep(sleep_const)

    start_angles = calculate_inverse_kinematics([-0.25, 0.45, 0.15, 0, 3.14, 3.14/2], joint_angles)
    robot.send_joint_command(start_angles)
    rospy.sleep(sleep_const)

    # set requested postion to 100 (value between 0-255)
    robot.send_gripper_command(0)
    rospy.sleep(sleep_const)
    
    lift_angles = calculate_inverse_kinematics([-0.25, 0.45, 0.31, 0, 3.14, 3.14/2], start_angles)
    robot.send_joint_command(lift_angles)
    rospy.sleep(sleep_const)

    move_angles = calculate_inverse_kinematics([0.25, 0.45, 0.31, 0, 3.14, 3.14/2], lift_angles)
    robot.send_joint_command(move_angles)
    rospy.sleep(sleep_const)

    drop_angles = calculate_inverse_kinematics([0.25, 0.45, 0.15, 0, 3.14, 3.14/2], move_angles)
    robot.send_joint_command(drop_angles)
    rospy.sleep(sleep_const)

    robot.send_gripper_command(100)
    rospy.sleep(sleep_const)

    robot.send_joint_command(joint_angles)
    rospy.sleep(sleep_const)

    robot.close_connection()