#!/usr/bin/python3

import rospy
import math
import numpy as np
import tf2_ros as tf

import geometry_msgs.msg
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler

class DifferentialSteering:

    def __init__(self, robot_name):
        self.robot_name = robot_name
        rospy.init_node('differential_steering', anonymous=True)
        
        # create publisher for wheel commands
        wheel_command_topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.wheel_command_publisher = rospy.Publisher(wheel_command_topic, WheelsCmdStamped, queue_size = 10)
        
        # subscribe wheel tick messages
        left_wheel_tick_topic = '/' + robot_name + '/left_wheel_encoder_node/tick'
        right_wheel_tick_topic = '/' + robot_name + '/right_wheel_encoder_node/tick'
        left_msg = rospy.wait_for_message(left_wheel_tick_topic, WheelEncoderStamped)
        right_msg = rospy.wait_for_message(right_wheel_tick_topic, WheelEncoderStamped)
        # Tick to position variables
        self.left_resolution = left_msg.resolution
        self.right_resolution = right_msg.resolution
        self.initial_left_tick = left_msg.data
        self.initial_right_tick = right_msg.data
        self.left_tick_count = self.initial_left_tick
        self.right_tick_count = self.initial_right_tick
        self.prev_left_tick = 0
        self.prev_right_tick = 0
        # Constants
        self.axis_width = 0.08
        self.radius = 0.1
        # Calculated position
        self.x = 0
        self.y = 0
        self.angle = 0

        rospy.Subscriber(left_wheel_tick_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_tick_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

        self.transform_broadcaster = tf.TransformBroadcaster()

        # set the robot at pose (0,0,0)
        self.publish_transform(x=0, y=0, theta=0)
       

    def callback_left_wheel_tick(self, data):
        self.left_tick_count = data.data
        self.calc_pose()
        self.publish_transform(self.x, self.y, self.angle)
    
    def callback_right_wheel_tick(self, data):
        self.right_tick_count = data.data
        self.calc_pose()
        self.publish_transform(self.x, self.y, self.angle)

    def turn_wheels(self, velocity_left=0, velocity_right=0):
        command = WheelsCmdStamped()
        command.header = Header()
        command.vel_left = velocity_left
        command.vel_right = velocity_right
        command.header.stamp = rospy.Time.now()
        self.wheel_command_publisher.publish(command)

    def run(self):
        self.turn_wheels()
        x_end = 1
        y_end = 1
        theta_end = 3.14/2

        velocity_max = 2
        angular_velocity_max = 1

        distance = np.sqrt((x_end-self.x)**2 + (y_end-self.y)**2)
        delta_theta = theta_end -self.angle 

        t_linear = distance / velocity_max
        t_angular = delta_theta / angular_velocity_max

        estimated_time = 1

        velocity = distance / estimated_time
        angular_velocity = delta_theta / estimated_time

        wheel_offset = ((angular_velocity*self.axis_width) / 2)
        target_velocity_left = velocity - wheel_offset
        target_velocity_right = velocity + wheel_offset
    
        self.turn_wheels(target_velocity_left, target_velocity_right)
        rospy.sleep(estimated_time)
        self.turn_wheels()
        while not rospy.is_shutdown():
            self.turn_wheels()
            rospy.loginfo_throttle(0.2, f"x: {self.x}, y: {self.y}, theta: {self.angle}")
            """
            if abs(x_end - self.x) < 0.001 and abs(y_end-self.y) < 0.001:
                break
            distance = np.sqrt((x_end-self.x)**2 + (y_end-self.y)**2)
            delta_theta = theta_end -self.angle 

            t_linear = distance / velocity_max
            t_angular = delta_theta / angular_velocity_max

            estimated_time = 1 

            wheel_offset = ((angular_velocity*self.axis_width) / 2)
            target_velocity_left = velocity - wheel_offset
            target_velocity_right = velocity + wheel_offset
            self.turn_wheels(target_velocity_left, target_velocity_right)

            rospy.sleep(estimated_time)
            """
        self.turn_wheels()
        
    def publish_transform(self, x, y, theta):
        """publishes a transform between the map frame and the robot_name/base frame
                Parameters:
                    x = relative x position
                    y = relative y position
                    theta = relative rotation around z-axis
                    robot_name = name of duckiebot
        """
        q = quaternion_from_euler(0, 0 , theta)

        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = f"{robot_name}/base" 
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # self.transform_broadcaster.sendTransform((x, y, 0.0), q, rospy.Time.now(),  robot_name + "/base", "map")

        self.transform_broadcaster.sendTransform(t)

    def calc_pose(self) -> None:
        left_tick = self.left_tick_count - self.initial_left_tick - self.prev_left_tick
        right_tick = self.right_tick_count - self.initial_right_tick - self.prev_right_tick

        # calculating wheel displacements
        distance_travelled_left_wheel = (2*math.pi*self.radius*left_tick) / self.left_resolution
        distance_travelled_right_wheel = (2*math.pi*self.radius*right_tick) / self.right_resolution

        # calculating robot's linear and angular displacements
        displacement = (distance_travelled_left_wheel + distance_travelled_right_wheel) / 2
        delta_theta = (distance_travelled_right_wheel - distance_travelled_left_wheel) / (2*self.axis_width)

        # updating the pose
        self.x = self.x + displacement * math.cos(self.angle + (delta_theta/2))
        self.y = self.y + displacement * math.sin(self.angle + (delta_theta/2))
        self.angle = self.angle + delta_theta
        self.prev_left_tick += left_tick
        self.prev_right_tick += right_tick

if __name__ == '__main__':
    robot_name = "lamda"
    ds = DifferentialSteering(robot_name)
    ds.run()
