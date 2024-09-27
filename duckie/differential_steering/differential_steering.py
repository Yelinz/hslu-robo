#!/usr/bin/python3

import rospy
import math
import tf2_ros as tf

import geometry_msgs.msg
from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
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
        right_msg = rospy.wait_for_message(left_wheel_tick_topic, WheelEncoderStamped)
        self.left_tick_count = left_msg.data
        self.right_tick_count = right_msg.data
        self.left_resolution = left_msg.resolution
        self.right_resolution = right_msg.resolution
        print(self.left_resolution)

        rospy.Subscriber(left_wheel_tick_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_tick_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

        self.transform_broadcaster = tf.TransformBroadcaster()

        # set the robot at pose (0,0,0)
        self.publish_transform(x=0,y=0,theta=0)
       

    def callback_left_wheel_tick(self, data):
        #rospy.loginfo_throttle(1, f"left: {self.left_tick_count}")
        self.left_tick_count = data.data
    
    def callback_right_wheel_tick(self, data):
        #rospy.loginfo_throttle(1, f"right: {self.right_tick_count}")
        self.right_tick_count = data.data

    def run(self):
        axis_width = 10
        x = 0
        y = 0
        heading = 0
        prev_left = self.left_tick_count
        prev_right = self.right_tick_count
        while not rospy.is_shutdown():
            # TODO Backwards
            left_delta = self.left_tick_count - prev_left
            right_delta = self.right_tick_count - prev_right

            prev_left = self.left_tick_count
            prev_right = self.right_tick_count

            if (left_delta - right_delta <= 2):
                x += left_delta * math.cos(heading)
                y += right_delta * math.sin(heading)
                rospy.loginfo_throttle(0.2, f"x: {x}, y: {y}, theta: {heading}")
                continue

            R = axis_width * (left_delta + right_delta) / (2 * (right_delta - left_delta))
            wd = (right_delta - left_delta) / axis_width

            x_t = R * math.sin(wd + heading) - R * math.sin(heading)
            y_t = R * math.cos(wd + heading) + R * math.cos(heading)
            x += x_t
            y -= y_t
            # TODO fix angle calculation, then it should work
            # https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot?rq=1
            heading += (x_t - left_delta) / axis_width
            rospy.loginfo_throttle(0.2, f"x: {x}, y: {y}, theta: {heading}")
        
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


if __name__ == '__main__':
    robot_name = "lamda"
    ds = DifferentialSteering(robot_name)
    ds.run()
