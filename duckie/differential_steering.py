#!/usr/bin/python3

import rospy
import tf

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
        rospy.Subscriber(left_wheel_tick_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_tick_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

        self.transform_broadcaster = tf.TransformBroadcaster()

        # set the robot at pose (0,0,0)
        self.publish_transform(x=0,y=0,theta=0)
       

    def callback_left_wheel_tick(self, data):
        left_tick_count = data.data
    
    def callback_right_wheel_tick(self, data):
        right_tick_count = data.data

    def run(self):
            while not rospy.is_shutdown():
                pass
        
    def publish_transform(self, x, y, theta):
        """publishes a transform between the map frame and the robot_name/base frame
                Parameters:
                    x = relative x position
                    y = relative y position
                    theta = relative rotation around z-axis
                    robot_name = name of duckiebot
        """
        q = quaternion_from_euler(0, 0 , theta)
        self.transform_broadcaster.sendTransform((x, y, 0.0), q, rospy.Time.now(),  robot_name + "/base", "map")


if __name__ == '__main__':
    robot_name = "alpha"
    ds = DifferentialSteering(robot_name)
    ds.run()
