#!/usr/bin/python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header



class Breitenberg:

    def __init__(self, robot_name):
        rospy.init_node('breitenberg', anonymous=True)
        self.wheel_command_publisher = rospy.Publisher('/' + robot_name + '/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size = 1)
        self.wheel_command = WheelsCmdStamped()
        self.wheel_command.header = Header()
        self.camera_image_msg = CompressedImage()
        self.cv2_bridge = CvBridge()
        rospy.Subscriber('/' + robot_name + '/camera_node/image/compressed', CompressedImage, self.callback, queue_size=1, buff_size=2**30)
        self.rate = rospy.Rate(20)
        rospy.sleep(2.0)
  
  
    def run(self):
        last_image_stamp = self.camera_image_msg.header.stamp
        while not rospy.is_shutdown():
            if not self.camera_image_msg.header.stamp == last_image_stamp:
                vel_left, vel_right = self.do_image_analysis()
                self.turn_wheels(vel_left, vel_right)
                last_image_stamp = self.camera_image_msg.header.stamp
            self.rate.sleep()


    def callback(self, data):
        self.camera_image_msg = data
       

    def do_image_analysis(self):
        # note: you can use the Jupyter notebook `image_processing.ipynb` to learn how to do the image processing
        # and to test your code with a sample image
        
        # convert compressed image to cv2 image array
        image = self.cv2_bridge.compressed_imgmsg_to_cv2(self.camera_image_msg, "bgr8")
        # convert image to hsv space
        # note that cv2 uses colorspace bgr by default
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    

        # define the range of colors that we are interested in
        yellow_lower_hsv = np.array([0, 0, 0])
        yellow_upper_hsv = np.array([255, 255, 255])

        # create a mask that has a value of 0 if the pixel is outside the range, or 255 if it is inside the range
        mask = cv2.inRange(image_hsv, yellow_lower_hsv, yellow_upper_hsv)

        #  devide the image into a left and a right half

        # for each half, sum up the pixels of the mask
  
        default_speed = 0.1
        return default_speed, default_speed
       

    def turn_wheels(self, vel_left, vel_right):
        self.wheel_command.vel_left = vel_left
        self.wheel_command.vel_right = vel_right
        # set the timestamp
        self.wheel_command.header.stamp = rospy.Time.now()
        # and publish the wheel command
        self.wheel_command_publisher.publish(self.wheel_command)
        rospy.loginfo("left_command=" + str(self.wheel_command.vel_left) + " right_command=" + str(self.wheel_command.vel_right))


if __name__ == '__main__':
    b = Breitenberg("lamda")
    b.run()

