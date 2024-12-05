#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np

class CameraSubscriber:
    
    def __init__(self, robot_name):
        # initialize a node with a name, annonymous=True ensures that the name is unique
        rospy.init_node('camera_listener', anonymous=True)
        # subscribe to a topic of type CompressedImage  
        topic = '/' + robot_name + '/camera_node/image/compressed'
        # when a message is received, the callback is invoked
        rospy.Subscriber(topic, CompressedImage, self.callback)
        rospy.sleep(2.0) # needed to make sure the node is indeed initialized
        
        # create a openCV <-> ROS bridge
        self.cv2_bridge = CvBridge()
        self.rate = rospy.Rate(10)  # the node is running at 10 hz

    def callback(self, data):
        # the callback should be light and fast
        self.image = data

    def do_image_processing(self):
        # image processing is done on the latest image received
        img = self.cv2_bridge.compressed_imgmsg_to_cv2(self.image, "bgr8")
        # cutoff the top half of the image, that part does not matter
        img = img[img.shape[0]//2:, :]

        cv2.imwrite('./raw.png', img)
        filtered, mask = self.filter_line(img)
        cv2.imwrite('./filtered.png', filtered)

        frame_height, frame_width, _ = filtered.shape

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Find the largest contour, assuming it is the line
        if contours is not None:
            contour = max(contours, key=cv2.contourArea)
            cx = self.get_line_position(contour, frame_width)

            # Draw the contour and center line for visualization
            cv2.drawContours(filtered, [contour], -1, (0, 255, 0), 3)
            cv2.line(filtered, (cx, 0), (cx, frame_height), (255, 0, 0), 2)
            cv2.imwrite('./contour.png', filtered)

            # Correct the robot's direction
            tolerance = 10
            if cx < frame_width // tolerance:
                # self.left()
                print('turn left')
            elif cx > 2 * frame_width // tolerance:
                # self.right()
                print('turn right')
            else:
                print("Moving forward...")
        else:
            print("no line detected")

        
    def filter_line(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([15, 20, 50])
        upper_yellow = np.array([60, 255, 255])

        #create a mask for green colour using inRange function
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return cv2.bitwise_and(img, img, mask=mask), mask
    
    def get_line_position(self, contour, frame_width):
        """Get the x-coordinate of the center of the line."""
        M = cv2.moments(contour)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            return cx
        return frame_width // 2  # Default to center if no contour

    def run(self):
        while not rospy.is_shutdown():
            self.do_image_processing()
            self.rate.sleep()


if __name__ == '__main__':
    robot_name = "phi"
    cs = CameraSubscriber(robot_name)
    cs.run()
