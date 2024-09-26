#!/usr/bin/python3

import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from std_msgs.msg import Header



class WheelCommandPublisher:

    def __init__(self, robot_name): 
        # initialize a node with a name, annonymous=True ensures that the name is unique
        rospy.init_node('wheel_command_publisher', anonymous=True)

        # create a publisher for a topic of type duckietown_msgs.msg.WheelsCmdStamped
        topic = '/' + robot_name + '/wheels_driver_node/wheels_cmd'
        self.publisher = rospy.Publisher(topic, WheelsCmdStamped, queue_size = 10)
        rospy.sleep(2.0)
        
        # create a WheelsCmdStamped message
        self.command = WheelsCmdStamped()
        self.command.header = Header()
   

    def turn_wheels(self, velocity):
        self.command.vel_left = velocity
        self.command.vel_right = velocity

        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)
        rospy.loginfo("Publishing wheel command")

    def run(self):
        self.turn_wheels(0.3)
        rospy.sleep(3) # wait 3 seconds
        # stop wheels
        self.turn_wheels(0.0)

if __name__ == '__main__':
    wcp = WheelCommandPublisher("lamda")
    wcp.run()
