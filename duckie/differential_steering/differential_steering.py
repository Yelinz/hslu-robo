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
        right_msg = rospy.wait_for_message(right_wheel_tick_topic, WheelEncoderStamped)
        self.initial_left_tick = left_msg.data
        self.initial_right_tick = right_msg.data
        self.left_tick_count = self.initial_left_tick
        self.right_tick_count = self.initial_right_tick
        self.prev_left_tick = 0
        self.prev_right_tick = 0

        self.left_resolution = left_msg.resolution
        self.right_resolution = right_msg.resolution
        
        self.axis_width = 10
        self.x = 0
        self.y = 0
        self.angle = 0
        self.radius = 5

        rospy.Subscriber(left_wheel_tick_topic, WheelEncoderStamped, self.callback_left_wheel_tick)
        rospy.Subscriber(right_wheel_tick_topic, WheelEncoderStamped, self.callback_right_wheel_tick)

        self.transform_broadcaster = tf.TransformBroadcaster()

        # set the robot at pose (0,0,0)
        self.publish_transform(x=0, y=0, theta=0)
       

    def callback_left_wheel_tick(self, data):
        #rospy.loginfo_throttle(1, f"left: {self.left_tick_count}")
        self.left_tick_count = data.data
        self.calc_pose()
        self.publish_transform(self.x, self.y, self.angle)
    
    def callback_right_wheel_tick(self, data):
        #rospy.loginfo_throttle(1, f"right: {self.right_tick_count}")
        self.right_tick_count = data.data
        self.calc_pose()
        self.publish_transform(self.x, self.y, self.angle)

    def ticks_to_pose(self, left_ticks, right_ticks):
        return

    def turn_wheels(self, velocity):
        self.command.vel_left = velocity
        self.command.vel_right = -velocity*0.5

        self.command.header.stamp = rospy.Time.now()
        self.publisher.publish(self.command)
        rospy.loginfo("Publishing wheel command")

    def run(self):
        prev_left = self.left_tick_count
        prev_right = self.right_tick_count
        target_x = 10
        target_y = 0
        target_angle = 0

        while not rospy.is_shutdown():
            # TODO Backwards
            """
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
            heading += (x_t - left_delta) / axis_width
            # TODO fix angle calculation, then it should work
            # https://robotics.stackexchange.com/questions/1653/calculate-position-of-differential-drive-robot?rq=1
            # https://rossum.sourceforge.net/papers/DiffSteer/

            distance_left = self.radius*self.angle # s_L
            distance_right = (self.radius+self.axis_width*self.angle) # s_R
            distance = (distance_left+distance_right)/2 # s overline
            self.angle += (distance_left-distance_right)/(2*self.axis_width) # theta
            self.x += distance*math.cos(self.angle)
            self.y += distance*math.sin(self.angle)
            velocity_x = 0
            velocity_y = 0
            """

            rospy.loginfo_throttle(0.2, f"x: {self.x}, y: {self.y}, theta: {self.angle}")
        
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
        delta_theta = (distance_travelled_right_wheel - distance_travelled_left_wheel) / self.axis_width

        # updating the pose
        self.x = self.x + displacement * math.cos(self.angle + (delta_theta/2)) /100
        self.y = self.y + displacement * math.sin(self.angle + (delta_theta/2)) /100
        self.angle = self.angle + delta_theta
        self.prev_left_tick += left_tick
        self.prev_right_tick += right_tick

if __name__ == '__main__':
    robot_name = "lamda"
    ds = DifferentialSteering(robot_name)
    ds.run()
