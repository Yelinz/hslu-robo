#!/usr/bin/env python3
import rospy
import vision
import llm
from qt_vosk_app.srv import speech_recognize
from qt_robot_interface.srv import speech_config
import std_msgs

if __name__ == '__main__':
    rospy.init_node('my_python_node')

    rospy.loginfo("my_python_node started!")
    
    param1 = rospy.get_param('~param1', 'default_value')
    rospy.loginfo("value of param1 is %s", param1)

    # init publishers
    emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', std_msgs.msg.String, queue_size=0)
    say_publisher = rospy.Publisher('/qt_robot/speech/say', std_msgs.msg.String, queue_size=0)
    recognize_proxy = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)

    rospy.sleep(3.0)

    try:
        objects = vision.get_objects()
        rospy.loginfo(objects)

        status = speechConfig("en-US",0,0)
        llm.play(objects, emotion_publisher, say_publisher, recognize_proxy)
        rospy.spin()
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finished!")
