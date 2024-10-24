#!/usr/bin/env python3
import rospy
import vision
from qt_vosk_app.srv import speech_recognize
from qt_robot_interface.srv import speech_config, speech_say
from custom_interfaces.srv import Detectron
import std_msgs
import requests
import json
import gameloop

class Interaction:
    key = 'AIzaSyBMnXSIdpgPmUQyrAEqlSYvtkvArUnkteE'
    url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key={key}"
    headers = { 'Content-Type': 'application/json' }

    def __init__(self):
        self.emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', std_msgs.msg.String, queue_size=5)
        self.guesture_publisher = rospy.Publisher('/qt_robot/gesture/play', std_msgs.msg.String, queue_size=5)
        self.say_service = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
        self.recognize_service = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
        self.vision_service = rospy.ServiceProxy('custom/cv/detectron/detect', Detectron)
        speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)

        rospy.wait_for_service('custom/cv/detectron/detect')
        rospy.sleep(3.0)

        speechConfig("en-US",0,0)

    def feel(self, emotion) -> None:
        if emotion == "talking":
            self.emotion_publisher.publish(std_msgs.msg.String('QT/talking'))
        elif emotion == "happy":
            self.emotion_publisher.publish(std_msgs.msg.String('QT/happy'))
            self.emotion_publisher.publish(std_msgs.msg.String('QT/happy'))
            self.guesture_publisher.publish(std_msgs.msg.String('QT/clapping'))
        elif emotion == "angry":
            self.emotion_publisher.publish(std_msgs.msg.String('QT/angry'))
            self.guesture_publisher.publish(std_msgs.msg.String('QT/angry'))
        elif emotion == "sad":
            self.emotion_publisher.publish(std_msgs.msg.String('QT/cry'))
            self.guesture_publisher.publish(std_msgs.msg.String('QT/sad'))
    
    def speak(self, say: str) -> None:
        self.say_service(say)

    def hear(self) -> str:
        transcript = ''

        while transcript == '':
            resp = self.recognize_service("en_US", [], 20)
            transcript = resp.transcript

        rospy.loginfo(f"hears: {transcript}")
        return str(transcript).lower()

    def get_objects(self) -> list:
        try:
            response = self.vision_service()

            bounding_boxes = response.bounding_boxes

            objects = {bbox.class_name for bbox in bounding_boxes}
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return []

        rospy.loginfo(f"sees: {objects}")
        return list(objects)

    def get_llm_response(self, prompt: str) -> str:
        data = {
            "contents": [{
                "parts": [{"text": prompt}]
            }]
        }

        try:
            response = requests.post(self.url, headers=self.headers, data=json.dumps(data))
            response.raise_for_status()
            response_json = response.json()
            text = response_json["candidates"][0]["content"]["parts"][0]["text"]
            rospy.loginfo(f"llm response: {text}")
            return text
        except requests.exceptions.RequestException as e:
            rospy.loginfo(e)
            return ""


if __name__ == '__main__':
    rospy.init_node('my_python_node')

    rospy.loginfo("I spy started!")
    
    robot_interaction = Interaction()

    try:
        gameloop.start(robot_interaction)
    except KeyboardInterrupt:
        pass

    rospy.loginfo("Finished!")
