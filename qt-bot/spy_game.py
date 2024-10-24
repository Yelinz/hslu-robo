import random
import os
import json
import requests
import std_msgs
import vision
import rospy

# genai.configure(api_key=)
# model = genai.GenerativeModel("gemini-1.5-flash")

public_emotion_publisher = None
public_say_publisher = None
public_recognize_proxy = None

def play(object_list, emotion_publisher, say_publisher, recognize_proxy) -> str:
    global public_emotion_publisher
    global public_say_publisher
    global public_recognize_proxy
    public_emotion_publisher = emotion_publisher
    public_say_publisher = say_publisher
    public_recognize_proxy = recognize_proxy

    selected_item = random.choice(object_list)
    first_letter = selected_item[0]
    question = f'I spy with my little eye something beginning with "{first_letter}".'
    speak(question)
    rospy.sleep(5)

    user_success = False
    while not user_success:    
        user_response = get_user_response() # z.b. ist es ein rundes objekt

        if user_response == "exit":
            break

        if selected_item.lower() in user_response.lower():
            user_success = True
            success_text = f'Congrats, you have guessed the word correctly: {selected_item}'
            speak(success_text)
            break

        question_mark = '?'
        prompt = f'We are playing I spy. Answer the following question about the object "{selected_item}" with only "Yes" or "No": {user_response.strip(question_mark)}{question_mark}'
        llm_reponse = get_llm_response(prompt)
        rospy.loginfo(f"llm response {llm_reponse}")
        speak(llm_reponse)
        rospy.sleep(60)

def play_reverse():
    question = get_user_response()

    while True:
        availble_objects = vision.get_objects()
        prompt = f'We are playing I spy'
        llm_reponse = get_llm_response(prompt)
        answer = get_user_response()
        sleep(5)

def speak(say: str) -> None:
    global public_emotion_publisher
    global public_say_publisher

    public_emotion_publisher.publish(std_msgs.msg.String('QT/talking'))
    public_say_publisher.publish(std_msgs.msg.String(say))

    pass

# hear
def get_user_response() -> str:
    global public_recognize_proxy

    resp = public_recognize_proxy("en_US", [], 20)
    rospy.loginfo(f"hear {resp.transcript}")
    return str(resp.transcript)

def get_llm_response(prompt: str) -> str:
    # return model.generate_content(prompt).candidates[0].content.parts[0].text
    KEY = '<<enter key>>'
    # KEY = os.environ["API_KEY"]
    url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key={KEY}"
    headers = { 'Content-Type': 'application/json' }
    data = {
        "contents": [{
            "parts": [{"text": prompt}]
        }]
    }

    try:
        response = requests.post(url, headers=headers, data=json.dumps(data))
        response.raise_for_status()
        response_json = response.json()
        return response_json["candidates"][0]["content"]["parts"][0]["text"]
    except requests.exceptions.RequestException as e:
        rospy.loginfo(e)
        return ""


def pause() -> None:
    pass

# if __name__ == "__main__":
#     play(['Apple', 'Pineapple', 'Car', 'House', 'Human', 'Dog', 'Bot'])
