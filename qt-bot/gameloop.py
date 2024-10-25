import random
import rospy

def test(robot_interaction) -> None:
    rospy.loginfo("SPEAK")
    robot_interaction.feel("talking")
    robot_interaction.speak("HELLO THIS IS A TEST INPUT TO TEST THE WAIT LOGIC USING THE SERVICE PROXY")
    rospy.loginfo("DONE")
    robot_interaction.feel("happy")

def play(robot_interaction) -> str:
    object_list = robot_interaction.get_objects()
    selected_item = random.choice(object_list)
    rospy.loginfo(f"hidden word: {selected_item}")
    first_letter = selected_item[0].lower()
    robot_interaction.feel("talking")
    question = f'I spy with my little eye something beginning with "{first_letter}".'
    robot_interaction.speak(question)

    while True:    
        user_response = robot_interaction.hear() # z.b. ist es ein rundes objekt
        
        if "give up" in user_response:
            robot_interaction.feel("angry")
            text = f'Ok you give up, the word was: {selected_item}'
            robot_interaction.speak(text)
            break

        if selected_item in user_response:
            robot_interaction.feel("happy")
            success_text = f'Congrats, you have guessed the word correctly: {selected_item}'
            robot_interaction.speak(success_text)
            break

        prompt = f'Return a plain text response. We are playing I spy. You are the spy and have chosen "{selected_item}". Answer the following question about "{selected_item}" with only "Yes" or "No". If it does not make sense reponsed with "Ask a question". QUESTION: {user_response}'
        llm_reponse = robot_interaction.get_llm_response(prompt)
        robot_interaction.feel("talking")
        robot_interaction.speak(llm_reponse)

def play_reverse(robot_interaction):
    question = robot_interaction.hear()

    interactions = []
    availble_objects = robot_interaction.get_objects()
    tries = len(availble_objects)
    for i in range(tries):
        if i >= tries - 1:
            robot_interaction.feel("angry")
            robot_interaction.speak("I give up I do not know.")
            continue

        prompt = f'Return a plain text response. We are playing I spy. You are the guesser. The hint is: "{question}". The objects you see are: {", ".join(availble_objects)}.'
        if len(interactions):
            prompt += f' Your past guesses and their answers are: {", ".join(interactions)}'
        llm_reponse = robot_interaction.get_llm_response(prompt)
        robot_interaction.feel("talking")
        robot_interaction.speak(llm_reponse)

        answer = robot_interaction.hear()
        if "correct" in answer:
            break
        interactions.append((prompt, answer))


def start(interactions):
    while True:
        play(interactions)

        interactions.feel("talking")
        interactions.speak("Do you want to play again?")
        resp = interactions.hear()        
        if "no" in resp:
            break

        interactions.feel("talking")
        interactions.speak("Now you are the spy and I am the guesser.")
        play_reverse(interactions)

        interactions.feel("talking")
        interactions.speak("Do you want to play again?")
        resp = interactions.hear()        
        if "no" in resp:
            break

    interactions.feel("sad")
    interactions.speak("Thank you for playing!")
