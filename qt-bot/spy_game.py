import google.generativeai as genai
import random
import os

genai.configure(api_key=os.environ["API_KEY"])
model = genai.GenerativeModel("gemini-1.5-flash")


def play(object_list) -> str:
    selected_item = random.choice(object_list)
    first_letter = selected_item[0]
    question = f'I spy with my little eye something beginning with "{first_letter}".'
    speak(question)

    user_success = False
    while not user_success:    
        user_response = get_user_response() # z.b. ist es ein rundes objekt

        if selected_item.lower() in user_response.lower():
            user_success = True
            success_text = f'Congrats, I have guessed the word correctly: {selected_item}'
            speak(success_text)
            continue

        question_mark = '?'
        prompt = f'Answer the following question about the object "{selected_item}" with only "Yes" or "No": {user_response.strip(question_mark)}{question_mark}'
        llm_reponse = get_llm_response(prompt)
        speak(llm_reponse)

def speak(say: str) -> None:
    print(say)
    pause()
    pass

def get_user_response() -> str:
    return input() # test
    pass

def get_llm_response(prompt: str) -> str:
    return model.generate_content(prompt).candidates[0].content.parts[0].text

def pause() -> None:
    pass

if __name__ == "__main__":
    play(['Apple', 'Pineapple', 'Car', 'House', 'Human', 'Dog', 'Bot'])