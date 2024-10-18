import google.generativeai as genai
import os

genai.configure(api_key=os.environ["API_KEY"])

model = genai.GenerativeModel("gemini-1.5-flash")


def process(object_list) -> str:
    selected_item = object_list[0]
    first_letter = selected_item[0]
    question = f'I spy with my little eye something beginning with {first_letter}.'
    speak(question)
    


def speak(say: str):
    pass

response = model.generate_content("Write a story about a magic backpack.")
print(response.text)