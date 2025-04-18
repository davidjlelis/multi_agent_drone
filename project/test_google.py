import google.generativeai as genai
import os

gemini_api = "AIzaSyCyUFIlmCg-bQ4gYSk-HdgKOdZviWgUT-8" 
# Set up your API key
genai.configure(api_key=gemini_api)

# Initialize the model
model = genai.GenerativeModel('gemini-1.5-flash')

# Generate content
response = model.generate_content("The opposite of hot is")
print(response.text)
