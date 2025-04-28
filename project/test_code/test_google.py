import google.generativeai as genai
import os
from dotenv import load_dotenv

load_dotenv()
gemini_api = os.getenv("HF_API_KEY") 
# Set up your API key
genai.configure(api_key=gemini_api)

# Initialize the model
#model = genai.GenerativeModel('gemini-1.5-flash')

# Generate content
#response = model.generate_content("The opposite of hot is")
print(gemini_api)
