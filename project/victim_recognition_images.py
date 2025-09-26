# Purpose: Detects people in an image (via video stream) in disaster scenes
# Steps:
#   1. Grab video from file folder
#   2. Process each frame of the video to check for person via YOLO
#       a. If person found in frame, confirm person via VLM and pass VLM output through LLM
#       b. Else, move to the next frame
#   3. Grab output from LLM and export results to a results folder for evaluation

# Video footage taken from DroneStock

import numpy as np
import cv2
from PIL import Image
import torch
from transformers import AutoProcessor, AutoModelForCausalLM, AutoTokenizer, Pipeline
from ultralytics import YOLO
from dotenv import load_dotenv
import os
# import requests
from huggingface_hub import InferenceClient
from huggingface_hub.errors import HfHubHTTPError
import json

# Load YOLO model
print('Loading YOLO Model...')
yolo_model = YOLO("yolov8n.pt")
print('YOLO Model completed!')

# Vision-Language Model: Load Florence-2 model
VL_model_name = "microsoft/Florence-2-base"
print('Loading Florence2-base Model...')
device = "cuda" if torch.cuda.is_available() else "cpu"
torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

VLM_processor = AutoProcessor.from_pretrained(VL_model_name, trust_remote_code=True)
VL_model = AutoModelForCausalLM.from_pretrained(VL_model_name, torch_dtype=torch_dtype, trust_remote_code=True).to(device)
print('Florence2-base Model complete!')

# Large Language Model (Qwen2.5-72B-Instruct)
API_URL = "https://router.huggingface.co/nebius/v1/chat/completions"        

try:
    load_dotenv()
    hf_api = os.getenv("HUGGING_FACE_HUB_TOKEN")
except KeyError:
    print("Error: HUGGING_FACE_HUB_TOKEN environment variable not set.")
    exit()

client = InferenceClient(provider="nebius", api_key=hf_api)

def is_emergency(description: str) -> bool:
    prompt = f"""
            You are a search-and-rescue assistant for first responders in a city after an earthquake.

            The description below is of an image and the message is from a caller requesting first responder assistance. 
            Combining the context of the image and the description, respond with a full description of the scene.

            Description: "{description}"
            Message From Caller: "We are trapped by a building."

            Respond with "1. Person Found" if and only if the description includes a person. If no preson is found, state "1. No Person Found"
            If so, include in the response "2. Person Requires Immediate Assistance" if and only if the person may be injured or in a dangerous situation based on the Description and Message From Caller. If they are not injured or in a dangerous situation, state "2. Person does not require assistance". 
            If the person does require assistance, include "3. Assistance needed" and the kind of assistance they would need from first responders based on the Description and Message From Caller.
        """
    
    completion = client.chat.completions.create(
        model="Qwen/Qwen2.5-72B-Instruct",
        messages=[
            {
                "role": "user",
                "content": prompt
            }
        ],
        max_tokens=512,
    )

    return completion.choices[0].message["content"]

# video_file_name = 'waterfall'
# scene_path = './results/scenes/'
img_path = './disaster_images_dataset/'
img_results_path = '../videos/results/images/'
json_results_path = '../videos/results/JSON/'

best_conf = 0

results_data = {"conf": 0.0, "person_found": False, "requires_assistance": False,
                "assistance_instructions": '', "detection_id": False, "video": False}

for folder in os.listdir(img_results_path):
    print(folder)

# for scene in os.listdir(scene_path):
#     if scene.endswith('.jpg'):
#         file_path = os.path.join(scene_path, scene)

#         scene_img = cv2.imread(file_path)
#         # cv2.imshow('Image', scene_img)
#         # cv2.waitKey(0)
#         height, width, channels = scene_img.shape

#         img_array = np.frombuffer(scene_img, dtype=np.uint8).reshape((height, width, channels))
#         bgr_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR) # YOLO
#         rgb_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
#         pil_image = Image.fromarray(rgb_image)

#         json_file_name = f'{scene.replace(".jpg", "")}_results.json'

#         with open(f'{json_results_path}{json_file_name}', "w") as f:
#             json.dump([], f)

#         # print(f'Processing frame {frame_number}')
#         results = yolo_model(bgr_image, verbose=False)[0]
#         person_detected = False
#         annotated_frame = results.plot()
#         # cv2.imshow('YOLOv8 Detection', annotated_frame)
#         # cv2.waitKey(0)

#         for result in results.boxes.data:
#             x1, y1, x2, y2, conf, cls = result.tolist()

#             print(cls)

#             if int(cls) == 0:
#                 if round(conf, 2) > 0.40:
#                     person_detected = True
#                     results_data['conf'] = conf
#                     print('Person found! ', conf)
#                     best_conf = 0

#         if person_detected:
#             try:
#             # Pass image through VLM to get description
#                 prompt = "Describe the image"
#                 inputs = VLM_processor(images=pil_image, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
#                 output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
#                 generated_text = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]
#                 print(generated_text)

#             # Pass descrption through LLM to get rescue guidance
#                 LLM_response = is_emergency(generated_text)
#                 print(LLM_response)
#                 #insert into client response
#                 results_data['person_found'] = True
#                 results_data['requires_assistance'] = True if 'Person Found' in LLM_response else False
#                 results_data['assistance_instructions'] = LLM_response
#                 results_data['detection_id'] = f'{scene}'
#                 results_data['video'] = scene
#             except HfHubHTTPError as e:
#                 if "402 Client Error" in str(e):
#                     print('Max calls for free trier credits.')
#                     continue
#                 else:
#                     raise

#             if results_data['person_found']:
#                 cv2.putText(annotated_frame, f"Person found...", (10, 30),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                
#                 # cv2.imwrite(f'{yolo_results_path}{video_file_name}_results_{frame_number}.jpg', annotated_frame)
#                 cv2.imwrite(f'{img_results_path}{scene}_results.jpg', annotated_frame) 
#                 results_json = json.dumps(results_data)

#                 with open(f'{json_results_path}{json_file_name}', "r") as f:
#                     data = json.load(f)

#                 data.append(results_json)

#                 with open(f'{json_results_path}{json_file_name}', "w") as f:
#                     json.dump(data, f, indent=4)

#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             break 

# cv2.destroyAllWindows()