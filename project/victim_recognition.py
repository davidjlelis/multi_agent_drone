# Purpose: Detects people in an image (via video stream) in disaster scenes
# Steps:
#   1. Grab video from file folder
#   2. Process each frame of the video to check for person via YOLO
#       a. If person found in frame, confirm person via VLM and pass VLM output through LLM
#       b. Else, move to the next frame
#   3. Grab output from LLM and export results to a results folder for evaluation

# Video footage taken from DroneStock

import numpy
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
        You are a search-and-rescue assistant in an area victim to a disaster and are tasked to confirm if people are 
        safe or injured. Currently, it is being done in a 3D simualtion so assume renderings are real. 
        Given the description below, provide a response only if a person is found.

        Description: "{description}"

        Respond with "Person Found" only if the description contains a person and if so, determine 
        if the person may be injured or in danger and what assistance they would need from first responders.
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
video_path = f'../videos/'
img_results_path = '../videos/results/images/'
json_results_path = '../videos/results/JSON/'



results_data = {"conf": 0.0, "person_found": False, "requires_assistance": False,
                "assistance_instructions": '', "detection_id": False, "video": False}
for video_file_name in os.listdir(video_path):
    if video_file_name.endswith('.mp4'):
        file_path = os.path.join(video_path, video_file_name)

        cap = cv2.VideoCapture(file_path)

        frame_number = 0
        _conf = 0

        json_file_name = f'{video_file_name.replace(".mp4", "")}_results.json'

        with open(f'{json_results_path}{json_file_name}', "w") as f:
            json.dump([], f)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # print(f'Processing frame {frame_number}')
            results = yolo_model(frame, verbose=False)[0]
            person_detected = False
            annotated_frame = results.plot()
            # cv2.imshow('YOLOv8 Detection', annotated_frame)

            for result in results.boxes.data:
                x1, y1, x2, y2, conf, cls = result.tolist()

                if int(cls) == 0:
                    if round(conf, 2) > 0.70:
                        if round(conf, 2)> round(_conf, 2):
                            _conf = conf
                        else:
                            person_detected = True
                            results_data['conf'] = _conf

            if person_detected:
                try:
                # Pass image through VLM to get description
                    prompt = "Describe the image"
                    inputs = VLM_processor(images=frame, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
                    output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
                    generated_text = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]

                # Pass descrption through LLM to get rescue guidance
                    LLM_response = is_emergency(generated_text)
                    #insert into client response
                    results_data['person_found'] = True
                    results_data['requires_assistance'] = True if 'Person Found' in LLM_response else False
                    results_data['assistance_instructions'] = LLM_response
                    results_data['detection_id'] = f'{video_file_name}_{frame_number}'
                    results_data['video'] = video_file_name
                except HfHubHTTPError as e:
                    if "402 Client Error" in str(e):
                        print('Max calls for free trier credits.')
                        continue
                    else:
                        raise

                if results_data['person_found']:
                    cv2.putText(annotated_frame, f"Person found...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                    
                    # cv2.imwrite(f'{yolo_results_path}{video_file_name}_results_{frame_number}.jpg', annotated_frame)
                    cv2.imwrite(f'{img_results_path}{video_file_name}_results_{frame_number}.jpg', annotated_frame) 
                    results_json = json.dumps(results_data)

                    with open(f'{json_results_path}{json_file_name}', "r") as f:
                        data = json.load(f)

                    data.append(results_json)

                    with open(f'{json_results_path}{json_file_name}', "w") as f:
                        json.dump(data, f, indent=4)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break 

            frame_number += 1   

cap.release()
cv2.destroyAllWindows()