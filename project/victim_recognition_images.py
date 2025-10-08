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
from PIL import Image, UnidentifiedImageError
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

def is_emergency(description: str, message: str, environment_description: str) -> bool:
    prompt = f"""
            You are a search-and-rescue assistant for first responders in the follow environment: {environment_description}.

            The description below is of an image and the message is from a caller requesting first responder assistance. 
            Combining the context of the image and the description, respond with a full description of the scene.

            Description: "{description}"
            Message From Caller: "{message}"

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
img_folder_path = './images_dataset/sample/'
img_results_path = '../videos/results/images/'
json_results_path = '../videos/results/JSON/'
# messages_fp = os.path.join(img_folder_path,'messages.json')

with open('./images_dataset/messages.json', 'r') as m:
    messages = json.load(m)

# for i in messages:
#     print(i['file_name'])

best_conf = 0
detection_id = 0
detections_folder = './results/images_sim/'

test = 'test_10'

if not os.path.exists(detections_folder):
    os.makedirs(detections_folder)
results = []

prev_vlm_desc = []

results_json_path = './results/images_sim/'
results_json_filename = 'results.json'
# results_json_filepath = os.path.join(results_json_path, results_json_filename)
# with open(results_json_filepath, "w") as f:
#     json.dump([], f)

for file in os.listdir(img_folder_path):
    fn = file.replace('.png', '')
    fn = fn.replace('.jpg', '')
    fn = fn.replace('.jpeg', '')
    results_json_filepath = os.path.join(results_json_path+fn+'/', results_json_filename)

    if test == 'test_1':
        with open(results_json_filepath, "w") as f:
            json.dump([], f)
    '''
    results files
    {
        'img_path' : '../../../
        'world_and_scenario' : '..._file'
        "conf": 0,
        "person_found": true,
        "vlm_description": "...",
        "requires_assistance": false,
        "assistance_instructions": "...",
        "world_name": "image_dataset_human_damage",
        "detection_id": 1,
        "message": "We are trapped by a building."
    }
    '''
    # print('Processing file', file)
    results_data = {'img_path': img_folder_path+file
                    , 'world_and_scenario': 'image_sim_'+file
                    ,'conf': 0
                    , 'person_found': False
                    , 'vlm_description': None
                    , 'requires_assistance': None
                    , 'assistance_instructions': "image_dataset_human_damage"
                    , 'detection_id': -1
                    , 'message': None}
    try:
        img_path = os.path.join(img_folder_path, file)
        image = cv2.imread(img_path, cv2.COLOR_BGRA2BGR)
        pil_image = Image.open(img_path)
    except UnidentifiedImageError:
        print("Corrupt or unsupported file:", img_path)

    # cv2.imshow("image",image)
    # cv2.waitKey()

    max_conf = 0
    best_result = None

    yolo_image_rotations = [image
                            , cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
                            , cv2.rotate(image, cv2.ROTATE_180)
                            , cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)]

    for i in range(len(yolo_image_rotations)):
        results = yolo_model(yolo_image_rotations[i], verbose=False)[0]
        for box in results.boxes:
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            if cls in (0,2) and conf > max_conf and conf > 0.8:
                best_result = box
                best_yolo_image = yolo_image_rotations[i]
                max_conf = conf

                if i == 0:
                    image_rotate_angle = 0
                elif i == 1:
                    image_rotate_angle = -90
                elif i == 2:
                    image_rotate_angle = 180
                elif i == 3:
                    image_rotate_angle = 90
    
    if best_result is not None:
        x1, y1, x2, y2 = best_result.xyxy[0].tolist()
        conf = float(best_result.conf[0])
        cls = int(best_result.cls[0])
        results = {}
        if cls in (0, 2):
            person_detected = True
            results_data['conf'] = conf

            # set rotated image
            cv2.rectangle(best_yolo_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            if cls == 0:
                cv2.putText(best_yolo_image, f"Person {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
            elif cls == 2:
                cv2.putText(best_yolo_image, f"Car {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)

            if image_rotate_angle == 0:
                pass
            elif image_rotate_angle == 90:
                best_yolo_image = cv2.rotate(best_yolo_image, cv2.ROTATE_90_CLOCKWISE)
            elif image_rotate_angle == 180:
                best_yolo_image = cv2.rotate(best_yolo_image, cv2.ROTATE_180)
            elif image_rotate_angle == -90:
                best_yolo_image = cv2.rotate(best_yolo_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if person_detected: # and self.far_from_other_detections(new_detection=(est_x, est_y), detections=detections) and (est_x, est_y) not in detections:            
            # Florence2 VLM
            prompt = "<MORE_DETAILED_CAPTION>"
            inputs = VLM_processor(images=pil_image, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
            output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
            vlm_description = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]
            vlm_description = vlm_description.replace('3D rendering', '')
            detection_id += 1
            if vlm_description not in prev_vlm_desc:
                # print(f'VLM description: {vlm_description}')

                yolo_detection = f'{test}_{fn}.jpg'
                yolo_detection_path = os.path.join(detections_folder+fn+'/', yolo_detection)

                try:
                    for item in messages:
                        if item['file_name'] == file:
                            message = item['message']
                            environment_description = item['environment_description']
                    results_data['message'] = message
                    prev_vlm_desc.append(vlm_description)
                    # LLM Processing
                    LLM_response = is_emergency(description=vlm_description, message=message, environment_description=environment_description)

                    #insert into client response
                    results_data['vlm_description'] = vlm_description
                    results_data['person_found'] = True if '1. Person Found' in LLM_response else False
                    results_data['requires_assistance'] = True if '2. Person Requires Assistance' in LLM_response else False
                    results_data['assistance_instructions'] = LLM_response
                except HfHubHTTPError as e:
                    if "402 Client Error" in str(e):
                        print('Max calls for free trier credits.')

                        LLM_response = 'Max calls for free trier credits.'
                        results_data['vlm_description'] = vlm_description
                        results_data['person_found'] = True
                        results_data['requires_assistance'] = 'N/A'
                        results_data['assistance_instructions'] = LLM_response
                        results_data['detection_id'] = detection_id
                    else:
                        print(f'Error occured {e}. Skipping image')

                if results_data['person_found'] or person_detected:
                    if best_yolo_image is None or best_yolo_image.size == 0:
                        print("No valid image to save:", yolo_detection_path)
                    else:
                        cv2.imwrite(yolo_detection_path, best_yolo_image)
                    cv2.putText(image, f"Person found...", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                    results_data['detection_id'] = detection_id


                    # Export into results json file
                    results_data = json.dumps(results_data)
                    try:
                        with open(results_json_filepath, "r") as f:
                            data = json.load(f)

                        data.append(results_data)

                        with open(results_json_filepath, "w") as f:
                            json.dump(data, f, indent=4)
                        # print(f'Data sent to server: {telemetry_json}')
                    except BrokenPipeError:
                        print("⚠️ Connection lost while sending data. Skipping send.")
                        break  # Exit the run() loop if server is gone
    # cv2.imshow('Image', image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()