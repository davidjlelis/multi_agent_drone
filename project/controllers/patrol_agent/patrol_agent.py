# Copyright 1996-2024 Cyberbotics Ltd.
# Licensed under the Apache License, Version 2.0
# See the License for details: https://www.apache.org/licenses/LICENSE-2.0

from controller import Robot, Motor, InertialUnit, GPS, Gyro
import math
import socket
import time
import sys
import struct
import numpy as np
import cv2
from PIL import Image
import torch
from transformers import AutoProcessor, AutoModelForCausalLM, AutoTokenizer, Pipeline
from ultralytics import YOLO
import google.generativeai as genai
from dotenv import load_dotenv
import os
import requests
from huggingface_hub import InferenceClient

SAVE_PATH = "latest_detection.jpg"

# login(hf_token)

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

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic(Robot):
    K_VERTICAL_THRUST = 68.5
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0
    K_ROLL_P = 50.0
    K_PITCH_P = 30.0
    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precision = 1

    def __init__(self):
        super().__init__()

        self.time_step = int(self.getBasicTimeStep())

        #self.name = self.getName()
        #self.client = self.setup_client_connection(name=self.name)

        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(0.7)

        for motor in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = [0] * 6
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

    def set_position(self, pos):
        self.current_pose = pos

    def setup_client_connection(self, host='localhost', port=8000, name="drone"):
        try:
            client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client.connect((host, port))
            client.send(self.name.encode())
            print(f"[CLIENT] Connected to server at {host}:{port} as {self.name}")
            return client
        except Exception as e:
            print(f"[CLIENT ERROR] Could not connect to server: {e}")
            sys.exit(1)
        

    def move_to_target(self, waypoints):
        if self.target_position[0:2] == [0, 0]:
            self.target_position[0:2] = waypoints[0]

        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            self.target_index = (self.target_index + 1) % len(waypoints)
            self.target_position[0:2] = waypoints[self.target_index]

        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1],
            self.target_position[0] - self.current_pose[0]
        )

        angle_left = self.target_position[2] - self.current_pose[5]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        return yaw_disturbance, pitch_disturbance

    def estimate_person_location(self, drone_loc):
        x_d, y_d = drone_loc['x_d'], drone_loc['y_d']
        altitude, pitch, yaw = drone_loc['altitude'], drone_loc['pitch'], drone_loc['yaw']

        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)

        if pitch >= 90:
            return round(x_d, 5), round(y_d, 5)

        d = altitude * math.tan(pitch_rad)
        x_p = x_d + d * math.cos(yaw_rad)
        y_p = y_d + d * math.sin(yaw_rad)

        return round(x_p, 5), round(y_p, 5)

    def is_emergency(self, description: str) -> bool:
        prompt = f"""
            You are a search-and-rescue assistant in a project simulation. Assume any 3D models of people are real people.
            Determine if the following description indicates a dangerous or emergency situation. Lean on the side of cautious.

            Description: "{description}"

            Respond with "Yes" or "No". If "Yes", provide guidance for first responders.
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

    def run(self):
        print(f'Starting up {self.getName()}')
        t1 = self.getTime()

        roll_disturbance = pitch_disturbance = yaw_disturbance = 0
        waypoints = [[10, 10], [-10, 10], [-10, -10], [10, -10]]
        self.target_altitude = 10

        # Get waypoints from server
        # data = self.client.recv(1024).decode()
        # x, y = map(float, data.split(','))
        # waypoints = [x, y]


        initial_yolo_time = -10

        self.last_yolo_time = initial_yolo_time

        while self.step(self.time_step) != -1:
            current_time = self.getTime()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acc, pitch_acc, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            telemetry_data = {
                "x_d": x_pos, "y_d": y_pos, "altitude": altitude,
                "roll": roll, "pitch": pitch, "yaw": yaw
            }

            raw_image = self.camera.getImage()
            img_array = np.frombuffer(raw_image, dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
            rgb_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
            pil_image = Image.fromarray(rgb_image)

            # YOLO person detection
            if current_time - self.last_yolo_time >= 10.0:
                self.last_yolo_time = current_time
                # print('Running YOLO detection')

                # yolo_results = yolo_model(rgb_image)
                # person_detected = any(det.cls == 0 for det in yolo_results[0].boxes)  # class 0 is 'person' in COCO

                # if person_detected:
                #     print('Person Detected!')

                # Florence 2 injury detection
                # if person_detected:
                    # print("YOLO detected a person. Running Florence-2 for confirmation...")
                prompt = "Describe the image"

                inputs = VLM_processor(images=pil_image, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
                output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
                generated_text = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]

                print("Florence-2 Output:", generated_text)

                LLM_response = self.is_emergency(generated_text)
                print('LLM Response:', LLM_response)

            if altitude > self.target_altitude - 1:
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)
                    t1 = self.getTime()

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acc + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acc + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_altitude_diff = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_altitude_diff, 3.0)

            self.front_left_motor.setVelocity(self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input)
            self.front_right_motor.setVelocity(- (self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input))
            self.rear_left_motor.setVelocity(- (self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input))
            self.rear_right_motor.setVelocity(self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input)


robot = Mavic()
robot.run()
