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
from huggingface_hub.errors import HfHubHTTPError
import json
from cryptography.fernet import Fernet
sys.path.append("../..")  # Allow imports from the parent directory
from key_manager import encryption_key  # Import the shared key
import re

cipher = Fernet(encryption_key)

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
        self.target_altitude = 10

        # Connect to server to get waypoints
        self.server_ip = 'localhost'
        self.server_port = 5555
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_ip, self.server_port))
        self.waypoints = self.get_waypoints_from_server()
        print(f"Recieved {len(self.waypoints)} from waypoints from server")

    def get_waypoints_from_server(self):
        try:
            print(f"[Waypoint Client] Connected to server at {self.server_ip}:{self.server_port}")

            data_size_bytes = b''
            while len(data_size_bytes) < 8:
                packet = self.client_socket.recv(8 - len(data_size_bytes))
                if not packet:
                    raise Exception("Connection closed while reading data size.")
                data_size_bytes += packet

            data_size = int.from_bytes(data_size_bytes, byteorder='big')
            print(f"[Waypoint Client] Expecting {data_size} bytes of data.")

            data = b''
            while len(data) < data_size:
                packet = self.client_socket.recv(min(4096, data_size - len(data)))
                if not packet:
                    raise Exception("Connection closed while reading waypoint data.")
                data += packet

            self.client_socket.sendall(b'READY')
            waypoints = json.loads(data.decode('utf-8'))
            print("[Waypoint Client] Received and parsed waypoints successfully.")
            return waypoints

        except Exception as e:
            print(f"[Waypoint Client] Error: {e}")
            sys.exit(1)


    def set_position(self, pos):
        self.current_pose = [float(p) for p in pos]  # Ensure all positions are floats
        
    def move_to_target(self):
        # Ensure target_position and current_pose are floats
        target_position_floats = [float(x) for x in self.target_position]
        current_pose_floats = [float(x) for x in self.current_pose[0:2]]

        # defaults to the edge of the environment
        if target_position_floats[0:2] == [0, 0]:
            self.target_position[0:2] = [self.waypoints[0]['y'], self.waypoints[0]['x'], 0]

        # Compare positions to check if the drone has reached the target
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(target_position_floats, current_pose_floats)]):
            self.target_index = (self.target_index + 1) % len(self.waypoints)
            self.target_position[0:2] = [self.waypoints[self.target_index]['y'], self.waypoints[self.target_index]['x'], 0]
        # Ensure float conversion during calculations
        self.target_position[2] = np.arctan2(
            float(self.target_position[1]) - float(self.current_pose[1]),
            float(self.target_position[0]) - float(self.current_pose[0])
        )

        angle_left = self.target_position[2] - float(self.current_pose[5])
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if angle_left > np.pi:
            angle_left -= 2 * np.pi

        #yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        yaw_disturbance = clamp(self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi), -self.MAX_YAW_DISTURBANCE, self.MAX_YAW_DISTURBANCE)
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        # Check if the drone is sufficiently close to the target, then reduce disturbance
        if abs(angle_left) < 0.05:  # Small angle difference threshold to stop rotating
            yaw_disturbance = 0
            pitch_disturbance = 0      

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
            You are a search-and-rescue assistant in an area victim to a disaster and are tasked to confirm if people are 
            safe or injured. Given the description below, provide a response.

            Description: "{description}"

            Respond with "Person Found" if the description contains a person. If the description contains a person, determine 
            if the person may be injured and what assistance they would need. Format the response in a JSON format:

            {{
                "person_found": True or False,
                "requires_assistance": True or False,
                "assistance_instructions": "instructions"
            }}
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
    
    def extract_json_from_text(self, text):
        """
        Extracts and returns a valid JSON object (as a dict) from a string containing additional content.
        """

        print(text)
        try:
            # Use regex to find a JSON-like block in the text
            match = re.search(r'\{[\s\S]*\}', text)
            if match:
                json_str = match.group(0)
                # Convert to dict
                return json.loads(json_str)
            else:
                raise ValueError("No JSON object found in the text.")
        except json.JSONDecodeError as e:
            print("Invalid JSON format:", e)
        except Exception as e:
            print("Error:", e)
        return {'person_found': False, 'requires_assistance': False, 'assistance_instructions': False}

    def run(self):
        print(f'Starting up {self.getName()}')
        t1 = self.getTime()

        roll_disturbance = pitch_disturbance = yaw_disturbance = 0

        initial_yolo_time = -10
        self.last_yolo_time = initial_yolo_time

        # Connect to server to send mapping information
        client = self.client_socket

        # initiate confidence values
        best_conf = 0.0

        while self.step(self.time_step) != -1:
            # current_time = self.getTime()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acc, pitch_acc, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            telemetry_data = {
                "x_d": x_pos, "y_d": y_pos, "altitude": altitude,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "conf": 0.0, "person_found": False, "requires_assistance": False,
                "assistance_instructions": ''
            }

            raw_image = self.camera.getImage()
            img_array = np.frombuffer(raw_image, dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
            bgr_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR) # YOLO
            rgb_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
            pil_image = Image.fromarray(rgb_image)

            # Run YOLOv8 to find people
            results = yolo_model(bgr_image, verbose=False)[0]
            person_detected = False

            for result in results.boxes.data:
                x1, y1, x2, y2, conf, cls = result.tolist()
                
                if int(cls) == 0: # If person is detected to a 90% confidence
                    # print(conf)
                    if conf > 0.8:
                        # if the new conf is better than the currently best conf, set best_conf to new conf
                        if conf > best_conf:
                            best_conf = conf
                        # else, there isn't a better conf and set person_detected is True
                        else:
                            person_detected = True
                            # est_x, est_y = self.estimate_person_location(telemetry_data)
                            cv2.rectangle(bgr_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(bgr_image, f"Person {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
                            telemetry_data['conf'] = best_conf

            if person_detected:
                #print("YOLO has detected a person. Running Florence-2 to get image description")
                prompt = "Describe the image"
                inputs = VLM_processor(images=pil_image, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
                output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
                generated_text = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]
                cv2.imwrite("yolo_detection.jpg", bgr_image)
                #print("Person confirmed. Florence-2 Output:", generated_text)
                response = {'person_found': False, 'requires_assistance': False, 'assistance_instructions': False}
                try:
                    # LLM Processing
                    LLM_response = self.is_emergency(generated_text)
                    #print('LLM Response:', LLM_response)
                    response = self.extract_json_from_text(LLM_response)

                    #insert into client response
                    telemetry_data['person_found'] = response['person_found']
                    telemetry_data['requires_assistance'] = response['requires_assistance']
                    telemetry_data['assistance_instructions'] = response['assistance_instructions']
                except HfHubHTTPError as e:
                    if "402 Client Error" in str(e):
                        print('Max calls for free trier credits.')
                    else:
                        raise
                
                #print(response)

                if telemetry_data['person_found']:
                    cv2.putText(bgr_image, f"Person found...", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                    cv2.imwrite("vlm_detection.jpg", bgr_image)

                    # Encrypt and send telemetry data for mapping
                    telemetry_json = json.dumps(telemetry_data)
                    encrypted_telemetry = cipher.encrypt(telemetry_json.encode('utf-8'))
                    try:
                        client.sendall(len(encrypted_telemetry).to_bytes(8, byteorder='big'))
                        client.sendall(encrypted_telemetry)
                    except BrokenPipeError:
                        print("⚠️ Connection lost while sending data. Skipping send.")
                        break  # Exit the run() loop if server is gone

                best_conf = 0.0


            if altitude > self.target_altitude - 1:
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target()
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
        
        # After loop:
        try:
            client.sendall(len(cipher.encrypt(b"BYE")).to_bytes(8, byteorder='big'))
            client.sendall(cipher.encrypt(b"BYE"))
        except:
            print("⚠️ Server already closed. Skipping BYE.")
        finally:
            client.close()


robot = Mavic()
robot.run()
