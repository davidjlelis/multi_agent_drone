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
import math

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
        self.camera.setFov(0.7854)
        self.camera.enable(self.time_step)
        self.camera_pitch_sensor = self.getDevice("camera pitch sensor")
        self.camera_pitch_sensor.enable(self.time_step)
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
        self.camera_yaw_motor = self.getDevice("camera yaw")
        self.camera_yaw_motor.setPosition(0.0)
        self.camera_roll_motor = self.getDevice("camera roll")
        self.camera_roll_motor.setPosition(0.0)    # level


        for motor in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 20
        self.end_of_search = False

        # Connect to server to get waypoints
        self.server_ip = 'localhost'
        self.server_port = 5555
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_ip, self.server_port))
        self.world_details = self.get_world_details_from_server()
        self.waypoints = self.world_details['waypoints']
        self.world_description = self.world_details['world_description']
        self.world_name = self.world_details['world_name']
        print(f"Recieved {len(self.waypoints)} from waypoints from server")

    def get_world_details_from_server(self):
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
            world_details = json.loads(data.decode('utf-8'))
            print(f"[Waypoint Client] Received and parsed waypoints successfully. {world_details['waypoints']}")
            return world_details

        except Exception as e:
            print(f"[Waypoint Client] Error: {e}")
            sys.exit(1)


    def set_position(self, pos):
        self.current_pose = [float(p) for p in pos]  # Ensure all positions are floats

    def move_to_target(self, verbose_movement=False, verbose_target=False):
        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = [self.waypoints[0]['x'], self.waypoints[0]['y'], 0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
            self.target_index += 1

            if self.waypoints[self.target_index]['end']:
                self.end_of_search = True

            if self.target_index > len(self.waypoints) - 1 and not self.end_of_search:
                self.target_index = 0
            else:
                self.target_position[0:2] = [self.waypoints[self.target_index]['x'], self.waypoints[self.target_index]['y'], 0]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

        # This will be in ]-pi;pi]
        self.target_position[2] = np.arctan2(
            self.target_position[1] - self.current_pose[1], self.target_position[0] - self.current_pose[0])
        # This is now in ]-2pi;2pi[
        angle_left = self.target_position[2] - self.current_pose[5]
        # Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2 * np.pi) % (2 * np.pi)
        if (angle_left > np.pi):
            angle_left -= 2 * np.pi

        # Turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE * angle_left / (2 * np.pi)
        # non proportional and decreasing function
        pitch_disturbance = clamp(
            np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1)

        if verbose_movement:
            distance_left = np.sqrt(((self.target_position[0] - self.current_pose[0]) ** 2) + (
                (self.target_position[1] - self.current_pose[1]) ** 2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(
                angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def is_emergency(self, description: str) -> bool:
        prompt = f"""
            You are a search-and-rescue assistant in the following environment:{self.world_description}
             
            You're tasked to confirm if people are present in a location and if they are safe or injured. 
            Currently, it is being done in a 3D simulation so assume all 3D renderings are real. 
            Given the description below, provide a response only if a person is found.

            Description: "{description}"

            Explain the scene. Respond with "1. Person Found" if the description includes a person. If so, include in the response "2. Person Requires Assistance"
            if and only if the person may be injured or in a dangerous situation. If they are not injured or in a dangerous situation, state "2. Person does not require assistance". If the
            person does require assistance, include "3. Assistance needed" and the kind of assistance they would need from first responders.
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
    
    # def extract_json_from_text(self, text):
    #     """
    #     Extracts and returns a valid JSON object (as a dict) from a string containing additional content.
    #     """

    #     # print(text)
    #     try:
    #         # Use regex to find a JSON-like block in the text
    #         match = re.search(r'\{[\s\S]*\}', text)
    #         if match:
    #             json_str = match.group(0)
    #             # Convert to dict
    #             return json.loads(json_str)
    #         else:
    #             raise ValueError("No JSON object found in the text.")
    #     except json.JSONDecodeError as e:
    #         print("Invalid JSON format:", e)
    #     except Exception as e:
    #         print("Error:", e)
    #     return {'person_found': False, 'requires_assistance': False, 'assistance_instructions': False}

    def far_from_other_detections(self, new_detection, detections):
        if len(detections) == 0:
            return True

        for detection in detections:
            distance = math.sqrt((detection[0]-new_detection[0])**2+(detection[1]-new_detection[1])**2)
            if distance < 15:
                return False
            
        return True
    
    def euler_to_rotation_matrix(self, roll, pitch, yaw):
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)]
        ])

        Ry = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])

        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        return Rz @ Ry @ Rx


    def estimate_location(self, pixel_coord):
        x_drone, y_drone, z_drone = self.gps.getValues()
        drone_roll, drone_pitch, drone_yaw = self.imu.getRollPitchYaw()
        gimbal_pitch = self.camera_pitch_sensor.getValue()
        camera_pitch = drone_pitch - gimbal_pitch

        width = self.camera.getWidth()
        height = self.camera.getHeight()
        x_pixel, y_pixel = pixel_coord

        fov_rad = self.camera.getFov()
        f = width / (2 * np.tan(fov_rad/2))

        cx, cy = width / 2, height / 2
        K_inv = np.linalg.inv(np.array([
            [f, 0, cx],
            [0, f, cy],
            [0, 0, 1]
        ]))

        # Image to normalized camera coordinates
        pixel_vec = np.array([x_pixel, y_pixel, 1])
        cam_ray = K_inv @ pixel_vec
        cam_ray /= np.linalg.norm(cam_ray)  # Normalize direction

        # Convert to world coordinates
        R_drone_to_world = self.euler_to_rotation_matrix(drone_roll, camera_pitch, drone_yaw)
        world_ray = R_drone_to_world @ cam_ray

        # Ray origin is drone position
        origin = np.array([x_drone, y_drone, z_drone])

        # Solve for intersection with ground plane z = 0
        t = -origin[2] / world_ray[2]  # z = 0 -> solve for t
        
        # if t < 0:
        #     print(t)
            # return None  # Ray points upward or parallel to ground
        


        world_point = origin + t * world_ray
        # print(f'World Ray: {world_ray}')
        # print(f't: {t}')
        # print(f'Ground intersection: {world_point}')
        # print("Drone pitch:", np.degrees(drone_pitch))
        # print("Gimbal pitch sensor (degrees):", np.degrees(gimbal_pitch))
        # print("Total camera pitch (degrees):", np.degrees(drone_pitch + gimbal_pitch))
        # print("Pixel:", pixel_coord)
        # print("Cam ray:", cam_ray)
        # print("Drone pos:", (x_drone, y_drone, z_drone))
        # print("World ray:", world_ray)
        # print("Intersection:", world_point[:2])
        # print("Roll, Pitch, Yaw (degrees):", np.degrees(drone_roll), np.degrees(drone_pitch), np.degrees(drone_yaw))
        # print("Gimbal pitch (degrees):", np.degrees(gimbal_pitch))
        return world_point[:2]  # Return (x, y) only

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

        # make a list of coordinates of where people have been detected
        detections = []
        detection_id = 0
        detections_folder = '../../results/detections'
        if not os.path.exists(detections_folder):
            os.makedirs(detections_folder)

        while self.step(self.time_step) != -1:
            # current_time = self.getTime()

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acc, pitch_acc, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            telemetry_data = {
                "x_d": int(x_pos), "y_d": int(y_pos), "altitude": altitude,
                "est_x": False, "est_y": False,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "conf": 0.0, "person_found": False, "vlm_description": False,
                "requires_assistance": False, "assistance_instructions": '', 
                "world_name": self.world_name, "detection_id": False
            }

            height = self.camera.getHeight()
            width = self.camera.getWidth()

            # print(f'Height: {height}, Width: {width}')

            raw_image = self.camera.getImage()
            img_array = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4))
            bgr_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR) # YOLO
            rgb_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
            pil_image = Image.fromarray(rgb_image)
        

            # Run YOLOv8 to find people
            results = yolo_model(bgr_image, verbose=False)[0]
            person_detected = False

            for result in results.boxes.data:
                x1, y1, x2, y2, conf, cls = result.tolist()
                bounding_box_height = y2-y1

                if int(cls) == 0: # If person is detected to a 90% confidence
                    if altitude < self.target_altitude:
                        continue
                    # print(conf)
                    elif round(conf, 2) > 0.40:
                        # if the new conf is better than the currently best conf, set best_conf to new conf
                        if round(conf, 2) > round(best_conf, 2):
                            best_conf = conf
                        # else, there isn't a better conf and set person_detected is True
                        else:
                            person_detected = True
                            # est_x, est_y = self.estimate_person_location(telemetry_data)
                            cv2.rectangle(bgr_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                            cv2.putText(bgr_image, f"Person {conf:.2f}", (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0), 1)
                            telemetry_data['conf'] = best_conf
                            best_conf = 0
                            person_x = x1
                            person_y = y1

                            est_loc = self.estimate_location((person_x, person_y))
                            est_x = int(est_loc[0])
                            est_y = int(est_loc[1])

            if person_detected and (est_x, est_y) not in detections and self.far_from_other_detections(new_detection=(est_x, est_y), detections=detections):
                # print(f'Person Detected. {int(x_pos)},{int(y_pos)}')
                #print("YOLO has detected a person. Running Florence-2 to get image description")
                prompt = "Describe the image"
                inputs = VLM_processor(images=pil_image, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
                output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
                vlm_description = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]

                yolo_detection = f'{detection_id}_yolo_detection.jpg'
                yolo_detection_path = os.path.join(detections_folder, yolo_detection)

                cv2.imwrite(yolo_detection_path, bgr_image)
                #print("Person confirmed. Florence-2 Output:", generated_text)
                # response = {'person_found': False, 'requires_assistance': False, 'assistance_instructions': False}
                try:
                    # LLM Processing
                    LLM_response = self.is_emergency(vlm_description)
                    # response = self.extract_json_from_text(LLM_response)
                    #insert into client response
                    telemetry_data['vlm_description'] = vlm_description
                    telemetry_data['person_found'] = True if '1. Person Found' in LLM_response else False
                    telemetry_data['requires_assistance'] = True if '2. Person Requires Assistance' in LLM_response else False
                    telemetry_data['assistance_instructions'] = LLM_response
                except HfHubHTTPError as e:
                    if "402 Client Error" in str(e):
                        print('Max calls for free trier credits.')

                        LLM_response = 'Max calls for free trier credits.'
                        telemetry_data['vlm_description'] = vlm_description
                        telemetry_data['person_found'] = True
                        telemetry_data['requires_assistance'] = 'N/A'
                        telemetry_data['assistance_instructions'] = LLM_response
                        telemetry_data['detection_id'] = detection_id
                    else:
                        raise

                if telemetry_data['person_found']:
                    cv2.putText(bgr_image, f"Person found...", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)

                    telemetry_data['est_x'] = int(est_loc[0])
                    telemetry_data['est_y'] = int(est_loc[1])

                    detections.append((int(est_loc[0]), int(est_loc[1])))
                    detection_id += 1

                    telemetry_data['detection_id'] = detection_id


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
            
            if self.target_position == [0,0] and x_pos > -0.5 and x_pos < 0.5 and y_pos > -0.5 and y_pos < 0.5 and self.end_of_search:
                self.target_altitude = 0

            if self.target_altitude < 0.25 and self.end_of_search:
                self.front_left_motor.setVelocity(0)
                self.front_right_motor.setVelocity(0)
                self.rear_left_motor.setVelocity(0)
                self.rear_right_motor.setVelocity(0)

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
