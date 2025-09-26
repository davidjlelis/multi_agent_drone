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
from transformers import AutoProcessor, AutoModelForCausalLM, AutoTokenizer, Pipeline, Blip2Processor, Blip2ForConditionalGeneration
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
    target_precision = 2

    def __init__(self):
        super().__init__()

        self.time_step = int(self.getBasicTimeStep())

        #self.name = self.getName()
        #self.client = self.setup_client_connection(name=self.name)

        self.camera = self.getDevice("camera")
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
        self.camera_pitch_motor.setPosition(0.5)        
        self.camera_yaw_motor = self.getDevice("camera yaw")
        self.camera_yaw_motor.setPosition(1.57)
        self.camera_roll_motor = self.getDevice("camera roll")
        self.camera_roll_motor.setPosition(0.0)    # level


        for motor in [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 10
        self.orbiting = False
        self.start_scanning = False
        self.orbit_loops = 0
        self.total_orbit_points = 0
        self.return_home = False
        self.orbit_started = False
        self.yolo_interval = 1
        self.last_yolo_time = 0.0


        # Connect to server to get waypoints
        self.server_ip = 'localhost'
        self.server_port = 5555
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.server_ip, self.server_port))
        self.world_details = self.get_world_details_from_server()
        self.waypoints = self.world_details['waypoints']
        self.world_description = self.world_details['world_description']
        self.world_name = self.world_details['world_name']
        self.goal_position = self.world_details['goal_position']
        self.message = self.world_details['message']
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

    def move_to_target(self, waypoints, verbose_movement=True, verbose_target=True):
            if self.target_position[0:2] == [0, 0]:  # Initialization
                self.target_position[0:2] = waypoints[0]
                if verbose_target:
                    print("First target: ", self.target_position[0:2])

            # if the robot is at the position with a precision of target_precision
            if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):
                # print(f'At target position: {self.target_position}')
                self.target_index += 1
                if self.orbiting and self.target_index >= self.total_orbit_points:
                    # only count a completed orbit if we've actually started orbiting (left the goal)
                    if self.orbit_started:
                        print('One orbit complete')
                        self.orbit_loops += 1
                    # reset index so we continue following orbit waypoints if needed
                    self.target_index = 0

                if self.target_index > len(waypoints) - 1:
                    self.target_index = 0
                self.target_position[0:2] = waypoints[self.target_index]
                if verbose_target:
                    print("Target reached! New target: ",
                        self.target_position[0:2])
                # print(f'Moving to new target position: {self.target_position}')

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
            You are a search-and-rescue assistant for first responders in the following environment: {self.world_description}

            The description below is of an image and the message is from a caller requesting first responder assistance. 
            Combining the context of the image and the description, respond with a full description of the scene.

            Description: "{description}"
            Message From Caller: "{self.message}"

            Respond with "1. Person Found" if and only if the description includes a person. If no preson is found, state "1. No Person Found"
            If so, include in the response "2. Person Requires Immediate Assistance" if and only if the person may be injured or in a dangerous situation based on the Description and Message From Caller. If they are not injured or in a dangerous situation, state "2. Person does not require assistance". 
            If the person does require assistance, include "3. Assistance needed" and the kind of assistance they would need from first responders based on the Description and Message From Caller.
        """
        try:
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
        except HfHubHTTPError as e:
            print(f'Error - Hugging Face API request failed: {e}')
            return "API Error"
        except Exception as e:
            print(f'Unexcepted failure in LLM call: {e}')
            return "Unknwon error"
        
    def orbit_path(self, radius):
        x0, y0 = (self.goal_position[0], self.goal_position[1])
        points = []
        for i in range(8):
            theta = math.radians(45 * i)  # Convert degrees to radians
            x = x0 + radius * math.cos(theta)
            y = y0 + radius * math.sin(theta)
            points.append([x, y, 0])
        return points

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

        start_up_complete = False

        height = self.camera.getHeight()
        width = self.camera.getWidth()

        orbit_radius = 5.0
        start_orbit_threshold = orbit_radius * 0.8

        prev_vlm_desc = []

        iterations = 0

        while self.step(self.time_step) != -1:
            max_conf = 0
            best_result = None
            best_yolo_image = None
            image_rotate_angle = None
            person_detected = False

            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            telemetry_data = {
                "x_d": int(x_pos), "y_d": int(y_pos), "altitude": altitude,
                "est_x": False, "est_y": False,
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "conf": 0.0, "person_found": False, "vlm_description": False,
                "requires_assistance": False, "assistance_instructions": '', 
                "world_name": self.world_name, "detection_id": False, "message": self.message
            }
            raw_image = self.camera.getImage()
            img_array = np.frombuffer(raw_image, dtype=np.uint8).reshape((height, width, 4))
            bgr_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR) # YOLO
            rgb_image = cv2.cvtColor(img_array, cv2.COLOR_BGRA2RGB)
            pil_image = Image.fromarray(rgb_image)

            if not start_up_complete and altitude >= self.target_altitude:
                start_up_complete = True
            
            if abs(x_pos - self.goal_position[0]) <= self.target_precision and abs(y_pos - self.goal_position[1]) <= self.target_precision and not self.orbiting:
                print('At goal. Begin Orbitting path')
                self.waypoints = self.orbit_path(radius=orbit_radius)
                self.total_orbit_points = len(self.waypoints)
                self.orbiting = True
                self.orbit_started = False            # haven't actually started moving on orbit yet
                self.start_scanning = False           # scanning only after orbit movement starts
                self.target_index = 0                 # start at the first orbit waypoint
                # force the controller to aim at the first orbit waypoint immediately:
                self.target_position[0:2] = self.waypoints[self.target_index]


            if self.orbiting and self.target_altitude > 5.1:
                self.target_altitude -= 0.1

            if self.orbiting and not self.start_scanning:
                dist_from_goal = math.sqrt((x_pos - self.goal_position[0])**2 + (y_pos-self.goal_position[1])**2)
                if dist_from_goal >= start_orbit_threshold:
                    print("Drone at orbit path. begin scanning...")
                    self.start_scanning = True
                    self.orbit_started = True   # <-- now we truly started orbiting


            # Run YOLOv8 to find people
            # rotate image to thoroughly look through people
            if start_up_complete and not self.return_home and self.orbiting and self.start_scanning:
                current_time = self.getTime()
                if current_time - self.last_yolo_time > self.yolo_interval:
                    self.last_yolo_time = current_time
                    yolo_image_rotations = [bgr_image
                                            , cv2.rotate(bgr_image, cv2.ROTATE_90_CLOCKWISE)
                                            , cv2.rotate(bgr_image, cv2.ROTATE_180)
                                            , cv2.rotate(bgr_image, cv2.ROTATE_90_COUNTERCLOCKWISE)]
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
                if cls in (0, 2):
                    person_detected = True
                    telemetry_data['conf'] = conf

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
                    # set OG image
                    iterations += 1
                    
                    # Florence2 VLM
                    prompt = "<MORE_DETAILED_CAPTION>"
                    inputs = VLM_processor(images=pil_image, text=prompt, return_tensors="pt").to(VL_model.device, torch_dtype)
                    output_tokens = VL_model.generate(**inputs, max_new_tokens=50)
                    vlm_description = VLM_processor.batch_decode(output_tokens, skip_special_tokens=True)[0]

                    detection_id += 1

                    vlm_description = vlm_description.replace('3D rendering', '')

                    if vlm_description not in prev_vlm_desc:
                        print(f'VLM description: {vlm_description}')

                        yolo_detection = f'{detection_id}_yolo_detection.jpg'
                        yolo_detection_path = os.path.join(detections_folder, yolo_detection)

                        try:
                            prev_vlm_desc.append(vlm_description)
                            # LLM Processing
                            LLM_response = self.is_emergency(vlm_description)

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

                        if telemetry_data['person_found'] or person_detected:
                            cv2.imwrite(yolo_detection_path, best_yolo_image)
                            cv2.putText(bgr_image, f"Person found...", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 1)
                            telemetry_data['detection_id'] = detection_id


                            # Encrypt and send telemetry data for mapping
                            telemetry_json = json.dumps(telemetry_data)
                            encrypted_telemetry = cipher.encrypt(telemetry_json.encode('utf-8'))
                            try:
                                client.sendall(len(encrypted_telemetry).to_bytes(8, byteorder='big'))
                                client.sendall(encrypted_telemetry)
                                # print(f'Data sent to server: {telemetry_json}')
                            except BrokenPipeError:
                                print("⚠️ Connection lost while sending data. Skipping send.")
                                break  # Exit the run() loop if server is gone

                    best_conf = 0.0

            if self.orbiting and self.orbit_loops >= 1:
                self.start_scanning = False
                self.orbiting = False
                self.return_home = True
                self.target_altitude = 20

            if self.return_home:
                self.waypoints = [[0,0]]
                if abs(x_pos-self.target_position[0]) < self.target_precision and abs(y_pos-self.target_position[1]) < self.target_precision:
                    self.target_altitude = self.target_altitude - 0.05
                else:
                    self.target_altitude = 20

                if self.target_altitude < 0.10:
                    self.front_left_motor.setVelocity(0)
                    self.front_right_motor.setVelocity(0)
                    self.rear_left_motor.setVelocity(0)
                    self.rear_right_motor.setVelocity(0)

                    print('Survey Completed. Sending Completion Signal')

                    try:
                        msg = cipher.encrypt(b"MISSION_COMPLETE")
                        client.sendall(len(msg).to_bytes(8, byteorder='big'))
                        client.sendall(msg)
                        quit()
                    except:
                        print("⚠️ Could not notify server of completion.")

                    break  # Exit run loop

            if altitude > self.target_altitude - 1:
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(self.waypoints, verbose_movement=False, verbose_target=False)
                    t1 = self.getTime()
            
            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


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