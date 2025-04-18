import socket
import threading
import time
import json
from cryptography.fernet import Fernet
import sys
import struct
import cv2
import numpy as np
import math
import requests
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForCausalLM
from ultralytics import YOLO

sys.path.append("..")  # Allow imports from the parent directory
from key_manager import encryption_key  # Import the shared key


HOST = '127.0.0.1'
PORT = 5555
MAX_CONNECTIONS_PER_WINDOW = 10  # Threshold for jamming detection
WINDOW_DURATION = 5  # Time window in seconds

cipher = Fernet(encryption_key)

# Connection tracking
connection_timestamps = []
lock = threading.Lock()
server_running = True  # Flag to control the server loop

# Path to save the latest detection
SAVE_PATH = "latest_detection.jpg"

# Load YOLO model (pretrained for person detection)
model = YOLO("yolov8n.pt") 

# Set up Florence-2
device = "cuda:0" if torch.cuda.is_available() else "cpu"
torch_dtype = torch.float16 if torch.cuda.is_available() else torch.float32

VL_model = AutoModelForCausalLM.from_pretrained("microsoft/Florence-2-base", torch_dtype=torch_dtype, trust_remote_code=True).to(device)
processor = AutoProcessor.from_pretrained("microsoft/Florence-2-base", trust_remote_code=True)

prompt = "<OD>"



def detect_jamming():
    """Detects if too many connections occur in a short time window."""
    global connection_timestamps
    with lock:
        current_time = time.time()
        # Remove timestamps older than WINDOW_DURATION
        connection_timestamps = [t for t in connection_timestamps if current_time - t < WINDOW_DURATION]

        if len(connection_timestamps) >= MAX_CONNECTIONS_PER_WINDOW:
            print("🚨 WARNING: Possible jamming detected! Too many connections in a short time.")
            return True
    return False

def estimate_person_location(drone_loc_dict):
    x_d = drone_loc_dict['x_d']
    y_d = drone_loc_dict['y_d']
    altitude = drone_loc_dict['altitude']
    roll = drone_loc_dict['roll']
    pitch = drone_loc_dict['pitch']
    yaw = drone_loc_dict['yaw']

    #Convert_angles to radians
    yaw_rad = math.radians(yaw)
    pitch_rad = math.radians(pitch)

    if pitch >= 90:
        return round(x_d, 5), round(y_d, 5)
    
    d = altitude * math.tan(pitch_rad)

    x_p = x_d + d * math.cos(yaw_rad)
    y_p = y_d + d * math.cos(yaw_rad)

    return round(x_p, 5), round(y_p, 5)
    pass

def handle_client(conn, addr):
    """Handles communication with a connected client."""
    global connection_timestamps
    with lock:
        connection_timestamps.append(time.time())  # Log new connection

    # if detect_jamming():
    #     print(f"⚠️ Blocking connection from {addr} due to possible jamming.")
    #     conn.send(cipher.encrypt(b"Too many connections! Possible jamming detected."))
    #     conn.close()
    #     return

    print(f"🔗 New connection from {addr}")
    # try:
    #     with open(f"data_from_{addr}.txt", "x") as file:
    #         file.write(f"File created for {addr}\n")
    # except FileExistsError:
    #     print("File already exists.")
    #     open(f"data_from_{addr}.txt", "w").close()
    try:
        while True:
            # Receive the image size
            #data_size = struct.unpack("L", conn.recv(8))[0]
            #print(f"Expected image size: {data_size} bytes")
            data = conn.recv(16)
            json_length, img_length = struct.unpack("QQ", data)

            #JSON data
            json_data = conn.recv(json_length).decode()
            received_dict = json.loads(json_data)
            #print("Received Dictionary:", received_dict)

            # Receive the image data
            img_bytes = b""
            while len(img_bytes) < img_length:
                print(f"Receiving... {len(img_bytes)}/{img_length} bytes")
                packet = conn.recv(img_length - len(img_bytes))
                if not packet:
                    print("Received an empty packet. Connection might be closed.")
                    break
                img_bytes += packet
            print(f"Fully received image data: {len(img_bytes)} bytes")

            # Decode and display the image
            #frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            image_array = np.frombuffer(img_bytes, dtype=np.uint8)
            image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

            if image is None:
                print("❌ Error: Could not decode the image. Data might be corrupted.")
                return  # Exit if the image couldn't be decoded

            print("✅ Successfully decoded the image.")

            # with open("received_image.jpg", "wb") as f:
            #     f.write(data)
            # print("✅ Image saved as received_image.jpg. Try opening it manually.")

            # Run YOLO detection
            #results = model(image)[0]
            #person_detected = False  # Flag to track if a person is detected

            # Draw bounding boxes around detected people
            # for result in results.boxes.data:
            #     x1, y1, x2, y2, conf, cls = result.tolist()
            #     #print(f"x1: {x1}, y1: {y1}, x2: {x2}, y2: {y2}, conf: {conf}, cls: {cls}")
            #     if int(cls) == 0:  # Class 0 corresponds to 'person' in YOLO
            #         person_detected = True
            #         if conf > 0.7:
            #             #print("Drone Location:", received_dict["x_d"], received_dict["y_d"])
            #             #print("Estimated Person Location:", estimate_person_location(received_dict))
            #             est_x, est_y = estimate_person_location(received_dict)
            #             cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            #             cv2.putText(image, f"Person {conf:.2f}, Estimated Location: {est_x}, {est_y}", (int(x1), int(y1) - 10),
            #                         cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)

            # Run Florence-2
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(image_rgb)

            inputs = processor(images=pil_image, text=prompt, return_tensors="pt").to(device, torch_dtype)
            generated_outputs = VL_model.generate(**inputs, max_new_tokens=256)

            generated_text = processor.batch_decode(generated_outputs, skip_special_tokens=True)[0]
            print("Florence-2 Output:", generated_text)

            person_detected = "person" in generated_text.lower()

            cv2.imwrite(SAVE_PATH, image)

            # Save the latest detection ONLY if a person is detected
            if person_detected:
                est_x, est_y = estimate_person_location(received_dict)
                cv2.putText(image, f"Florence-2L Person Detected, Estimated Location: {est_x}, {est_y}", (10,30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                cv2.imwrite(SAVE_PATH, image)
                print(f"📸 Florence-2 detected a person! Image saved: {SAVE_PATH}")

                # for YOLO
                # cv2.imwrite(SAVE_PATH, image)
                # print(f"📸 Person detected! Image saved: {SAVE_PATH}")

            # Display the image with detections
            #cv2.imshow("Drone Camera - YOLO Person Detection", image)

            #if cv2.waitKey(1) & 0xFF == ord('q'):
                #break

    except ConnectionResetError:
        print(f"⚠️ Connection lost with {addr}")
    finally:
        conn.close()
        print(f"🔌 Connection closed: {addr}")

def accept_clients(server_socket):
    """Accepts client connections in a loop."""
    print('accepting client')
    global server_running
    while server_running:
        try:
            server_socket.settimeout(1.0)  # Avoid blocking indefinitely
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
        except socket.timeout:
            continue  # Keep checking if the server is running
        except OSError:
            break  # Server socket closed

# Start server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()
print(f"🚀 Server listening on {HOST}:{PORT}")

# Start accepting clients in a separate thread
accept_thread = threading.Thread(target=accept_clients, args=(server_socket,), daemon=True)
accept_thread.start()

# Main loop for server shutdown
while True:
    command = input("Enter 'x' to exit server: ").strip().lower()
    if command == 'x':
        print("🛑 Shutting down server...")
        server_running = False  # Signal the accept_clients thread to stop
        server_socket.close()  # Close the server socket
        accept_thread.join()  # Wait for thread to exit
        cv2.destroyAllWindows()
        break
