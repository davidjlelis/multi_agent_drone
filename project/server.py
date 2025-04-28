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
import pickle

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

def handle_client(conn, addr, waypoints):
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
        # Send waypoints as JSON
        waypoints_json = pickle.dumps(waypoints)
        print(f"Sent {len(waypoints_json)} bytes of pickled data.")

        # First send the size of the data
        conn.sendall(len(waypoints_json).to_bytes(8, byteorder='big'))
        conn.sendall(waypoints_json)
        print(f"✅ Sent waypoints to {addr}")
    except ConnectionResetError:
        print(f"⚠️ Connection lost with {addr}")
    finally:
        conn.close()
        print(f"🔌 Connection closed: {addr}")

def accept_clients(server_socket, waypoints):
    """Accepts client connections in a loop."""
    print('accepting client')
    global server_running
    while server_running:
        try:
            server_socket.settimeout(1.0)  # Avoid blocking indefinitely
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr, waypoints), daemon=True).start()
        except socket.timeout:
            continue  # Keep checking if the server is running
        except OSError:
            break  # Server socket closed

def generate_waypoints(x_dim, y_dim, step=1):
    waypoints = []
    x_start = int(-x_dim/2)
    x_end = int(x_dim/2)
    y_start = int(-y_dim/2)
    y_end = int(y_dim/2)

    for x in range(x_start, x_end+1, step):
        for y in range(y_start, y_end+1, step):
            waypoints.append({'x': x, 'y' : y})
    
    return waypoints
    

if __name__ == "__main__":
    # 
    x_dim = int(input("Enter X dimension of explorable environment (in meters): "))-20
    y_dim = int(input("Enter Y dimension of explorable environment (in meters): "))-20
    step = int(input("Enter step size (in meters) (default 1): "))

    waypoints = generate_waypoints(x_dim=x_dim, y_dim=y_dim, step=step)
    print(f"Waypoints created {x_dim}m x {y_dim} m at every {step} meters")

    # Start server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print(f"🚀 Server listening on {HOST}:{PORT}")

    # Start accepting clients in a separate thread
    accept_thread = threading.Thread(target=accept_clients, args=(server_socket, waypoints,), daemon=True)
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
