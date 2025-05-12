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

# def detect_jamming():
#     """Detects if too many connections occur in a short time window."""
#     global connection_timestamps
#     with lock:
#         current_time = time.time()
#         # Remove timestamps older than WINDOW_DURATION
#         connection_timestamps = [t for t in connection_timestamps if current_time - t < WINDOW_DURATION]

#         if len(connection_timestamps) >= MAX_CONNECTIONS_PER_WINDOW:
#             print("🚨 WARNING: Possible jamming detected! Too many connections in a short time.")
#             return True
#     return False

class Node:
    def __init__(self, x, y, size, node_type = 0) -> None:
        self.x = x                      # x coordinate
        self.y = y                      # y coordinate
        self.size = size                # size
        self.node_type = node_type      # open space (0), obstacle (1), start (2), goal (3)
        self.state = 0
        self.value = 1
        pass

    def draw(self, image, flipped_y):
        # color = (255, 255, 255) if self.node_type == 0 else (0, 0, 0)
        color = (0,0,0)
        if self.node_type == 0:
            color = (255, 255, 255)
        elif self.node_type == 2:
            color = (0, 255, 0)
        elif self.node_type == 3:
            color = (0, 0, 255)


        top_left = (self.x * self.size, flipped_y * self.size)
        bottom_right = ((self.x + 1) * self.size, (flipped_y + 1) * self.size)
        cv2.rectangle(image, top_left, bottom_right, color, -1)  # Fill
        cv2.rectangle(image, top_left, bottom_right, (200, 200, 200), 1)

class Map:
    def __init__(self, width, height, cell_size, grid_data):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.nodes = [
            [Node(x, y, cell_size, grid_data[y][x]) for x in range(width)]
            for y in range(height)
        ]
        self.start_node = None
        self.goal_nodes = []

    def setObstacles(self, obstacles):
        # for row in self.nodes:
        #     for node in row:
        #         print(node.x, ',', node.y, ',', node.node_type)
        for obstacle in obstacles:
            for row in self.nodes:
                for node in row:
                    if node.x == 50 and node.y == 50:
                        node.node_type = 2
                        self.start_node = node
                    elif node.x >= obstacle['lower_x']+50 and node.x <= obstacle['upper_x']+50 and node.y >= obstacle['lower_y']+50 and node.y <= obstacle['upper_y']+50:
                        node.node_type = 1
                    # elif node.x == 1 and node.y == 1: # checking to see where the grid begins
                    #     node.node_type = 1

    def setGoal(self, goal):
        for row in self.nodes:
            for node in row:
                if node.x == goal['x']+50 and node.y == goal['y']+50:
                    self.goal_nodes.append(node)
                    node.node_type = 3

    def getMap(self):
        width = self.width * self.cell_size
        height = self.height * self.cell_size
        img = np.ones((height, width, 3), dtype=np.uint8) * 255
        
        
        for row in self.nodes:
            for node in row:
                flipped_y = self.height - 1 - node.y
                node.draw(img, flipped_y)

        return img
    
    def pathfinding_Astar(self):
        pass

    def pathfinding_RTT(self):
        pass

def estimate_location(drone_loc_dict):
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
        waypoints_json = json.dumps(waypoints)
        print(f"Sent {len(waypoints_json)} bytes of data.")
        waypoints_bytes = waypoints_json.encode('utf-8')
        # First send the size of the data
        conn.sendall(len(waypoints_bytes).to_bytes(8, byteorder='big'))
        conn.sendall(waypoints_bytes)
        print(f"✅ Sent waypoints to {addr}")

        try:
            print("Waiting for client ready signal")
            conn.settimeout(10)
            ready_signal = conn.recv(8)

            if not ready_signal:
                print(f'No ready signal from {addr}. Closing connection')
                return
            print(f"Client {addr} is ready. Listening for telemetry...")
            conn.settimeout(None) # Reset timeout
        except:
            print(f"⚠️ Timeout waiting for client ready signal from {addr}. Closing connection.")
            return

        # Listen for detections
        while True:
            data_size_bytes = conn.recv(8)
            if not data_size_bytes:
                break
            data_size = int.from_bytes(data_size_bytes, byteorder='big')
            data = b''

            while len(data) < data_size:
                packet = conn.recv(data_size - len(data))
                if not packet:
                    break
                data += packet

            if not data:
                break

            decrypted_data = cipher.decrypt(data)
            detection = json.loads(decrypted_data.decode('utf-8'))
            print(detection)
            # Assume detection includes drone state
            x_d, y_d = detection['x_d'], detection['y_d']
            x_p, y_p = estimate_location(detection)
            #print(f'Person found nearby {x_p},{y_p}. Drone location at {x_d}, {y_d}')
            #draw_detection_on_map(x=x_p, y=y_p)

    except (ConnectionResetError, BrokenPipeError) as e:
        print(f"⚠️ Connection lost with {addr}: {e}")
    except Exception as e:
        print(f"⚠️ Connection lost with {addr}: {e}")
    finally:
        pass
        #conn.close()
        #print(f"🔌 Connection closed: {addr}")

def accept_clients(server_socket, waypoints):
    """Accepts client connections in a loop."""
    print('accepting client\n')
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
    # Get environment dimension
    x_dim = int(input("Enter X dimension of explorable environment (in meters): "))
    y_dim = int(input("Enter Y dimension of explorable environment (in meters): "))
    step = int(input("Enter step size (in meters) (default 1): "))

    x_path = x_dim-20
    y_path = y_dim-20

    waypoints = generate_waypoints(x_dim=x_path, y_dim=x_path, step=step)
    print(f"Waypoints created {x_dim} m x {y_dim} m at every {step} meters")

    # Create blank map (visual)
    window_name = "Environment Map"

    grid_data = []

    for i in range(y_dim):
        grid_data.append([])
        for j in range(x_dim):
            grid_data[i].append(0)

    # Create and display the map
    map_grid = Map(width=x_dim, height=y_dim, cell_size=5, grid_data=grid_data)
    obstacles = [
        {
            'name': 'obstacle_1'
            , 'lower_x': 28
            , 'upper_x': 44
            , 'lower_y': -8
            , 'upper_y': 9
        }, 
        {
            'name': 'obstacle_2'
            , 'lower_x': -27
            , 'upper_x': -5
            , 'lower_y': 16
            , 'upper_y': 38
        }, 
        {
            'name': 'obstacle_3'
            , 'lower_x': 23
            , 'upper_x': 47
            , 'lower_y': 22
            , 'upper_y': 38
        }, 
        {
            'name': 'obstacle_4'
            , 'lower_x': 2
            , 'upper_x': 13
            , 'lower_y': 40
            , 'upper_y': 50
        }
    ]
    map_grid.setObstacles(obstacles)

    goal = {'x': -22, 'y': 40}
    map_grid.setGoal(goal)

    # Get inital Grid/Environment/Map State
    map_img = map_grid.getMap()

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
        #command = input("\nEnter 'x' to exit server: ").strip().lower()
        cv2.imshow(window_name, map_img)
        print('Server and Environment Map has opened. Press ESC to close')
        key = cv2.waitKey(0) & 0XFF

        #if command == 'x':
        if key == 27:
            print("🛑 Shutting down server...")
            server_running = False  # Signal the accept_clients thread to stop
            server_socket.close()  # Close the server socket
            accept_thread.join()  # Wait for thread to exit
            cv2.destroyAllWindows()
            break
