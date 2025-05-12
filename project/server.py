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
        self.node_type = node_type      # open space (0), obstacle (1)
        pass

    def draw(self, image):
        color = (255, 255, 255) if self.node_type == 0 else (0, 0, 0)

        top_left = (self.x * self.size, self.y * self.size)
        bottom_right = ((self.x + 1) * self.size, (self.y + 1) * self.size)
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

    def setObstacles(self, obstacles):
        # for row in self.nodes:
        #     for node in row:
        #         print(node.x, ',', node.y, ',', node.node_type)
        for obstacle in obstacles:
            for row in self.nodes:
                for node in row:
                    if node.x >= obstacle['lower_x']+50 and node.x <= obstacle['upper_x']+50 and node.y >= obstacle['lower_y']+50 and node.y <= obstacle['upper_y']+50:
                        node.node_type = 1



    def draw(self):
        img = np.ones((self.height * self.cell_size, self.width * self.cell_size, 3), dtype=np.uint8) * 255
        for row in self.nodes:
            for node in row:
                node.draw(img)
        cv2.imshow("Grid Map", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

# class Node:
#     def __init__(self, x_loc, y_loc, node_type) -> None:
#         self.x_loc = x_loc          # x coordinate
#         self.y_loc = y_loc          # y coordinate
#         self.node_type = node_type  # node type (open space [0]/obstacle [1]/path [2])
#         pass

#     def printLoc(self):
#         print(self.x_loc,',', self.y_loc, self.node_type)
    
#     def getNode(self):
#         return self

# class Map:
#     def __init__(self, x_dim, y_dim) -> None:
#         self.x_dim = x_dim
#         self.y_dim = y_dim
#         self.grid = []
#         for i in range(y_dim):
#             self.grid.append([])
#             for j in range(x_dim):
#                 #print('creating Node', j, i)
#                 self.grid[i].append(Node(j, i, 0))
#         pass

#     def printMap(self):
#         for i in range(len(self.grid)):
#             # print(self.grid[i])
#             for j in range(len(self.grid[i])):
#                 self.grid[i][j].printLoc()

#     def getMap(self):
#         return self.grid
    
#     def setObstacles(self, obstacles):
#         for obstacle in obstacles:
#             print(obstacle)
#             # translate to (0 to 100, 0 to 100) coordinates vs (-50 to 50, -50 to 50)
#             obstacle['lower_x'] = obstacle['lower_x'] + 50
#             obstacle['upper_x'] = obstacle['upper_x'] + 50
#             obstacle['lower_y'] = obstacle['lower_y'] + 50
#             obstacle['upper_y'] = obstacle['upper_y'] + 50

#         print('Updates Coordinates')
#         for obstacle in obstacles:
#             for i in range(len(self.grid)):
#                 for j in range(len(self.grid[i])):
#                     if self.grid[i][j].x_loc >= obstacle['lower_x'] and self.grid[i][j].x_loc <= obstacle['upper_x'] and self.grid[i][j].y_loc >= obstacle['lower_y'] and self.grid[i][j].y_loc <= obstacle['upper_y']:
#                         self.grid[i][j].node_type = 1
#                         # print(self.grid[i][j].x_loc, self.grid[i][j].y_loc, self.grid[i][j].node_type)
#         pass

#     def drawMap(self):
#         pass

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

# def draw_detection_on_map(x, y):
#     global map_img, x_offset, y_offset

#     x_pix = int((x + x_offset) * PIXELS_PER_METER)
#     y_pix = int((y_offset - y) * PIXELS_PER_METER)  # Invert y-axis for image coordinates

#     if 0 <= x_pix < map_img.shape[1] and 0 <= y_pix < map_img.shape[0]:
#         cv2.circle(map_img, (x_pix, y_pix), 5, (0, 0, 255), -1)  # Red dot

def create_blank_map(x_dim, y_dim, cell_size=5, line_color=(0,0,0), thickness=1):
    #global map_img, x_offset, y_offset
    
    # Size of grid/image
    width = (x_dim * cell_size)
    height = (y_dim * cell_size)

    # create white background
    map_img = np.ones((height, width, 3), dtype=np.uint8) * 255

    # draw vertical lines
    for i in range(x_dim+1):
        cv2.line(map_img, (i*cell_size, 0), (i*cell_size, height), line_color, thickness)

    for j in range(y_dim+1):
        cv2.line(map_img, (0, j*cell_size), (width, j*cell_size), line_color, thickness)
    return map_img
    # map_img = np.ones((height, width, 3), dtype=np.uint8) * 255
    # x_offset = x_dim // 2
    # y_offset = y_dim // 2

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
    #map_img = create_blank_map(x_dim=x_dim, y_dim=y_dim)
    #window_name = "Environment Map"

    # Create blank map (array)
    # map = Map(x_dim=x_dim,y_dim=y_dim)

    # grid_data = [
    #     [0, 0, 1, 0, 0],
    #     [0, 1, 1, 0, 0],
    #     [0, 0, 0, 0, 1],
    #     [1, 0, 0, 1, 0],
    # ]

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
    map_grid.draw()

    # map.setObstacles(obstacles=obstacles)
    # map.printMap()

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
        #cv2.imshow(window_name, map_img)
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
