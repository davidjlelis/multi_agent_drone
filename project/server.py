import socket
import threading
import time
import json
from cryptography.fernet import Fernet
import sys
import numpy as np
import math
import matplotlib.pyplot as plt
import random
from shapely.geometry import Polygon, Point, LineString
from queue import Queue, Empty
import os

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
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.parent = None
        self.cost = 0

class Obstacle:
    def __init__(self, vertices):
        self.polygon = Polygon(vertices)

    def collides(self, x, y):
        return self.polygon.contains(Point(x, y))

    def line_intersects(self, x1, y1, x2, y2):
        return self.polygon.intersects(LineString([(x1, y1), (x2, y2)]))

    def draw(self, ax):
        x, y = self.polygon.exterior.xy
        ax.fill(x, y, color='black', alpha=0.4)

class RRTStar:
    def __init__(self, start, goal, map_size, obstacles=None,
                 step_size=5, goal_sample_rate=0.1,
                 search_radius=15, max_iter=500):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.map_size = map_size
        self.obstacles = obstacles or []
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.max_iter = max_iter
        self.nodes = [self.start]

    # def __init__(self, start, goal=None, map_size=(100, 100), obstacles=None,
    #              step_size=5, goal_sample_rate=0.1,
    #              search_radius=15, max_iter=500):
    #     self.start = Node(*start)
    #     self.goals = []
    #     if goal:
    #         self.goals.append(Node(*goal))
    #     self.reached_goals = []
    #     self.map_size = map_size
    #     self.obstacles = obstacles or []
    #     self.step_size = step_size
    #     self.goal_sample_rate = goal_sample_rate
    #     self.search_radius = search_radius
    #     self.max_iter = max_iter
    #     self.nodes = [self.start]

    def distance(self, n1, n2):
        return np.hypot(n1.x - n2.x, n1.y - n2.y)

    # def sample(self):
    #     if random.random() < self.goal_sample_rate:
    #         return self.goal
    #     return Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

    # def sample(self):
    #     if random.random() < self.goal_sample_rate:
    #         return random.choice(self.goals)  # pick a random goal
    #     return Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal
        x = random.uniform(-self.map_size[0] / 2, self.map_size[0] / 2)
        y = random.uniform(-self.map_size[1] / 2, self.map_size[1] / 2)
        return Node(x, y)

    def nearest(self, random_node):
        return min(self.nodes, key=lambda node: self.distance(node, random_node))

    def steer(self, from_node, to_node):
        dist = self.distance(from_node, to_node)
        if dist < self.step_size:
            return to_node
        theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new_x = from_node.x + self.step_size * np.cos(theta)
        new_y = from_node.y + self.step_size * np.sin(theta)
        new_node = Node(new_x, new_y)
        return new_node

    # def is_collision_free(self, node):
    #     for obs in self.obstacles:
    #         if obs.collides(node.x, node.y):
    #             return False
    #     return 0 <= node.x <= self.map_size[0] and 0 <= node.y <= self.map_size[1]

    def is_collision_free(self, node):
        for obs in self.obstacles:
            if obs.collides(node.x, node.y):
                return False
        half_width, half_height = self.map_size[0] / 2, self.map_size[1] / 2
        return (-half_width <= node.x <= half_width) and (-half_height <= node.y <= half_height)


    def check_line_collision(self, n1, n2):
        for obs in self.obstacles:
            if obs.line_intersects(n1.x, n1.y, n2.x, n2.y):
                return True
        return False

    def get_nearby_nodes(self, new_node):
        return [node for node in self.nodes if self.distance(node, new_node) <= self.search_radius]

    def add_goal(self, goal_coord):
        self.goals.append(Node(*goal_coord))

    def choose_parent(self, new_node, nearby_nodes):
        min_cost = float('inf')
        best_node = None
        for node in nearby_nodes:
            if not self.check_line_collision(node, new_node):
                cost = node.cost + self.distance(node, new_node)
                if cost < min_cost:
                    min_cost = cost
                    best_node = node
        if best_node:
            new_node.parent = best_node
            new_node.cost = min_cost

    def rewire(self, new_node, nearby_nodes):
        for node in nearby_nodes:
            if not self.check_line_collision(new_node, node):
                new_cost = new_node.cost + self.distance(new_node, node)
                if new_cost < node.cost:
                    node.parent = new_node
                    node.cost = new_cost

    def is_goal_reached(self, node):
        return self.distance(node, self.goal) < self.step_size and not self.check_line_collision(node, self.goal)

    def extract_path(self, goal):
        path = []
        node = goal
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return path[::-1]

    def plan(self, ax, pause_time=0.01):
        for _ in range(self.max_iter):
            rnd = self.sample()
            nearest_node = self.nearest(rnd)
            new_node = self.steer(nearest_node, rnd)

            if not self.is_collision_free(new_node):
                continue

            if self.check_line_collision(nearest_node, new_node):
                continue  # ensure edge is also collision-free

            nearby = self.get_nearby_nodes(new_node)
            self.choose_parent(new_node, nearby)

            if new_node.parent and not self.check_line_collision(new_node.parent, new_node):
                self.nodes.append(new_node)
                self.rewire(new_node, nearby)

                ax.plot([new_node.x, new_node.parent.x], [new_node.y, new_node.parent.y], "-g", linewidth=0.5)
                plt.pause(pause_time)

                if self.is_goal_reached(new_node):
                    self.goal.parent = new_node
                    self.goal.cost = new_node.cost + self.distance(new_node, self.goal)
                    self.nodes.append(self.goal)
                    ax.plot([self.goal.x, new_node.x], [self.goal.y, new_node.y], "-g", linewidth=0.5)
                    plt.pause(pause_time)
                    break

        return self.extract_path(self.goal)

    # def plan(self, ax=None, pause_time=0.01):
    #     goal = self.goals[-1]  # most recently added goal

    #     for i in range(self.max_iter):
    #         rnd = self.sample()
    #         nearest_node = self.nearest(rnd)
    #         new_node = self.steer(nearest_node, rnd)

    #         if not self.is_collision_free(new_node):
    #             continue
    #         if self.check_line_collision(nearest_node, new_node):
    #             continue

    #         nearby = self.get_nearby_nodes(new_node)
    #         self.choose_parent(new_node, nearby)

    #         if new_node.parent and not self.check_line_collision(new_node.parent, new_node):
    #             self.nodes.append(new_node)
    #             self.rewire(new_node, nearby)

    #             if ax and i % 10 == 0:
    #                 ax.plot([new_node.x, new_node.parent.x], [new_node.y, new_node.parent.y], "-g", linewidth=0.5)
    #                 plt.pause(pause_time)

    #             if self.distance(new_node, goal) < self.step_size and not self.check_line_collision(new_node, goal):
    #                 goal.parent = new_node
    #                 goal.cost = new_node.cost + self.distance(new_node, goal)
    #                 self.nodes.append(goal)

    #                 if ax:
    #                     ax.plot([goal.x, new_node.x], [goal.y, new_node.y], "-g", linewidth=0.5)
    #                     path = self.extract_path(goal)
    #                     px, py = zip(*path)
    #                     ax.plot(px, py, "-r", linewidth=2, label=f"Path to ({int(goal.x)}, {int(goal.y)})")
    #                     ax.plot(goal.x, goal.y, "or")
    #                     ax.legend()
    #                     ax.set_title(f"Planned to goal ({int(goal.x)}, {int(goal.y)})")
    #                     plt.pause(pause_time)
    #                 break

    #     return self.extract_path(goal)

    # def draw_final(self, ax, path=None):
    #     if path:
    #         px, py = zip(*path)
    #         ax.plot(px, py, "-r", linewidth=2, label="Final Path")

    #     ax.plot(self.start.x, self.start.y, "ob", label="Start")
    #     ax.plot(self.goal.x, self.goal.y, "or", label="Goal")
    #     ax.legend()
    #     ax.set_xlim(0, self.map_size[0])
    #     ax.set_ylim(0, self.map_size[1])
    #     ax.set_aspect('equal')
    #     ax.set_title("Safe RRT* Path Planning")

    def draw_final(self, ax, path=None):
        if path:
            px, py = zip(*path)
            ax.plot(px, py, "-r", linewidth=2, label="Final Path")

        ax.plot(self.start.x, self.start.y, "ob", label="Start")
        ax.plot(self.goal.x, self.goal.y, "or", label="Goal")
        ax.legend()
        
        # FIXED axis limits for centered map
        half_width, half_height = self.map_size[0] / 2, self.map_size[1] / 2
        ax.set_xlim(-half_width, half_width)
        ax.set_ylim(-half_height, half_height)

        ax.set_aspect('equal')
        ax.set_title("Safe RRT* Path Planning")



# def estimate_location(drone_loc_dict):
#     x_d, y_d = drone_loc_dict['x_d'], drone_loc_dict['y_d']
#     altitude, pitch, yaw = drone_loc_dict['altitude'], drone_loc_dict['pitch'], drone_loc_dict['yaw']

#     yaw_rad = math.radians(yaw)
#     pitch_rad = math.radians(pitch)

#     if pitch >= 90:
#         return round(x_d, 5), round(y_d, 5)

#     d = altitude * math.tan(pitch_rad)
#     x_p = x_d + d * math.cos(yaw_rad)
#     y_p = y_d + d * math.sin(yaw_rad)

#     return round(x_p, 5), round(y_p, 5)

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

        # Create json file
        output_path = "detections.json"
        with open(output_path, "w") as f:
            json.dump([], f)

        # Listen for detections
        # detection JSON format
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
            else:
                decrypted_data = cipher.decrypt(data)
                detection = json.loads(decrypted_data.decode('utf-8'))
                # print(detection)
                # Assume detection includes drone state
                # x_p, y_p = estimate_location(detection)
                print('Person found! Saving Coordinates and message.')            

                with open(output_path, "r") as f:
                    data = json.load(f)

                data.append(detection)

                with open(output_path, "w") as f:
                    json.dump(data, f, indent=4)

                x = detection['x_d']
                y = detection['y_d']

                # new_goal = {'name': f'goal_{detection['x_d']},{detection['y_d']}', 'x': detection['x_d'], 'y':detection['y_d']}
                new_goal = {'name': f'goal_{x},{y}', 'x': x, 'y':y}



                if new_goal in goals:
                    break
                else: 
                    goals.append(new_goal)
                    goal_queue.put(new_goal)

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

# def generate_waypoints(x_dim, y_dim, step_x, step_y):
#     waypoints = []
#     x_start = int(-x_dim/2)
#     x_end = int(x_dim/2)
#     y_start = int(-y_dim/2)
#     y_end = int(y_dim/2)

#     for x in range(int(x_start), int(x_end)+1, int(step_x)):
#         for y in range(int(y_start), int(y_end)+1, int(step_y)):
#             waypoints.append({'x': x, 'y' : y})
    
#     return waypoints  

def generate_waypoints(x_dim, y_dim, step_x=32.36, step_y=25):
    waypoints = []
    x_min = -x_dim / 2
    y_min = -y_dim / 2

    x_max = x_dim / 2
    y_max = y_dim / 2

    # print(x_min, x_max, y_min, y_max)

    num_rows = int((y_max-y_min) // step_y) + 1
    num_cols = int((x_max-x_min) // step_x) + 1

    for row in range(num_rows):
        y = y_min + row * step_y
        x_coords = [x_min + col * step_x for col in range(num_cols)]

        if row % 2 == 1:
            x_coords.reverse()

        for x in x_coords:
            print(x, y)
            if x <= x_dim and y <= y_dim:
                waypoints.append({'x': round(x, 2), 'y' : round(y, 2)})
            else:
                waypoints.append({'x': x_max, 'y': round(y,2)})
    
    for col in range(num_cols):
        x = x_min + col * step_x
        y_coords = [y_min + row * step_y for row in range(num_rows)]

        if col % 2 == 1:
            y_coords.reverse()
        
        for y in y_coords:
            if x <= x_dim and y <= y_dim:
                waypoints.append({'x': round(x, 2), 'y' : round(y, 2)})
            else:
                waypoints.append({'x': round(x, 2), 'y': y_max})

    return waypoints  

def on_key(event):
    global  server_running
    if event.key == 'escape':
        print('Closing Server...')
        server_running = False

if __name__ == "__main__":
    # Global Variables
    global fig, ax, goals, paths, colors, goal_queue

    # Get environment dimension
    x_dim = int(input("Enter X dimension of explorable environment (in meters): "))
    y_dim = int(input("Enter Y dimension of explorable environment (in meters): "))

    step_x = 20
    step_y = 20

    # waypoints = generate_waypoints(x_dim=x_dim-10, y_dim=y_dim-10, step_x=step_x, step_y=step_y )
    waypoints = [ {'x': -40, 'y': -40},
                    {'x': 40, 'y': -40},
                    {'x': 40, 'y': -20},
                    {'x': -40, 'y': -20},
                    {'x': -40, 'y': 0},
                    {'x': 40, 'y': 0},
                    {'x': 40, 'y': 20},
                    {'x': -40, 'y': 20},
                    {'x': -40, 'y': 40},
                    {'x': 40, 'y': 40},

                    {'x': 40, 'y': -40},
                    {'x': 20, 'y': -40},
                    {'x': 20, 'y': 40},
                    {'x': 0, 'y': 40},
                    {'x': 0, 'y': -40},
                    {'x': 20, 'y': -40},
                    {'x': 20, 'y': 40},
                    {'x': 40, 'y': 40},
                    {'x': 40, 'y': -40}
    ]
    print(f"Waypoints created {x_dim} m x {y_dim} m \n {waypoints}")

    ### Testing RRT* Algorithm
    # Environment 1 Obstacles 
    environment = [
        {
            'name': 'start'
            , 'x': 0
            , 'y': 0
        },
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
        },
        {
            'name': 'obstacle_5'
            , 'lower_x': -13
            , 'upper_x': 13
            , 'lower_y': -12
            , 'upper_y': -28
        }
    ]

    start = (0,0)
    goals = []
    goal_queue = Queue()
    map_size = (x_dim, y_dim)
    obstacles = []

    for item in environment:
        if item['name'] == 'start':
            start = (item['x'], item['y'])
        elif 'obstacle' in item['name']:
            obstacles.append(Obstacle([(item['lower_x'], item['lower_y'])
                                      , (item['upper_x'],item['lower_y'])
                                      , (item['upper_x'], item['upper_y'])
                                      , (item['lower_x'], item['upper_y'])]))
            
    paths =[]
    colors = ['red', 'blue', 'green', 'purple']

    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_xlim(-x_dim/2, x_dim/2)
    ax.set_ylim(-y_dim/2, y_dim/2)
    ax.set_aspect('equal')
    ax.set_title("RRT* Visual - Press ESC to Stop")

    fig.canvas.mpl_connect('key_press_event', on_key)

    for obs in obstacles:
        obs.draw(ax)
    ax.plot(start[0], start[1], 'ob', label='Start')
    plt.legend()
    plt.ion()
    plt.show()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        accept_thread = threading.Thread(target=accept_clients, args=(s, waypoints, ), daemon=True)
        accept_thread.start()

        while server_running:
            try:
                goal = goal_queue.get(timeout=0.1)

                print(goal)
                goal_name = goal['name']
                goal_pos = (goal['x'], goal['y'])
                print(f'Planning path to {goal_name}: {goal_pos}')
                ax.plot(goal_pos[0], goal_pos[1], 'or')

                rrt = RRTStar(start, goal_pos, map_size, obstacles)
                path = rrt.plan(ax, pause_time=0.01)
                # rrt.add_goal(goal_pos)

                if path:
                    px, py = zip(*path)
                    ax.plot(px, py, 'r-', linewidth=2)
                    print(f'Path draw to {goal_name}')
                else:
                    print(f'WARNING: No Path to {goal_name}')

                plt.pause(0.1)
            except Empty:
                plt.pause(0.01)

    

    # # Start server
    # server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # server_socket.bind((HOST, PORT))
    # server_socket.listen()
    # print(f"🚀 Server listening on {HOST}:{PORT}")

    # # Start accepting clients in a separate thread
    # accept_thread = threading.Thread(target=accept_clients, args=(server_socket, waypoints, environment, ), daemon=True)
    # accept_thread.start()
    
    # # print("🛑 Shutting down server...")
    # server_running = True  # Signal the accept_clients thread to stop
    # while server_running:
    #     plt.pause(0.1) 
    # server_socket.close()  # Close the server socket
        

    # accept_thread.join()  # Wait for thread to exit