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
        self.map_bounds = (-map_size[0]/2, -map_size[1]/2, map_size[0]/2, map_size[1]/2)
        self.obstacles = obstacles or []
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.max_iter = max_iter
        self.nodes = [self.start]

    def distance(self, n1, n2):
        return np.hypot(n1.x - n2.x, n1.y - n2.y)

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal
        min_x, min_y, max_x, max_y = self.map_bounds
        return Node(random.uniform(min_x, max_x), random.uniform(min_y, max_y))


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

    def is_collision_free(self, node):
        min_x, min_y, max_x, max_y = self.map_bounds
        if not (min_x <= node.x <= max_x and min_y <= node.y <= max_y):
            return False
        for obs in self.obstacles:
            if obs.collides(node.x, node.y):
                return False
        return True


    def check_line_collision(self, n1, n2):
        for obs in self.obstacles:
            if obs.line_intersects(n1.x, n1.y, n2.x, n2.y):
                return True
        return False

    def get_nearby_nodes(self, new_node):
        return [node for node in self.nodes if self.distance(node, new_node) <= self.search_radius]

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

    def extract_path(self):
        path = []
        node = self.goal
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

        return self.extract_path()

    def draw_final(self, ax, path=None):
        if path:
            px, py = zip(*path)
            ax.plot(px, py, "-r", linewidth=2, label="Final Path")

        ax.plot(self.start.x, self.start.y, "ob", label="Start")
        ax.plot(self.goal.x, self.goal.y, "or", label="Goal")
        ax.legend()
        min_x, min_y, max_x, max_y = self.map_bounds
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_aspect('equal')
        ax.set_title("Safe RRT* Path Planning")

def handle_client(conn, addr, waypoints):
    """Handles communication with a connected client."""
    global connection_timestamps
    with lock:
        connection_timestamps.append(time.time())  # Log new connection

    print(f"🔗 New connection from {addr}")

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

        # Create/Empty Results json file
        results_json_path = './results/'
        results_json_filename = 'results.json'
        results_json_filepath = os.path.join(results_json_path, results_json_filename)
        with open(results_json_filepath, "w") as f:
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

                if decrypted_data == b"MISSION_COMPLETE":
                    print("Survey completed! Drone has landed at (0,0).")

                detection = json.loads(decrypted_data.decode('utf-8'))

                with open(results_json_filepath, "r") as f:
                    data = json.load(f)

                data.append(detection)

                with open(results_json_filepath, "w") as f:
                    json.dump(data, f, indent=4)

        conn.close()
        print("Connection Closed.")

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

def on_key(event):
    global  server_running
    if event.key == 'escape':
        print('Closing Server...')
        server_running = False

def get_world():
    world_input = input("""
===============================================================================================================
                        Please select number that corresponds to the active test world: \n
                        1. Test World 1 (Neutral Suburban)
                        2. Test World 2 (Flood)
                        3. Test World 3 (Urban City)
===============================================================================================================
                        Selection: """)
    print("""
===============================================================================================================
""")
    if world_input == "1":
        test_world = "test_world_1"
    elif world_input == "2":
        test_world = "test_world_2"
    elif world_input == "3":
        test_world = "test_world_3"
    else:
        print('Invalid Input. Please try again.')
        test_world = get_world()
    return test_world

def get_scenario():
    scenario_input = input("""
===============================================================================================================
                        Please select number that corresponds to the active test world: \n
                        1. Safe
                        2. Rescue
===============================================================================================================
                        Selection: """)

    if scenario_input == "1":
        scenario_type = "Safe"
    elif scenario_input == "2":
        scenario_type = "Rescue"
    else:
        print('Invalid Input. Please try again.')
        scenario_type = get_scenario()
    return scenario_type

if __name__ == "__main__":
    # Global Variables
    global fig, ax, paths, colors, goal_queue #, goals

    # Get environment dimension
    # x_dim = int(input("Enter X dimension of explorable environment (in meters): "))
    # y_dim = int(input("Enter Y dimension of explorable environment (in meters): "))
    # step = 25

    x_dim = 100
    y_dim = 100
    step = 25

    x_path = x_dim-20
    y_path = y_dim-20

    test_world = "" 

    ### Get World & Scenario inputs
    test_world = get_world()
    scenario_input = get_scenario()

    
    ### Get World data
    world_description_fp = './world_descriptions/worlds.json'
    environment = []
    world_description_json = []
    world_description = ''

    with open(world_description_fp) as f:
        world_description_json = json.load(f)

    for desc in world_description_json:
        if desc['world_name'] == test_world:
            environment = desc['environment']
            world_description = desc['world_description']
            for scenario_type in desc['scenario']:
                if scenario_type['scenario_type'] == scenario_input:
                    goal = (scenario_type['x'], scenario_type['y'])
                    message = scenario_type['message']
            world_name = desc['world_name']

    print(environment, world_description, goal, message, world_name)

    ### Get goal data
    start = (0,0)
    # goals = []
    # goal_queue = Queue()
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
    ax.set_ylim(-x_dim/2, y_dim/2)
    ax.set_aspect('equal')
    ax.set_title("RRT* Visual - Press ESC to Stop")

    fig.canvas.mpl_connect('key_press_event', on_key)

    for obs in obstacles:
        obs.draw(ax)
    ax.plot(start[0], start[1], 'ob', label='Start')
    plt.legend()
    plt.ion()
    plt.show()

    ax.plot(goal[0], goal[1], 'or')
    rrt = RRTStar(start, goal, map_size, obstacles)
    path = rrt.plan(ax, pause_time=0.01)

    if path:
        px, py = zip(*path)
        ax.plot(px, py, 'r-', linewidth=2)
        # print(f'Path draw to {goal_name}')
    else:
        print(f'WARNING: No Path to {goal}')

    waypoints = []

    for point in path:
        waypoints.append({'x': point[0], 'y': point[1], 'end': False})

    world_details = { 'world_name': world_name
                    , 'world_description': world_description
                    , 'waypoints': path
                    , 'goal_position': goal
                    , 'message': message}

    print(world_details)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}")
        accept_thread = threading.Thread(target=accept_clients, args=(s, world_details, ), daemon=True)
        accept_thread.start()

        while server_running:
            plt.pause(0.1)