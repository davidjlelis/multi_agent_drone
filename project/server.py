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

# class Node:
#     def __init__(self, x, y) -> None:
#         self.x, self.y = x, y
#         self.node_type = None
#         self.parent = None
#         self.cost = 0
#         pass

# class Obstacle:
#     def __init__(self, vertices) -> None:
#         self.polygon = Polygon(vertices)
#         pass

#     def collides(self, x, y):
#         return self.polygon.contains(Point(x,y))
    
#     def line_intersects(self, x1, y1, x2, y2):
#         line = LineString([(x1, y1), (x2, y2)])
#         return self.polygon.intersects(line)
    
#     def draw(self, ax):
#         x, y = self.polygon.exterior.xy
#         ax.fill(x, y, color='black', alpha=0.4)

# class RRTStar:
#     def __init__(self, start, goal, map_size, obstacles=None,
#                  step_size=5, goal_sample_rate=0.1,
#                  search_radius=15, max_iter=500):
#         self.start = Node(*start)
#         self.goal = Node(*goal)
#         self.map_size = map_size
#         self.obstacles = obstacles or []
#         self.step_size = step_size
#         self.goal_sample_rate = goal_sample_rate
#         self.search_radius = search_radius
#         self.max_iter = max_iter
#         self.nodes = [self.start]

#     def distance(self, n1, n2):
#         return np.hypot(n1.x - n2.x, n1.y - n2.y)

#     def sample(self):
#         if random.random() < self.goal_sample_rate:
#             return self.goal
#         return Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

#     def nearest(self, random_node):
#         return min(self.nodes, key=lambda node: self.distance(node, random_node))

#     def steer(self, from_node, to_node):
#         dist = self.distance(from_node, to_node)
#         if dist < self.step_size:
#             return to_node
#         theta = np.arctan2(to_node.y - from_node.y, to_node.x - from_node.x)
#         new_x = from_node.x + self.step_size * np.cos(theta)
#         new_y = from_node.y + self.step_size * np.sin(theta)
#         new_node = Node(new_x, new_y)
#         new_node.parent = from_node
#         new_node.cost = from_node.cost + self.step_size
#         return new_node

#     def is_collision_free(self, node):
#         for obs in self.obstacles:
#             if obs.collides(node.x, node.y):
#                 return False
#         return 0 <= node.x <= self.map_size[0] and 0 <= node.y <= self.map_size[1]

#     def get_nearby_nodes(self, new_node):
#         return [node for node in self.nodes if self.distance(node, new_node) <= self.search_radius]

#     def choose_parent(self, new_node, nearby_nodes):
#         min_cost = float('inf')
#         best_node = None
#         for node in nearby_nodes:
#             if not self.check_line_collision(node, new_node):
#                 cost = node.cost + self.distance(node, new_node)
#                 if cost < min_cost:
#                     min_cost = cost
#                     best_node = node
#         if best_node:
#             new_node.parent = best_node
#             new_node.cost = min_cost

#     def rewire(self, new_node, nearby_nodes):
#         for node in nearby_nodes:
#             if not self.check_line_collision(new_node, node):
#                 new_cost = new_node.cost + self.distance(new_node, node)
#                 if new_cost < node.cost:
#                     node.parent = new_node
#                     node.cost = new_cost

#     def check_line_collision(self, n1, n2):
#         for obs in self.obstacles:
#             if obs.line_intersects(n1.x, n1.y, n2.x, n2.y):
#                 return True
#         return False

#     def is_goal_reached(self, node):
#         return self.distance(node, self.goal) < self.step_size and not self.check_line_collision(node, self.goal)

#     def extract_path(self):
#         path = []
#         node = self.goal
#         while node:
#             path.append((node.x, node.y))
#             node = node.parent
#         return path[::-1]  # reverse

#     def plan(self, ax, pause_time=0.01):
#         for _ in range(self.max_iter):
#             rnd = self.sample()
#             nearest_node = self.nearest(rnd)
#             new_node = self.steer(nearest_node, rnd)

#             if not self.is_collision_free(new_node):
#                 continue

#             nearby = self.get_nearby_nodes(new_node)
#             self.choose_parent(new_node, nearby)
#             self.nodes.append(new_node)
#             self.rewire(new_node, nearby)

#             if new_node.parent:
#                 ax.plot([new_node.x, new_node.parent.x], [new_node.y, new_node.parent.y], "-g", linewidth=0.5)
#                 plt.pause(pause_time)

#             if self.is_goal_reached(new_node):
#                 if not self.check_line_collision(new_node, self.goal):
#                     self.goal.parent = new_node
#                     self.goal.cost = new_node.cost + self.distance(new_node, self.goal)
#                     self.nodes.append(self.goal)

#                     # Draw final goal connection
#                     ax.plot([self.goal.x, new_node.x], [self.goal.y, new_node.y], "-g", linewidth=0.5)
#                     plt.pause(pause_time)
#                     break

#         return self.extract_path()

#     def draw_final(self, ax, path=None):
#         if path:
#             px, py = zip(*path)
#             ax.plot(px, py, "-r", linewidth=2, label="Final Path")

#         ax.plot(self.start.x, self.start.y, "ob", label="Start")
#         ax.plot(self.goal.x, self.goal.y, "or", label="Goal")
#         ax.legend()
#         ax.set_xlim(0, self.map_size[0])
#         ax.set_ylim(0, self.map_size[1])
#         ax.set_aspect('equal')
#         ax.set_title("Live RRT* with Polygon Obstacles")

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

    def distance(self, n1, n2):
        return np.hypot(n1.x - n2.x, n1.y - n2.y)

    def sample(self):
        if random.random() < self.goal_sample_rate:
            return self.goal
        return Node(random.uniform(0, self.map_size[0]), random.uniform(0, self.map_size[1]))

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
        for obs in self.obstacles:
            if obs.collides(node.x, node.y):
                return False
        return 0 <= node.x <= self.map_size[0] and 0 <= node.y <= self.map_size[1]

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
        ax.set_xlim(0, self.map_size[0])
        ax.set_ylim(0, self.map_size[1])
        ax.set_aspect('equal')
        ax.set_title("Safe RRT* Path Planning")


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

    # Environment 1 Obstacles
    enivornment = [
        {
            'name': 'start'
            , 'x': 0
            , 'y': 0
        },
        {
            'name': 'goal_1'
            , 'x': -15
            , 'y': 40
        },
        {
            'name': 'goal_2'
            , 'x': 40
            , 'y': 40
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
        }
    ]

    start = (0,0)
    goals = []
    map_size = (x_dim, y_dim)
    obstacles = []

    for item in enivornment:
        if item['name'] == 'start':
            start = (item['x']+50, item['y']+50)
        elif 'goal' in item['name']:
            goals.append((item['x']+50, item['y']+50))
        elif 'obstacle' in item['name']:
            obstacles.append(Obstacle([(item['lower_x']+50, item['lower_y']+50)
                                      , (item['upper_x']+50,item['lower_y']+50)
                                      , (item['upper_x']+50, item['upper_y']+50)
                                      , (item['lower_x']+50, item['upper_y']+50)]))
            
    paths =[]
    colors = ['red', 'blue', 'green', 'purple']
            
    fig, ax = plt.subplots(figsize=(8,8))
    ax.set_xlim(0, map_size[0])
    ax.set_xlim(0, map_size[1])
    ax.set_aspect('equal')
    ax.set_title('RRT* for multliple goals')

    for obs in obstacles:
        obs.draw(ax)

    ax.plot(start[0], start[1], 'ob', label='Start')

    for i, goal in enumerate(goals):
        print(f'\n Planning path to Goal {i+1}: {goal}')

        rrt_star = RRTStar(start, goal, map_size, obstacles)
        path = rrt_star.plan(ax, pause_time=0.01)
        paths.append(path)

        if path:
            px, py = zip(*path)
            ax.plot(px, py, color=colors[i%len(colors)], linewidth=2, label=f'Goal {i+1}')
            ax.plot(goal[0], goal[1], 'o', color=colors[i%len(colors)])

        plt.pause(0.5)

    ax.legend()
    plt.show()

    # rrt_star = RRTStar(start=start, goal=goal, map_size=map_size, obstacles=obstacles)
    # path = rrt_star.plan(ax=ax)
    # rrt_star.draw_final(ax, path)
    # plt.show()

    # Start server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen()
    print(f"🚀 Server listening on {HOST}:{PORT}")

    # Start accepting clients in a separate thread
    accept_thread = threading.Thread(target=accept_clients, args=(server_socket, waypoints,), daemon=True)
    accept_thread.start()
    
    print("🛑 Shutting down server...")
    server_running = False  # Signal the accept_clients thread to stop
    server_socket.close()  # Close the server socket

    accept_thread.join()  # Wait for thread to exit