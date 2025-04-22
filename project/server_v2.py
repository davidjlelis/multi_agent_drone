import socket
import threading
import json
import math
import sys

HOST = 'localhost'
PORT = 8000
NUM_DRONES = 3  # Adjust as needed

FULL_WAYPOINT_LIST = [
    [5.0, 5.0], [10.0, 10.0],
    [-5.0, 5.0], [-10.0, 10.0],
    [-5.0, -5.0], [-10.0, -10.0]
]

connected_drones = []
drone_waypoints = {}
lock = threading.Lock()
server_running = True

def split_waypoints_among_drones():
    chunk_size = math.ceil(len(FULL_WAYPOINT_LIST) / NUM_DRONES)
    return [FULL_WAYPOINT_LIST[i:i+chunk_size] for i in range(0, len(FULL_WAYPOINT_LIST), chunk_size)]

def handle_client(conn, addr):
    global server_running
    try:
        drone_name = conn.recv(1024).decode().strip()
        print(f"[SERVER] Drone connected: {drone_name} from {addr}")

        with lock:
            connected_drones.append((drone_name, conn))
            if len(connected_drones) == NUM_DRONES:
                print("[SERVER] All drones connected. Distributing waypoints...")
                chunks = split_waypoints_among_drones()
                for i, (drone_name, conn) in enumerate(connected_drones):
                    assigned_waypoints = chunks[i] if i < len(chunks) else []
                    drone_waypoints[drone_name] = assigned_waypoints
                    data = json.dumps(assigned_waypoints)
                    conn.sendall(data.encode())
                    conn.close()
                server_running = False  # Stop accepting more connections after distribution

    except Exception as e:
        print(f"[SERVER ERROR] {e}")
        conn.close()

def start_input_monitor():
    global server_running
    while server_running:
        user_input = input()
        if user_input.strip().lower() == 'x':
            print("[SERVER] Shutdown command received. Exiting...")
            server_running = False
            break

def start_server():
    global server_running
    print(f"[SERVER] Starting on {HOST}:{PORT}")
    print('[SERVER] Press "X" and hit Enter to terminate the server at any time.')

    input_thread = threading.Thread(target=start_input_monitor, daemon=True)
    input_thread.start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()

        while server_running and len(connected_drones) < NUM_DRONES:
            try:
                s.settimeout(1.0)
                conn, addr = s.accept()
                threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[SERVER ERROR] {e}")
                break

    print("[SERVER] Shutting down...")

if __name__ == "__main__":
    start_server()
