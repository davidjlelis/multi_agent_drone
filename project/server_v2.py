# server.py
import socket
import threading
import json

HOST = '127.0.0.1'
PORT = 5555

WAYPOINTS = [
    {"x": -10, "y": -25},
    {"x": -20, "y": 20},
    {"x": 20, "y": -20},
    {"x": 20, "y": 20}
]

def handle_client(conn, addr):
    print(f"🔗 New connection from {addr}")
    try:
        # Send waypoints as JSON
        waypoints_json = json.dumps(WAYPOINTS).encode()
        
        # First send the size of the data
        conn.sendall(len(waypoints_json).to_bytes(8, byteorder='big'))
        conn.sendall(waypoints_json)
        print(f"✅ Sent waypoints to {addr}")
    except Exception as e:
        print(f"❌ Error with {addr}: {e}")
    finally:
        conn.close()
        print(f"🔌 Connection closed: {addr}")

def start_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((HOST, PORT))
    server_socket.listen()

    print(f"🚀 Waypoint Server listening on {HOST}:{PORT}")

    try:
        while True:
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()
    except KeyboardInterrupt:
        print("\n🛑 Server shutting down.")
    finally:
        server_socket.close()

if __name__ == "__main__":
    start_server()
