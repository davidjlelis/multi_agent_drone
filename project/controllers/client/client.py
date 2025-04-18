# Copyright 1996-2024 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Example of Python controller for Mavic patrolling around the house.
   Open the robot window to see the camera view.
   This demonstrates how to go to specific world coordinates using its GPS, imu and gyroscope.
   The drone reaches a given altitude and patrols from waypoint to waypoint."""

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Keyboard, LED
import math
import socket
import time
import sys
import os
import struct
import json
try: 
    import cv2
except ImportError:
    sys.exit("Warning: 'opencv' module not found.")
try:
    import numpy as np
except ImportError:
    sys.exit("Warning: 'numpy' module not found.")
from cryptography.fernet import Fernet
sys.path.append("../..")  # Allow imports from two levels up
from key_manager import encryption_key  # Import the shared key
cipher = Fernet(encryption_key)

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_OFFSET = 0.6
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    # Precision between the target position and the robot position in meters
    target_precision = 0.5

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
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
        self.camera_pitch_motor.setPosition(0.7)
        motors = [self.front_left_motor, self.front_right_motor,
                  self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.current_pose = 6 * [0]  # X, Y, Z, yaw, pitch, roll
        self.target_position = [0, 0, 0]
        self.target_index = 0
        self.target_altitude = 0

    def set_position(self, pos):
        """
        Set the new absolute position of the robot
        Parameters:
            pos (list): [X,Y,Z,yaw,pitch,roll] current absolute position and angles
        """
        self.current_pose = pos
        
    def connect_server(self):
        HOST = '127.0.0.1'
        PORT = 5555
        connection_attempts = 0
        delay = 1  # Start with 1 second delay

        while connection_attempts < 3:
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.connect((HOST, PORT))
                print("✅ Connected to server.")
                return client_socket
            except ConnectionRefusedError:
                print(f"❌ Server not available. Retrying in {delay} seconds...")
                time.sleep(delay)
                delay = min(delay * 2, 10)  # Exponential backoff (capped at 10s)
                connection_attempts += 1

        print("⛔ Maximum retries reached. Giving up on connection.")
        return None  # Return None if the connection fails


    def move_to_target(self, waypoints, verbose_movement=False, verbose_target=False):
        """
        Move the robot to the given coordinates
        Parameters:
            waypoints (list): list of X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_position[0:2] == [0, 0]:  # Initialization
            self.target_position[0:2] = waypoints[0]
            if verbose_target:
                print("First target: ", self.target_position[0:2])

        # if the robot is at the position with a precision of target_precision
        if all([abs(x1 - x2) < self.target_precision for (x1, x2) in zip(self.target_position, self.current_pose[0:2])]):

            self.target_index += 1
            if self.target_index > len(waypoints) - 1:
                self.target_index = 0
            self.target_position[0:2] = waypoints[self.target_index]
            if verbose_target:
                print("Target reached! New target: ",
                      self.target_position[0:2])

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

    def run(self):
        # client_socket = connect_server(initialization=True)
        client_socket = self.connect_server()
        t1 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        # Specify the patrol coordinates
        waypoints = [[5, 5], [-5, 5], [5, -5], [-5, -5]]
        # target altitude of the robot in meters
        self.target_altitude = 10

        # try:
        #     with open("client_output.txt", "x") as file:
        #         file.write("New file created.\n")
        # except FileExistsError:
        #     #print("client_output.txt already exists.")
        #     open("client_output.txt", "w").close()

        while self.step(self.time_step) != -1:
            
            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            x_pos, y_pos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([x_pos, y_pos, altitude, roll, pitch, yaw])

            telemetry_data = {"x_d": x_pos
                              , "y_d": y_pos
                              , "altitude": altitude
                              , "roll": roll
                              , "pitch": pitch
                              , "yaw": yaw}
            
            json_data = json.dumps(telemetry_data).encode()
            json_length = len(json_data)

            # Prepare telemetry data
            #telemetry_data = f"Time: {t1:.2f}, Altitude: {altitude:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}\n"
            #print(f"Telemetry Data: {telemetry_data}")
            #encrypted_telemetry_data = cipher.encrypt(telemetry_data.encode())
            
            # with open("client_output.txt", "a") as file:
            #     file.write("Unencrypted message: "+telemetry_data) 
            #     file.write("Encrypted message: "+encrypted_telemetry_data.decode()+"\n")
                
            # Sending images to server
            image = self.camera.getImage()

            if image:
                img_array = np.frombuffer(image, dtype=np.uint8).reshape((self.camera.getHeight(), self.camera.getWidth(), 4))
                frame = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

                # Encode frame as JPEG safely
                success, encoded_frame = cv2.imencode('.jpg', frame)
                if success:
                    img_bytes = encoded_frame.tobytes()
                    img_length = len(img_bytes)
                else:
                    print("⚠️ Frame encoding failed. Skipping transmission.")
                    data = None  # Avoid sending corrupted data


            if client_socket:
                try:
                    emergency_landing = False
                    #client_socket.sendall(struct.pack("L", len(data)) + data)
                    client_socket.sendall(struct.pack("QQ", json_length, img_length))
                    client_socket.sendall(json_data)
                    client_socket.sendall(img_bytes)

                    #client_socket.send(encrypted_telemetry_data)
                    #print(f"📡 Sent: {encrypted_telemetry_data}")
                except:
                    print(f"⚠️ Connection lost! Reconnecting...")
                    client_socket.close()
                    client_socket = self.connect_server()

            if not client_socket:
                print('Lost connection to server. Returning to emergency landing point.')
                emergency_landing = True
                waypoints = [[0,0]]
            
            # if emergency_landing and x_pos < 0.5 and x_pos > -0.5 and y_pos < 0.5 and y_pos > -0.5:
            #     self.target_altitude = 0

            if emergency_landing:
                distance_to_origin = math.sqrt(x_pos**2 + y_pos**2)
                if distance_to_origin < 1.0:
                    self.target_altitude = 0
 

                
            if emergency_landing and altitude < 0.25:
                self.front_left_motor.setVelocity(0)
                self.front_right_motor.setVelocity(0)
                self.rear_left_motor.setVelocity(0)
                self.rear_right_motor.setVelocity(0)
                break
    
            if altitude > self.target_altitude - 1:
                # as soon as it reach the target altitude, compute the disturbances to go to the given waypoints.
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(
                        waypoints)
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
                
                

# To use this controller, the basicTimeStep should be set to 8 and the defaultDamping
# with a linear and angular damping both of 0.5
robot = Mavic()
robot.run()