"""
This Python script establishes a socket connection to a UR robot controller, sends joint path commands to move the 
robot along specified trajectories, and executes scripts to open and close the gripper. It demonstrates how to control
a UR robot programmatically via socket communication.
"""
import socket
import time
from Code.list_transformation import list_transformation

# Robot UR's IP address
HOST = "10.10.73.23X"

# Port of the server on the robot
PORT = 30002

# Scripts to open and close the gripper
Abrir_pinza = 'pinza40UR3.py'
Cerrar_pinza = 'pinza10UR3.py'

def send_joint_path(path, sock):
    """
    Send a joint path to the robot controller to move along specified configurations.
        :param path: A list of joint configurations (each configuration is a list of joint angles in radians).
        :param sock: The socket object for communication with the robot controller.
    """
    for joint_config in path:
        print(joint_config)
        sock.send(f"movej({joint_config}, a=0.5, v=0.5)".encode() + "\n".encode())
        time.sleep(3)

# Connect to the robot controller via socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# Trajectory -- configurations (joint variables in radians)
path1, path2, path3 = read_file()

# Send the trajectory to the robot controller
send_joint_path(path, sock)

# Send script file to close the gripper
with open(Cerrar_pinza, 'rb') as f:
    sock.sendall(f.read())
time.sleep(1)

# Send the trajectory to the robot controller
send_joint_path(path, sock)

# Send script file to open the gripper
with open(Abrir_pinza, 'rb') as f:
    sock.sendall(f.read())
time.sleep(1)

# Send the trajectory to the robot controller
send_joint_path(path, sock)

# Message printed when the trajectory execution is completed
print("Trayectoria finalizada")

data = sock.recv(1024)

# Close the connection
sock.close()
