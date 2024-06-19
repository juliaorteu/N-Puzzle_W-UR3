"""
This Python script establishes a socket connection to a UR robot controller, sends joint path commands to move the
robot along specified trajectories, and executes scripts to open and close the gripper. It demonstrates how to control
a UR robot programmatically via socket communication. The joint paths are read from an XML file using a custom
function from the 'conf_extraction' module.
"""

import socket
import time
from conf_extraction import *

# Robot UR's IP address
HOST = "10.10.73.236"

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

# Read joint paths from the XML file
paths = read_file()

# Truncate paths to include only the first six joint angles
paths = [[i[:6] for i in path] for path in paths]

# Send the joint paths to the robot controller and the script file to close the gripper
send_joint_path(path[0], sock)
with open(Cerrar_pinza, 'rb') as f:
    sock.sendall(f.read())
time.sleep(1)

send_joint_path(path[1], sock)
with open(Abrir_pinza, 'rb') as f:
    sock.sendall(f.read())
time.sleep(1)


# Send the joint paths to the robot controller
send_joint_path(path[2], sock)
send_joint_path(path[3], sock)
with open(Cerrar_pinza, 'rb') as f:
    sock.sendall(f.read())
time.sleep(1)

send_joint_path(path[4], sock)
send_joint_path(path[5], sock)

with open(Abrir_pinza, 'rb') as f:
    sock.sendall(f.read())
time.sleep(1)

# Send a single joint configuration to the robot controller
final_path=[[0.8567222222, 0.3292777778, 0.6716666667, 0.2431944444, 0.2560277778, 0.3501388889]]
send_joint_path(final_path, sock)

# Message printed when the trajectory execution is completed
print("Trayectoria finalizada")
data = sock.recv(1024)

# Close connection
sock.close()
