from spatialmath import * 
from spatialmath.base import * 
from spatialmath.base import sym
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import keyboard
import warnings
from scipy import linalg
from roboticstoolbox import DHLink, DHRobot, jtraj
import time
import socket
import struct 
import array

# Useful variables
from math import pi

# build robot using P. Corke's ever-bountiful toolbox
def constructRobot():
    # links
    l1 = DHLink(d = .1, a = 0, alpha = pi/2)
    l2 = DHLink(d= 0, a= .5, alpha= 0, qlim= [-pi,pi],offset=pi/2)
    l3 = DHLink(d= 0, a= .5, alpha= 0, qlim= [-pi,pi])
    l4 = DHLink(d= 0, a= .2, alpha= 0, qlim= [-pi,pi])

    # robot
    global robot 
    robot = DHRobot([l1, l2, l3, l4], name= 'robot')

    q = np.zeros([1,4]) 
    plt.close('all')    
    #print(robot.fkine(q))

def sendJoints(arr):
    # Send data to the connected client
    float_array = array.array('f', [arr[0], arr[1], arr[2], arr[3]])
    # Convert the array to bytes
    data_to_send = float_array.tobytes()
    # Send the bytes
    client_c_socket.send(data_to_send)

def newJointPositions(joyUI):
    deltaX, deltaY = joyUI[0],joyUI[1]
    T_EE = robot.fkine(q).A
    T_EE_new = transl(deltaX, deltaY, 0)*T_EE
    q_new = robot.ikine_LM(T_EE_new, q0=q, mask=[1,1,1,0,0,0]).q
    return q_new

'''
def testTrajectory():
    global steps 
    steps = 60

    T1 = transl(.8, 0, 0)                                      # First pose
    T2 = transl(.1,.3, .2)                                    # Second pose

    mask = [1,1,1,0,0,0]                                       # Masking matrix
    q1 = robot.ikine_LM(T1, q0=[0,0,0,0], mask= mask).q        # Solve for joint angles
    q2 = robot.ikine_LM(T2, q0=q1, mask= mask).q        # Solve for joint angles

    q_matrix = jtraj(q1, q2, steps).q
    return q_matrix
'''

if __name__ == '__main__':
    '''
    # Setup receiver socket (receive from joystick_input.py)
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Define the IP address and port to connect to
    host = '127.0.0.1'  # Loopback address (localhost)
    port = 23456
    # Connect to the server
    client_socket.connect((host, port))
    
    # Setup sender socket (send to traj_publisher.py)
    # Create a socket object
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Define the IP address and port to listen on
    host = '127.0.0.1'  # Loopback address (localhost)
    port = 12345
    # Bind the socket to the address and port
    server_socket.bind((host, port))
    # Listen for incoming connections
    server_socket.listen(1)
    print("Waiting for a connection...")
    # Wait for connections
    client_socket, client_address = server_socket.accept()
    print("Connected to:", client_address)
    '''

    # Define the server (Script A) address and port for receiving data
    server_a_address = ('', 22222)
    # Create a socket to receive data from Script A
    server_a_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_a_socket.bind(server_a_address)
    server_a_socket.listen(1)  # Listen for incoming connections
    print("Waiting for Script A to connect...")
    script_a_socket, script_a_address = server_a_socket.accept()
    print(f"Connection established with Script A at {script_a_address}")

    # Define the server (Script C) address and port
    server_c_address = ('127.0.0.1', 54321)
    # Create a socket for communication with Script C
    client_c_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_c_socket.connect(server_c_address)


    # traj = testTrajectory() # test trajectory

    constructRobot()
    # Set initial joint state
    q_init = np.array([0, pi/4, pi/2, -pi/4])
    # Declare a variable representing current joint state of robot
    q = q_init
    sendJoints(q_init)

    while(1):
        # Receive joint states from traj_generator
        # Wait for receipt of the bytes
    
        float_bytes = server_a_socket.recv(128) 
        # Convert the bytes back to a float array
        received_float_array = array.array('f')
        received_float_array.fromstring(float_bytes)

        q = newJointPositions(received_float_array)
        print(q)
        sendJoints(q)
        