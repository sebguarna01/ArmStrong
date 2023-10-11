#!/usr/bin/env python3.8
import time
import serial
#from vpython import *
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
from math import pi
from signal import signal, SIGPIPE, SIG_DFL  
signal(SIGPIPE,SIG_DFL) 


# build robot using P. Corke's ever-bountiful toolbox
def constructRobot():
    # qlim
    global ql 
    ql = [-pi, pi]
    
    # links
    l1 = DHLink(d = .1, a = 0, alpha = pi/2)
    l2 = DHLink(d= 0, a= .5, alpha= 0,offset=pi/2, qlim = ql)
    l3 = DHLink(d= 0, a= .5, alpha= 0, qlim = ql)
    l4 = DHLink(d= 0, a= .2, alpha= 0, qlim = ql)

    # robot
    global robot 
    robot = DHRobot([l1, l2, l3, l4], name= 'robot')

    q = np.zeros([1,4]) 
    plt.close('all')  

# compute new joint position
def newJointPositionsIK(joyUI):
    if (abs(joyUI[0])>0.1 or abs(joyUI[1])>0.1) and (abs(joyUI[0])<1.1 and abs(joyUI[1])<1.1): # neglect extreme joystick input
        fudge_factor = 30
        deltaX, deltaY = -1*joyUI[0]/fudge_factor,joyUI[1]/fudge_factor # IK-specific UI signal conditioning
        T_EE = robot.fkine(q).A
        pos_EE = transl(T_EE)
        if is_in_ws(pos_EE):
            T_EE_new = SE3.Trans(deltaX+pos_EE[0], deltaY+pos_EE[1], 0)
            q_new = robot.ikine_LM(T_EE_new, q0=q, mask=[1,1,1,0,0,0],joint_limits=False).q
            # as joint 1 should be continuous, offset wraparound
            dq1 = q_new[0]-q[0]
            if abs(dq1) > pi:
                if dq1>1: # if -pi to +pi
                    q_new[0]=-2*pi-q_new[0]
                elif dq1<1: # if +pi to -pi
                    q_new[0]=2*pi+q_new[0]
            print ("New robot EE position is: ", transl(T_EE_new.A))
        else:
            q_new = q_init
    else: 
        q_new = q

    return q_new

# compute new joint position
def newJointPositionsRRMC(joyUI):
    if (abs(joyUI[0])>0.1 or abs(joyUI[1])>0.1) and (abs(joyUI[0])<1.1 and abs(joyUI[1])<1.1): # neglect extreme joystick input
        velX, velY = -1*joyUI[0]/20,joyUI[1]/20 # RRMC-specific UI signal conditioning
        ev = [velX,velY,0,0,0,0]
        J = robot.jacob0(q)
        dq = np.linalg.pinv(J) @ ev
        q_next = np.add(q,dq)
        # ensure satisfaction of qlim
        condition = lambda x: ql[0] <= x <= ql[1]
        if all(condition(x) for x in [q_next[1],q_next[2],q_next[3],]):
            q_new = q_next
        else: q_new = q
        print("Joint velocities are: ", np.round(dq, 4))
    else: 
        q_new = q

    return q_new


def is_in_ws(pos_EE):
    sphere_center = (0, 0, 0.1)
    sphere_radius = 1.185
    # Calculate the Euclidean distance between the point and the sphere's center
    distance = math.sqrt((pos_EE[0] - sphere_center[0]) ** 2 + 
                        (pos_EE[1] - sphere_center[1]) ** 2 + 
                        (pos_EE[2] - sphere_center[2]) ** 2)
    # Check if the point is inside the sphere
    if distance < sphere_radius:
        return True
    else:
        return False


# send joints to traj_publisher
def sendJoints(arr):
    # Send data to the connected client
    float_array = array.array('f', [arr[0], arr[1], arr[2], arr[3]])
    # Convert the array to bytes
    data_to_send = float_array.tobytes()
    # Send the bytes
    client_socket.send(data_to_send)

if __name__ == '__main__':
    # arduino setup
    arduinoData = serial.Serial('/dev/ttyUSB0',baudrate = 9600,
                                timeout = 10)
    time.sleep(1)

    
    # Setup sender socket
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Define the IP address and port to listen on
    host = socket.gethostname()  # Loopback address (localhost)
    port = 5000
    # Connect to the server
    client_socket.connect((host, port))
    

    constructRobot()
    # Set initial joint state
    q_init = np.array([0, pi/4, pi/2, -pi/4])
    # Declare a variable representing current joint state of robot
    q = q_init
    #sendJoints(q_init)

    i=1
    while 1:
        while arduinoData.in_waiting == 0:
            pass
        
        # get joystick data from arduino
        # read data
        dataPacket = arduinoData.readline().decode().strip()
        # split into 2 values
        sensorValues = dataPacket.split(',')
        # convert to an array of floats
        uiArray = [float(n) for n in sensorValues]
        # condition such that range = [-1, 1]
        uiArray[0] = -1+2*uiArray[0]/1023
        uiArray[1] = -1+2*uiArray[1]/1023

        # compute new joint positions based on method of preference
        #q = newJointPositionsIK(uiArray)
        q = newJointPositionsRRMC(uiArray)
        print("Joystick input is: ",uiArray)

        # send new joint positions to traj_publisher
        sendJoints(q)