#!/usr/bin/env python3.8
import time
import serial
from vpython import *
import time
import socket
import struct 
import array

# send data through sockets function
def sendUI(arr):
    # Send data to the connected client
    float_array = array.array('f', [arr[0], arr[1]])
    # Convert the array to bytes
    data_to_send = float_array.tobytes()
    # Send the bytes
    client_socket.send(data_to_send)

# arduino setup
arduinoData = serial.Serial('/dev/ttyUSB0',baudrate = 9600,
                            timeout = 10)
time.sleep(1)

'''
# Setup sender socket
# Create a socket object
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Define the IP address and port to listen on
host = '127.0.0.1'  # Loopback address (localhost)
port = 23456
# Bind the socket to the address and port
server_socket.bind((host, port))
# Listen for incoming connections
server_socket.listen(1)
print("Waiting for a connection...")
# Wait for connections
client_socket, client_address = server_socket.accept()
print("Connected to:", client_address)
'''

# Define the server (Script B) address and port
server_address = ('', 22222)
# Create a socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# Connect to the server (Script B)
client_socket.connect(server_address)

while True:
    while arduinoData.in_waiting == 0:
        pass
    
    # read data
    dataPacket = arduinoData.readline().decode().strip()
    
    # split into 2 values
    sensorValues = dataPacket.split(',')
    
    # convert to an array of floats
    uiArray = [float(n) for n in sensorValues]
    
    # condition such that range = [-1, 1]
    uiArray[0] = -1+2*uiArray[0]/1023
    uiArray[1] = -1+2*uiArray[1]/1023

    # send array to traj_generator
    sendUI(uiArray)