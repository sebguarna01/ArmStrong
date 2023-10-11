#!/usr/bin/env python3.8
import time
import serial
from vpython import *

arduinoData = serial.Serial('/dev/ttyUSB0',baudrate = 9600,
                            timeout = 0.1)
time.sleep(1)

while True:
    while arduinoData.in_waiting == 0:
        pass
    
    # read and condition data
    dataPacket = arduinoData.readline().decode().strip()
    
    # split into 2 values
    sensorValues = dataPacket.split(',')
    
    # convert to an array (list) of floats
    uiArray = [float(n) for n in sensorValues]
    
    # condition the values; from 0,-1+2^8 to -1,1
    uiArray[0] = -1+2*uiArray[0]/1023
    uiArray[1] = -1+2*uiArray[1]/1023

    print(uiArray)
