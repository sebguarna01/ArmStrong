#!/usr/bin/env python2
import rospy
from std_msgs.msg import Float64
import numpy as np
import socket
import struct
import array
import time

# publish joint values to topic
def publish(joints):
    pub_joint_1.publish(joints[0])
    pub_joint_2.publish(joints[1])
    pub_joint_3.publish(joints[2])
    pub_joint_4.publish(joints[3])

if __name__ == '__main__':
    
    # initialise the node
    rospy.init_node("traj_publisher")
    
    # create joint-wise publishers
    pub_joint_1 = rospy.Publisher('/robot/joint_1_position_controller/command', 
                                  Float64, queue_size=10)
    pub_joint_2 = rospy.Publisher('/robot/joint_2_position_controller/command', 
                                  Float64, queue_size=10)
    pub_joint_3 = rospy.Publisher('/robot/joint_3_position_controller/command', 
                                  Float64, queue_size=10)
    pub_joint_4 = rospy.Publisher('/robot/joint_4_position_controller/command', 
                                  Float64, queue_size=10)
    
    # set rate at which messages will be published (Hz)
    rate = rospy.Rate(5) # is moot for now

    # Setup receiver socket (receive from traj_generator.py)
    # Create a socket object
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # Define the IP address and port to connect to
    host = socket.gethostname()  # Loopback address (localhost)
    port = 5000
    # Bind the socket to the address and port
    client_socket.bind((host, port))
    # Listen for incoming connections
    client_socket.listen(1)
    print("Waiting for a connection...")
    # Wait for connections
    client_socket, client_address = client_socket.accept()
    print("Connected to:", client_address)
    
    
    # continuous loop
    #while not rospy.is_shutdown():
    count = 1
    while(1):
        # Receive joint states from traj_generator
        # Wait for receipt of the bytes
        float_bytes = client_socket.recv(1024)  # note: is blocking call
        # Convert the bytes back to a float array
        received_float_array = array.array('f')
        received_float_array.fromstring(float_bytes)
        #print(received_float_array)

        # Publish the joint states to ROS
        publish(received_float_array)
