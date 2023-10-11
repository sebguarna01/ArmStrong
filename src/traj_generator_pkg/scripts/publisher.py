#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import numpy as np
import time


# publish joint values to topic
def publish(joints):
    pub_joint_1.publish(joints[0])
    #pub_joint_2.publish(joints[1])
    #pub_joint_3.publish(joints[2])
    pub_joint_4.publish(joints[3])

def new_joint_positions():
    pos = (count-50)*3.14/50
    arr = np.zeros(4)
    for i in range(4):
        arr[i] = pos
    return arr

if __name__ == '__main__':
    '''
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
    
    # internal joint positions data structrue 
    joint_positions = np.zeros(4)
    
    # set rate at which messages will be published (Hz)
    rate = rospy.Rate(1)
    '''
    # continuous loop
    count = 1
    while not rospy.is_shutdown():
        joint_positions = new_joint_positions()
        
        # for testing
        if count < 101:
            count+=count
        else:
            count = 1

        publish(joint_positions)

        #rate.sleep()