#!/home/kian/projects/uni/dmms/ArmStrong/ros_venv/bin/python3
import roboticstoolbox as rtb
from math import pi
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray, String, Float64, Int8
from sensor_msgs.msg import JointState
from spatialmath.base import transl

class Robot:
    def __init__(self, links, name):
        rospy.init_node('robot')
        self.name = name
        self.robot = rtb.DHRobot(
            links,
            name=name
        )
        # Initialise q and T
        self.end_effector_pose = np.eye(4)
        self.T = np.eye(4)
        self.initial_joint_angles = [0, pi/2, 0, 0, 0]
        self.initial_end_effector_pose = self.forward_kinematics(self.initial_joint_angles)
        self.current_joint_angles = self.initial_joint_angles
        self.current_end_effector_pose = self.forward_kinematics(self.current_joint_angles)
        self.desired_joint_angles = self.forward_kinematics(self.initial_joint_angles)
        self.desired_end_effector_pose = self.forward_kinematics(self.current_joint_angles)
        self.joint_control_mode = 'cartesian'
        self.current_joint_index = 0
        self.previous_joint_index = 0
        self.delta_x = 0
        self.delta_y = 0
        self.delta_z = 0
        self.velocity_scale = 0
        self.joint_velocities = [0, 0, 0, 0, 0]
        self.joint_pubs = [rospy.Publisher(f'robot/joint_{i + 1}_position_controller/command', Float64, queue_size=1) for i in range(5)]
        rospy.Subscriber('/cartesian_control', Float64MultiArray, self.cartesian_callback)
        rospy.Subscriber('/joint_control_mode', String, self.joint_control_mode_callback)
        rospy.Subscriber('/individual_joint_control', Float64, self.individual_joint_control_callback)
        rospy.Subscriber('/current_joint_index', Int8, self.current_joint_index_callback)

        rospy.spin()

    def forward_kinematics(self, q):
        return self.robot.fkine(q)
    
    def inverse_kinematics(self, T):
        T, *_ = self.robot.ikine_LM(T, slimit=100)  # multiple solutions are possible, taking the first one
        return T
    
    def move_cartesian(self):
        if self.joint_control_mode == 'individual':
            return
        time_step = 0.1
        # dx may be impossible here
        dx = self.delta_x * time_step * self.velocity_scale
        dy = self.delta_y * time_step * self.velocity_scale
        dz = self.delta_z * time_step * self.velocity_scale
        
        self.desired_end_effector_pose = self.current_end_effector_pose * transl(dx, dy, dz)

        self.desired_joint_angles = self.inverse_kinematics(self.desired_end_effector_pose)
        if self.desired_joint_angles is not None:
            self.current_end_effector_pose = self.desired_end_effector_pose
            self.current_joint_angles = self.desired_joint_angles
        else:
            rospy.logwar("Ikine did not find a solution, this is bad")
        self.publish_joint_states()
    
    def move_individual_joint(self, individual_joint_control_value):
        if self.joint_control_mode == 'cartesian':
            return
        time_step = 0.1
        joint_index = self.current_joint_index
        self.current_joint_angles[joint_index] += individual_joint_control_value * time_step
        min_angle, max_angle = self.robot.links[joint_index].qlim
        self.current_joint_angles[joint_index] = np.clip(self.current_joint_angles[joint_index], min_angle, max_angle)

        self.publish_joint_states()
    
    def cartesian_callback(self, msg):
        self.delta_x = msg.data[0]
        self.delta_y = msg.data[1]
        self.delta_z = msg.data[2]
        self.velocity_scale = msg.data[3]

    def individual_joint_control_callback(self, msg):
        individual_joint_control_value = msg.data
        self.move_individual_joint(individual_joint_control_value)

    def current_joint_index_callback(self, msg):
        self.current_joint_index = msg.data - 1

    def joint_control_mode_callback(self, msg):
        self.joint_control_mode = msg.data

    def publish_joint_states(self):
        current_joint_angles = self.current_joint_angles
        joint_publishers = self.joint_pubs
        for publisher, joint_angle in zip(joint_publishers, current_joint_angles):
            joint_angle_msg = Float64()
            joint_angle_msg.data = joint_angle
            publisher.publish(joint_angle_msg)

    # def move_ikine(self):
    #     num_steps = 100
    #     for s in np.linspace(0, 1, num_steps):
    #         intermediate_pose = SE3.interpolate(self.current_end_effector_pose, self.desired_end_effector_pose, s)
    #         intermediate_joint_angles = self.inverse_kinematics(intermediate_pose)

    #         current_end_effector_pose 

if __name__ == "__main__":
    links = [
        rtb.DHLink(theta=0, d=0, a=0, alpha=0, offset=0, qlim=[-pi, pi]),
        rtb.DHLink(theta=0, d=0.09, a=0, alpha=0, offset=pi/2, qlim=[-pi/2, pi/2]),
        rtb.DHLink(theta=0, d=0.52453, a=0, alpha=pi/2, offset=0, qlim=[(-pi - 0.79), 0.79]),
        rtb.DHLink(theta=0, d=0, a=0.52149, alpha=pi/2, offset=0, qlim=[-pi, 0]),
        rtb.DHLink(theta=0, d=0.0897, a=0, alpha=0, offset=0, qlim=[-2.64, 2.64])
    ]
    robot = Robot(links, name='robot')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        robot.move_cartesian()
        rate.sleep()
