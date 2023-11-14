#!/home/kian/projects/uni/dmms/ArmStrong/ros_venv/bin/python3
import rospy
from handheld_controller_package.msg import CustomJoy
from std_msgs.msg import Float64MultiArray, String, Int8, Float64
import time
import roboticstoolbox as rtb

class RobotController:
    def __init__(self):
        self.joint_control_mode = False
        self.current_joint = 0
        self.joints_limit = 4
        self.last_a_msg = 0
        self.dpad_last_msg_time = 0
        self.joint_mode_map = {False: "off", True: "on"}
        self.joint_control_mode_string = "cartesian"
        self.cartesian_control_pub = rospy.Publisher('/cartesian_control', Float64MultiArray, queue_size=1)
        self.joint_control_mode_pub = rospy.Publisher('/joint_control_mode', String, queue_size=1)
        self.individual_joint_control_pub = rospy.Publisher('/individual_joint_control', Float64, queue_size=1)
        self.current_joint_index_pub = rospy.Publisher("/current_joint_index", Int8, queue_size=1)

        rospy.init_node('robot_controller')
        rospy.Subscriber('/custom_joy', CustomJoy, self.controller_callback)
        rospy.spin()

    def joint_control_navigation(self, msg):
        current_time = time.time() * 1000
        # use msg.dpad_vertical to select which joint you're on
        if msg.dpad_vertical > 0 and current_time - self.dpad_last_msg_time > 100:
            self.current_joint += 1
            self.dpad_last_msg_time = current_time
            if self.current_joint > self.joints_limit:
                self.current_joint = 1
            rospy.loginfo('current joint: %i', self.current_joint)
        elif msg.dpad_vertical < 0 and current_time - self.dpad_last_msg_time > 100:
            self.current_joint -= 1
            self.dpad_last_msg_time = current_time
            if self.current_joint < 1:
                self.current_joint = 4
            rospy.loginfo('current joint: %i', self.current_joint)
        current_joint_index_msg = Int8()
        current_joint_index_msg.data = self.current_joint
        joint_control_mode_string_msg = String()
        joint_control_mode_string_msg.data = self.joint_control_mode_string
        self.current_joint_index_pub.publish(current_joint_index_msg)
        self.joint_control_mode_pub.publish(joint_control_mode_string_msg)
        self.individual_joint_control_pub.publish(msg.RS_horizontal)

    def cartesian_robot_navigation(self, msg):
        cartesian_control_msg = Float64MultiArray()
        velocity_scale = 0.05
        delta_x = msg.RS_horizontal
        delta_z = msg.RS_vertical
        delta_y = msg.LS_vertical
        cartesian_control_msg.data = [delta_x, delta_y, delta_z, velocity_scale]
        joint_control_mode_string_msg = String()
        joint_control_mode_string_msg.data = self.joint_control_mode_string
        self.cartesian_control_pub.publish(cartesian_control_msg)
        self.joint_control_mode_pub.publish(joint_control_mode_string_msg)

    def controller_callback(self, msg):
        if msg.A == 1 and self.last_a_msg != msg.A:
            self.joint_control_mode = not self.joint_control_mode
            # if self.joint_control_mode == True:
            #     self.joint_control_mode = False
            # elif self.joint_control_mode == False:
            #     self.joint_control_mode = True
            rospy.loginfo('Joint control mode is %s', self.joint_control_mode)
        self.last_a_msg = msg.A 
        if self.joint_control_mode is True:
            self.joint_control_mode_string = "individual"
            self.joint_control_navigation(msg)
        elif self.joint_control_mode is False:
            self.joint_control_mode_string = "cartesian"
            self.cartesian_robot_navigation(msg)
        pass

    def jointstate_callback(self, msg):
        self.joint_state = msg.position

if __name__ == '__main__':
    controller = RobotController()