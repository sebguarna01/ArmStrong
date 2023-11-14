#!/home/kian/projects/uni/dmms/ArmStrong/ros_venv/bin/python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
from handheld_controller_package.msg import CustomJoy
import time

class Xbox:
    def __init__(self):
        # Initialize the node
        rospy.init_node('xbox_controller_node')

        # Initialize prev_buttons
        self.prev_buttons = None
        self.custom_joy_buttons_pub = rospy.Publisher('/custom_joy', CustomJoy, queue_size=1)
        
        self.last_processed_time = 0
        self.bebounce_time = 50
        # Subscribe to the /joy topic
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.spin()

    def joy_callback(self, msg):
        current_time = time.time()*1000
        custom_joy_msg = CustomJoy()
        custom_joy_msg.A = msg.buttons[0]
        custom_joy_msg.B = msg.buttons[1]
        custom_joy_msg.X = msg.buttons[2]
        custom_joy_msg.Y = msg.buttons[3]
        custom_joy_msg.LB = msg.buttons[4]
        custom_joy_msg.RB = msg.buttons[5]
        custom_joy_msg.back = msg.buttons[6]
        custom_joy_msg.start = msg.buttons[7]
        custom_joy_msg.power = msg.buttons[8]
        custom_joy_msg.stick_button_left = msg.buttons[9]
        custom_joy_msg.stick_button_right = msg.buttons[10]
        custom_joy_msg.dpad_horizontal = msg.axes[6] if abs(msg.axes[6]) > 0.25 else 0
        custom_joy_msg.dpad_vertical = msg.axes[7] if abs(msg.axes[7]) > 0.25 else 0
        custom_joy_msg.LT = msg.axes[2] if abs(msg.axes[2]) > 0.25 else 0
        custom_joy_msg.RT = msg.axes[5] if abs(msg.axes[5]) > 0.25 else 0
        custom_joy_msg.LS_horizontal = msg.axes[0] if abs(msg.axes[0]) > 0.25 else 0
        custom_joy_msg.LS_vertical = msg.axes[1] if abs(msg.axes[1]) > 0.25 else 0
        custom_joy_msg.RS_horizontal = msg.axes[3] if abs(msg.axes[3]) > 0.25 else 0
        custom_joy_msg.RS_vertical = msg.axes[4] if abs(msg.axes[4]) > 0.25 else 0

        self.custom_joy_buttons_pub.publish(custom_joy_msg)


if __name__ == '__main__':
    xbox = Xbox()
