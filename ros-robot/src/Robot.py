#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from fiducial_msgs.msg import FiducialTransformArray

import math

class Robot:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.web_input = rospy.Subscriber('cmd_vel_mux/input/teleop', Twist, self.web_input_callback)
        #self.twist = Twist()
    
    def web_input_callback(self, data):
        self.cmd_vel_pub.publish(data)

rospy.init_node('Robot')
follower = Robot()
print("Robot.py node is running!")
rospy.spin()
