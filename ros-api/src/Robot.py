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
        self.twist = Twist()
            
rospy.init_node('Robot')
follower = Robot()
rospy.spin()
