#!/usr/bin/env python

'''
A module for a node that alters the quality of the image.
'''

import cv2
import numpy as np
import re
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class Compressor:
    ''''''
    def __init__(self):
        '''Initialize necessary publishers and subscribers.'''
        # Instance Variables
        # Initialize an object that converts OpenCV Images and ROS Image Messages
        self.cv_bridge = CvBridge()
        # Image Resolution
        self.img_res = {
            'height': 1080,
            'width': 1920
        }

        # Publisher - https://answers.ros.org/question/66325/publishing-compressed-images/
        self.publisher = rospy.Publisher('/camera/rgb/image_res/compressed', CompressedImage, queue_size=1)
        
        # Subscribers
        self.subscriber_cam = rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, self.callback_cam, queue_size=1)
        self.subscriber_set = rospy.Subscriber('/image_configs', String, self.callback_set, queue_size=1)


    def callback_cam(self, msg):
        '''A callback that resizes the image in line with the specified resolution.'''
        # Convert CompressedImage to OpenCV
        img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)

        # Apply New Resolution
        img = cv2.resize(img, (self.img_res['height'], self.img_res['width']))

        # Convert OpenCV to CompressedImage
        msg_new = self.cv_bridge.cv2_to_compressed_imgmsg(img)

        # Publish
        self.publisher.publish(msg_new)

    def callback_set(self, msg):
        '''A callback to retrieve the specified resolution from the web client.'''
        img_set = re.split(',', msg.data)

        self.img_res['height'] = img_set[0]
        self.img_res['width'] = img_set[1]


if __name__ == "__main__":
    rospy.init_node("img_comp")
    comp = Compressor()
    rospy.spin()