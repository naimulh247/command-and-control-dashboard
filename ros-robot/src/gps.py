#!/usr/bin/env python

'''
A module with a GPS node.

GPS2IP: http://www.capsicumdreams.com/gps2ip/
'''

import json
import re
import rospy
import socket

from std_msgs.msg import String

class GPS:
    '''A node which listens to GPS2IP Lite through a socket and publishes a GPS topic.'''
    def __init__(self):
        '''Initialize the publisher and instance variables.'''
        # Instance Variables
        self.HOST = rospy.get_param('~HOST', '172.20.38.175')
        self.PORT = rospy.get_param('~PORT', 11123)

        # Publisher
        self.publisher = rospy.Publisher('/gps', String, queue_size=1)

    def get_coords(self):
        '''A method to receive the GPS coordinates from GPS2IP Lite.'''
        # Instantiate a client object
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.HOST, self.PORT))
            # The data is received in the RMC data format
            gps_data = s.recv(1024)

        # Transform data into dictionary
        gps_keys = ['message_id', 'latitude', 'ns_indicator', 'longitude', 'ew_indicator']
        gps_values = re.split(',|\*', gps_data.decode())[:5]
        gps_dict = dict(zip(gps_keys, gps_values))

        # Cleanse the coordinate data
        for key in ['latitude', 'longitude']:
            # Identify the presence of a negative number indicator
            neg_num = False

            # The GPS2IP application transmits a negative coordinate with a zero prepended
            if gps_dict[key][0] == '0':
                neg_num = True
            
            # Transform the longitude and latitude into a format that can be utilized by the front-end web-client
            gps_dict[key] = float(gps_dict[key]) / 100

            # Apply the negative if the clause was triggered
            if neg_num:
                gps_dict[key] = -1 * gps_dict[key]

        # Publish the decoded GPS data
        self.publisher.publish(json.dumps(gps_dict))


if __name__ == '__main__':
    # Initialize a ROS node named GPS
    rospy.init_node("gps")

    # Initialize a GPS instance with the HOST and PORT
    gps_node = GPS()

    # Continuously publish coordinated until shut down
    while not rospy.is_shutdown():
        gps_node.get_coords()
