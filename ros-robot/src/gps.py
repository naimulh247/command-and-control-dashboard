#!/usr/bin/env python

'''
A module with a GPS node.

GPS2IP: http://www.capsicumdreams.com/gps2ip/
'''

import re
import rospy
import socket

from std_msgs.msg import String

class GPS:
    '''A node which listens to GPS2IP Lite through a socket and publishes a GPS topic.'''
    def __init__(self):
        '''Initialize the publisher and instance variables.'''
        # Instance Variables
        self.HOST = rospy.get_param('~HOST')
        self.PORT = rospy.get_param('~PORT')

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
        gps_keys = ['message_id', 'utc_time', 'status', 'latitude', 'ns_indicator', 'longitude', 'ew_indicator', 'speed_over_ground', 'course_over_ground', 'date', 'magnetic_variation', 'mode', 'checksum']
        gps_values = re.split(',|\*', gps_data.decode())
        gps_dict = dict(zip(gps_keys, gps_values))

        # Remove unnecessary values
        del gps_dict['checksum']

        # Publish the decoded GPS data
        self.publisher.publish(f"'{str(gps_dict)}'")


if __name__ == '__main__':
    # Initialize a ROS node named GPS
    rospy.init_node("gps")

    # Initialize a GPS instance with the HOST and PORT
    gps_node = GPS()

    # Continuously publish coordinated until shut down
    while not rospy.is_shutdown():
        gps_node.get_coords()