#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

def get_ros_topics():
    """
    Returns a list of active ROS topics using the `rostopic list` command.
    """
    command = "rostopic list"
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    topics = output.decode().split("\n")[:-1] # remove empty last element
    return topics

def rostopic_list_publisher():
    """
    ROS node that publishes the list of active ROS topics as a string message.
    """
    rospy.init_node('rostopic_list_publisher', anonymous=True)
    pub = rospy.Publisher('/rostopic_list', String, queue_size=10)
    rate = rospy.Rate(1) # 1 Hz

    while not rospy.is_shutdown():
        topics = get_ros_topics()
        message = ",".join(topics)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        rostopic_list_publisher()
    except rospy.ROSInterruptException:
        pass