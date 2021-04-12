#!/usr/bin/python

import rospy

from rospy.numpy_msg import numpy_msg

from geometry_msgs.msg import Twist

import tf2_ros
import tf2_geometry_msgs

import numpy as np 

from model_1 import velocity_publisher

if __name__ == '__main__':

    rospy.init_node("vel_publisher_1", anonymous=True)

    rospy.loginfo("Node init")

    velocity_publisher()

    rospy.spin()