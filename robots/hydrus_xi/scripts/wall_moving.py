#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf2_ros
import numpy as np
from std_msgs.msg import UInt8
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs


if __name__ == "__main__":

    rospy.init_node("wall_moving")
    model_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
   
 
    initialized = False
    round = 0
    while not rospy.is_shutdown():

        pose = Pose()
        twist = Twist()

        pose.position.x = -0.15 * math.sin(math.pi * round / 100)
        pose.position.y = 0.0
        pose.position.z = 0.0
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0

        twist.linear.x = 0.15 * math.pi / 6 * math.cos(math.pi * round / 100)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        model_state = ModelState()
        model_state.model_name = 'opening'
        model_state.pose = pose
        model_state.twist = twist
        model_state.reference_frame = 'world'

        response = model_srv(
        model_state=model_state, 
        )
        round += 1
 
        time.sleep(0.05)


