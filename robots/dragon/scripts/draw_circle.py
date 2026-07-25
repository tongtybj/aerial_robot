#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf2_ros
from std_msgs.msg import Bool
from std_msgs.msg import UInt8
from geometry_msgs.msg import Point
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import tf2_geometry_msgs

def odom_callback(msg):
    global cog2world
    cog2world = msg

def wrench_callback(msg):
    global external_wrench
    external_wrench = msg

def joint_callback(msg):
    global joint_states
    joint_states = msg



if __name__ == "__main__":

    rospy.init_node("push_and_slide")
    cog2world = Odometry()

    external_wrench = WrenchStamped()
    joint_states = JointState()

    external_wrench_added = WrenchStamped()

    link_num = rospy.get_param("~link_num", 4)
    duration = rospy.get_param("~duration", 0.005)
    joint_pub = rospy.Publisher("/dragon/joints_ctrl", JointState, queue_size=1)
    plan_pub = rospy.Publisher("/dragon/plan_start", Bool, queue_size=1)
    mode_pub = rospy.Publisher("/dragon/pos_mode", UInt8, queue_size=1)
    odom_sub = rospy.Subscriber("/dragon/uav/cog/odom", Odometry, odom_callback)

    nav_pub = rospy.Publisher("/dragon/uav/nav", FlightNav, queue_size=1)
    trajectory_pub = rospy.Publisher("/dragon/ee_pos", PointStamped, queue_size=1)


    time.sleep(1)

    mode_msg = UInt8()
    mode_msg.data = 2
    mode_pub.publish(mode_msg)
    print("Mode change to be-centric")
    time.sleep(3)
  
    nav_msg = FlightNav()
    nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
    nav_msg.target_pos_x = -0.60
    nav_msg.target_pos_y = 0.48

    nav_msg.pos_z_nav_mode = 4 
    nav_msg.target_pos_z = 1.2

    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_yaw = -math.pi / 2

    nav_pub.publish(nav_msg)
    print("Fly to initial pose [", nav_msg.target_pos_x, ",", nav_msg.target_pos_y, ",", nav_msg.target_pos_z, ",", nav_msg.target_yaw, "]"    )
    time.sleep(5)

    joints = JointState()
    joints.name = ["joint1_yaw", "joint2_yaw", "joint3_yaw"]
    joints.position = [math.pi/3, math.pi/3, -math.pi/6]
    joint_pub.publish(joints)
    print("Change to initial joint configuration")
    time.sleep(7)
    nav_msg = FlightNav()
    nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
    nav_msg.target_pos_x = -0.42
    nav_msg.target_pos_y = 0.48

    nav_msg.pos_z_nav_mode = 4 
    nav_msg.target_pos_z = 1.2

    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_yaw = -math.pi / 2

    nav_pub.publish(nav_msg)
    time.sleep(3)

    ee_pos_msg = PointStamped()

    round = 0
    time.sleep(1)

    while not rospy.is_shutdown():
        round += 1
        if round > 6400:
            round -= 6400
        # nav_msg = FlightNav()
        # nav_msg.pos_xy_nav_mode = 4 # pos_vel mode
        # nav_msg.target_pos_x = -0.35
        # nav_msg.target_pos_y = 0.0 - 0.2 * math.sin(math.pi * round / 1600)

        # nav_msg.pos_z_nav_mode = 4 
        # nav_msg.target_pos_z = 1.2 + 0.2 * math.cos(math.pi * round / 1600)

        # nav_msg.yaw_nav_mode = 4 
        # nav_msg.target_yaw = 0.0

        # nav_pub.publish(nav_msg)
     
        ee_pos_msg = PointStamped()
        ee_pos_msg.point.x = 0.528 + 0.2 * math.sin(math.pi * round / 1600)
        #ee_pos_msg.point.x = 0.428 + 0.2 * math.sin(math.pi * round / 1600)
        ee_pos_msg.point.y = 1.38
        ee_pos_msg.point.z = 0.00 + 0.2 * math.cos(math.pi * round / 1600)
        if round < 4400 and round > 400: 

            ee_pos_msg.point.x = 0.528 + 0.2 * math.sin(math.pi * (round + 1600) / 8000)
            #ee_pos_msg.point.x = 0.428 + 0.2 * math.sin(math.pi * round / 1600)
            ee_pos_msg.point.y = 1.42
            ee_pos_msg.point.z = 0.00 + 0.2 * math.cos(math.pi * (round + 1600) / 8000)
        if round  >= 4400: 
            ee_pos_msg.point.x = 0.528 + 0.2 * math.sin(math.pi * (round - 3200) / 1600)
            #ee_pos_msg.point.x = 0.428 + 0.2 * math.sin(math.pi * round / 1600)
            ee_pos_msg.point.y = 1.42
            ee_pos_msg.point.z = 0.00 + 0.2 * math.cos(math.pi * (round - 3200) / 1600)
   
        trajectory_pub.publish(ee_pos_msg)

        if round == 10:
            plan_msg = Bool()
            plan_msg.data = True
            plan_pub.publish(plan_msg)
            print("Begin to plan")
        time.sleep(duration)


