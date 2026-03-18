#!/usr/bin/env python

import sys
import time
import rospy
import math
import tf2_ros
from std_msgs.msg import Empty
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
    joint_pub = rospy.Publisher("/hydrus_xi/joints_ctrl", JointState, queue_size=1)
    #odam_sub = rospy.Subscriber("/hydrus_xi/uav/cog", Odometry, odam_callback)
    mode_pub = rospy.Publisher("/hydrus_xi/imp_mode", UInt8, queue_size=1)
    wrench_sub = rospy.Subscriber("/hydrus_xi/estimated_external_wrench", WrenchStamped, wrench_callback)
    joint_sub = rospy.Subscriber("/hydrus_xi/joint_states", JointState, joint_callback)
    odom_sub = rospy.Subscriber("/hydrus_xi/uav/cog/odom", Odometry, odom_callback)
   
    wrench_pub = rospy.Publisher("/hydrus_xi/external_wrench", WrenchStamped, queue_size=1)
    pos_pub = rospy.Publisher("/hydrus_xi/pos_cmds", Point, queue_size=1)
    nav_pub = rospy.Publisher("/hydrus_xi/uav/nav", FlightNav, queue_size=1)

    pos_mode_pub = rospy.Publisher("/hydrus_xi/pos_mode", UInt8, queue_size=1)


    contact_pub = rospy.Publisher("/hydrus_xi/contact_flag", Empty, queue_size=1)
    time.sleep(1)
    force_x = 0.0
#[-0.07, 0.79, -0.39]
#[1.0, 1.5, -0.59]
    contact_msg = Empty()

    mode = UInt8()
    contact_msg = Empty()
    mode.data = 1
    joints = JointState()
    joints.position = [math.pi/3, math.pi/3, -math.pi/6]
    nav_msg = FlightNav()
    pos_mode = UInt8()
    pos_mode.data = 0
    pos_mode_pub.publish(pos_mode)

    #joint_pub.publish(joints)
   

    print("preparation 1")
    time.sleep(1)

    nav_msg.pos_xy_nav_mode = 4 # pos_vel mode


    nav_msg.target_pos_x = -1.40
    nav_msg.target_vel_x = 0.0
    nav_msg.target_pos_y = 0.0
    nav_msg.target_vel_y = 0.0

    nav_msg.pos_z_nav_mode = 4 
    nav_msg.target_pos_z = 1.2
    nav_msg.target_vel_z = 0.05
    nav_msg.yaw_nav_mode = 4 
    nav_msg.target_yaw = -joint_states.position[5] - joint_states.position[6]
    nav_msg.target_omega_z = -0.05

    nav_pub.publish(nav_msg)

    time.sleep(3)

   # time.sleep(6)
    print("preparation 2")

    for i in range(10):
        nav_msg = FlightNav()
        nav_msg.pos_xy_nav_mode = 4
        nav_msg.target_pos_x = -1.40 + (0.25/10) * (i+1)

        nav_msg.yaw_nav_mode = 4 
        nav_msg.target_yaw = -joint_states.position[5] - joint_states.position[6]
        nav_msg.pos_z_nav_mode = 4 
        nav_msg.target_pos_z = 1.2
        nav_msg.target_vel_z = 0.0
        nav_pub.publish(nav_msg)

        time.sleep(0.5)
    print("preparation 3")
    round = 0
    time.sleep(5)
    pos_mode.data = 2
    pos_mode_pub.publish(pos_mode)
    while not rospy.is_shutdown():
        round += 1
     

        nav_msg = FlightNav()
     
       
        filted_force_x = external_wrench.wrench.force.x * 0.1 + force_x * 0.9
        nav_msg.pos_xy_nav_mode = 4

        nav_msg.target_pos_x = -1.75
        #nav_msg.target_pos_x = -1.20 + (external_wrench.wrench.force.x + 0.6)/0.5*0.05
        nav_msg.target_vel_x = 0.0
        nav_msg.target_pos_y = 0.3 * math.sin(math.pi * round / 1200) + 0.6
        nav_msg.target_vel_y = 0.05 * math.pi * math.cos(math.pi * round / 1200)

        #nav_msg.target_acc_y = -0.048 * math.pi * math.pi * math.cos(math.pi * round / 50)
        nav_msg.pos_z_nav_mode = 4 
        # nav_msg.target_pos_z = 0.9
        # nav_msg.target_vel_z = 0.0

        nav_msg.target_pos_z = 0.9 + 0.3 * math.cos(math.pi * round / 1200)
        nav_msg.target_vel_z = -0.05 * math.pi * math.sin(math.pi * round / 1200)

        #nav_msg.target_acc_z = -0.048 * math.pi * math.pi * math.sin(math.pi * round / 50)
        nav_msg.yaw_nav_mode = 4 
        nav_msg.target_yaw = -joint_states.position[5] - joint_states.position[6]
        #nav_msg.target_yaw = -0.445
        #print(cog2world)
        nav_pub.publish(nav_msg)
        #contact_pub.publish(contact_msg)

        # external_wrench_added.wrench.torque.z = 0.4 * math.cos(round / 50)
        # external_wrench_added.wrench.force.y = 0.6 * math.cos(round / 50)
        #wrench_pub.publish(external_wrench_added)
        time.sleep(duration)


