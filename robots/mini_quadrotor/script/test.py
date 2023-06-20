#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
class CircTrajFollow():
  def __init__(self):
    self.linear_move_pub = rospy.Publisher("/quadrotor/uav/nav", FlightNav, queue_size=1)
    self.linear_move_sub = rospy.Subscriber("/quadrotor/uav/cog/odom", Odometry, self.odomCb)
    self.max_height = rospy.get_param("~max_height", 9)#2.0
    self.min_height = rospy.get_param("~min_height", 0)#1.0
    self.ascend_speed = rospy.get_param("~ascend_speed", 0.01)
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.rate = 1.0/20.0 #20Hz
    self.ceiling_avoid_sub = rospy.Subscriber("/quadrotor/ceiling/min_distance", Float32, self.doceiling_avoidcb)
    self.limit_distance = rospy.get_param("/distance",0.2)
    self.flag = 0
    self.avoid_distance = rospy.get_param("/avoid",0.1) 
    time.sleep(0.5)
  def doceiling_avoidcb(self,msg):
    now_distance = msg.data
    rospy.loginfo("ok!")
    if now_distance <= 0.2:
      self.flag = 2
  def odomCb(self, msg):
    now_z = msg.pose.pose.position.z
      # rospy.loginfo(now_z)
    if now_z > self.max_height:
      self.flag = 1

          # finish sbuscribe
      self.linear_move_sub.unregister()
    else:
      self.flag = 0
          #write processes based on robot's position

  def main(self):

    while not rospy.is_shutdown():
        #write publisher
        msg = FlightNav()

        if self.flag == 1:
          rospy.loginfo("descend")
          target_alt = self.min_height
          msg.pos_z_nav_mode = msg.POS_MODE
          msg.target_pos_z = target_alt
        elif self.flag == 0:
          rospy.loginfo("ascend")
          msg.pos_z_nav_mode = msg.VEL_MODE
          msg.target_pos_diff_z = self.ascend_speed
          self.linear_move_pub.publish(msg)
          time.sleep(self.rate)
        elif self.flag == 2:
          self.linear_move_sub.unregister()
          rospy.loginfo("avoid ceiling,descend")
          target_alt = 0 #now_z - self.avoid_distance 
          msg.pos_z_nav_mode = msg.POS_MODE
          msg.target_pos_z = target_alt
          self.linear_move_pub.publish(msg)
          # #rate.sleep()


if __name__ == "__main__":

  rospy.init_node("linear_move_oscillation")

  Tracker = CircTrajFollow()
  Tracker.main()



