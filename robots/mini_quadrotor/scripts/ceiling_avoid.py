#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class Ceilingavoid():

  def __init__(self):
    self.linear_move_pub = rospy.Publisher("/quadrotor/uav/nav", FlightNav, queue_size=1)
    self.linear_move_sub = rospy.Subscriber("/quadrotor/uav/cog/odom", Odometry, self.odomCb)
    self.max_height = rospy.get_param("~max_height", 9)#2.0
    self.min_height = rospy.get_param("~min_height", 0)#1.0
    self.ascend_speed = rospy.get_param("~ascend_speed", 0.01)
    self.avoid_distance = rospy.get_param("~avoid_distance",0.1)
    self.thresh_distance = rospy.get_param("~thresh_distance",0.2)
    self.rate = rospy.get_param("~main_rate",1.0/20.0) #20Hz
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.ceiling_avoid_sub = rospy.Subscriber("/quadrotor/ceiling/min_distance", Float32, self.doceiling_avoidCb)
    self.flag = 0
    self.now_z = 0
    # self.now_z = None

    time.sleep(0.5)

  def doceiling_avoidCb(self,msg):
    now_distance = msg.data
   # rospy.loginfo("ok!")
    if now_distance <= self.thresh_distance:
      self.flag = 2

  def odomCb(self, msg):

    self.now_z = msg.pose.pose.position.z
      # rospy.loginfo(now_z)
    if self.now_z > self.max_height:
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
          msg.pos_z_nav_mode = msg.POS_MODE
          msg.target_pos_z = self.min_height
        elif self.flag == 0:
          rospy.loginfo("ascend")
          # rospy.loginfo("thresh distance is %f",self.thresh_distance)
          msg.pos_z_nav_mode = msg.VEL_MODE
          msg.target_pos_diff_z = self.ascend_speed
          self.linear_move_pub.publish(msg)
          time.sleep(self.rate)
        elif self.flag == 2:
          rospy.loginfo("ceiling detected, ascending")
          # if self.now_z is None:
          #   continue

          self.linear_move_sub.unregister()
          target_alt =  self.now_z - self.avoid_distance
          msg.pos_z_nav_mode = msg.POS_MODE
          msg.target_pos_z = target_alt
          # rospy.loginfo("thresh distance is %f",self.thresh_distance)
          self.linear_move_pub.publish(msg)
          self.flag = 3
          # #rate.sleep()


if __name__ == "__main__":

  rospy.init_node("ceiling_avoidance")

  Tracker = Ceilingavoid()
  Tracker.main()



