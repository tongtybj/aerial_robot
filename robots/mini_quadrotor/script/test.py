#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry

class CircTrajFollow():
  def __init__(self):
    self.linear_move_pub = rospy.Publisher("/quadrotor/uav/nav", FlightNav, queue_size=1)
    self.linear_move_sub = rospy.Subscriber("/quadrotor/uav/cog/odom", Odometry, self.odomCb)
    self.max_height = rospy.get_param("~max_height", 2.0)
    self.min_height = rospy.get_param("~min_height", 1.0)
    self.ascend_speed = rospy.get_param("~ascend_speed", 0.01)
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.rate = 1.0/20.0 #20Hz

    self.flag = False
    time.sleep(0.5)

  def odomCb(self, msg):
      now_z = msg.pose.pose.position.z
      # rospy.loginfo(now_z)
      if now_z > self.max_height:
          self.flag = True

          # finish sbuscribe
          self.linear_move_sub.unregister()
      else:
          self.flag = False
          #write processes based on robot's position

  def main(self):

    while not rospy.is_shutdown():
        #write publisher
        msg = FlightNav()

        if self.flag:
            rospy.loginfo("descend")
            target_alt = self.min_height
            msg.pos_z_nav_mode = msg.POS_MODE
            msg.target_pos_z = target_alt
        else:
            rospy.loginfo("ascend")
            msg.pos_z_nav_mode = msg.VEL_MODE
            msg.target_pos_diff_z = self.ascend_speed
        self.linear_move_pub.publish(msg)
        time.sleep(self.rate)
        #rate.sleep()


if __name__ == "__main__":

  rospy.init_node("linear_move_oscillation")

  Tracker = CircTrajFollow()
  Tracker.main()



