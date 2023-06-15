#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovariance
class CircTrajFollow():
  def __init__(self):
    self.linear_move_pub = rospy.Publisher("/quadrotor/uav/nav", FlightNav, queue_size=1)
    self.linear_move_sub = rospy.Subscriber("/quadrotor/uav/cog/odom", Odometry, self.odomCb)
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.rate = 1.0/20.0 #20Hz

    self.flag = False
    time.sleep(0.5)

  def odomCb(self, msg):
      now_z = msg.pose.pose.position.z
      # rospy.loginfo(now_z)
      if now_z > 2.0:
          self.flag = True
      else:
          self.flag = False
          #write processes based on robot's position

  def main(self):

    while not rospy.is_shutdown():
        #write publisher
        fn = FlightNav()
        fn.pos_z_nav_mode = 2
        if self.flag:
            rospy.loginfo("descend")
            fn.target_pos_z = 1
        # else:
        #     rospy.loginfo("ascend")
        #     fn.target_pos_z = 3  ##code for oscillate
        self.linear_move_pub.publish(fn)
        time.sleep(self.rate)
        #rate.sleep()


if __name__ == "__main__":

  rospy.init_node("linear_move_oscillation")

  Tracker = CircTrajFollow()
  Tracker.main()



