#!/usr/bin/env python

import rospy
from spinal.msg import ServoStates
from std_msgs.msg import Empty
import numpy as np

class TigerJointLoadChecker:
    def __init__(self):
        rospy.init_node('tiger_joint_load_check')
        self.load_thresh = rospy.get_param('~load_thresh', 350)
        self.pub = rospy.Publisher('/tiger/teleop_command/land', Empty, queue_size = 1)
        self.sub = rospy.Subscriber('/tiger/servo/states', ServoStates, self.servoStateCallback)
        self.land_flag = False

    def servoStateCallback(self, msg):
        servo_loads = np.array([s.load for s in msg.servos])
        servo_max = np.abs(servo_loads).max()
        if servo_max >= self.load_thresh:
            rospy.logwarn_throttle(0.5, "servo overload!!! {}".format(servo_loads))

            if not self.land_flag:
                rospy.loginfo("send land because of servo overload")
                self.pub.publish(Empty())
                self.land_flag = True

        # rospy.loginfo_throttle(1.0, "servo load: {}".format(servo_loads))

if __name__ == '__main__':
    checker = TigerJointLoadChecker()
    rospy.spin()

