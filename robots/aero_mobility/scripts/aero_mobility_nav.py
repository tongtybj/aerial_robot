#!/usr/bin/env python

import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
import argparse
import numpy as np
import tf

class AeroMobilityNavigation:
    def __init__(self):
        rospy.init_node('aerial_mobility_navigation')
        self.nav_msg = FlightNav()
        self.pub = rospy.Publisher('/aero_mobility/uav/nav', FlightNav, queue_size = 10)
        self.sub = rospy.Subscriber('/aero_mobility/mocap/pose', PoseStamped, self.uavStateCallback)

        rospy.sleep(0.5)

    def uavStateCallback(self, msg):
        self.pose_state = msg
        self.sub.unregister()

    def publishStateMsg(self, target_position, execute_time, frequency):

        state_seq_len = int(execute_time * frequency / 1000.0)
        state_seq = []

        if state_seq_len > 1:

            state_seq.append(np.linspace(self.pose_state.pose.position.x, float(target_position[0]), num = state_seq_len, endpoint = True))
            state_seq.append(np.linspace(self.pose_state.pose.position.y, float(target_position[1]), num = state_seq_len, endpoint = True))
            state_seq.append(np.linspace(self.pose_state.pose.position.z, float(target_position[2]), num = state_seq_len, endpoint = True))

            rpy = tf.transformations.euler_from_quaternion((self.pose_state.pose.orientation.x, self.pose_state.pose.orientation.y, self.pose_state.pose.orientation.z, self.pose_state.pose.orientation.w))

            state_seq.append(np.linspace(rpy[2], float(target_position[3]), num = state_seq_len, endpoint = True))

            state_seq = np.stack(state_seq).transpose()
        else:
            state_seq = [target_position]

        state_msg = FlightNav()

        for state in state_seq:
            state_msg.header.stamp = rospy.Time.now()
            state_msg.pos_xy_nav_mode = 2
            state_msg.pos_z_nav_mode = 2
            state_msg.yaw_nav_mode = 2
            state_msg.target_pos_x = state[0]
            state_msg.target_pos_y = state[1]
            state_msg.target_pos_z = state[2]
            state_msg.target_yaw = state[3]
            self.pub.publish(state_msg)
            rospy.sleep(1.0 / args.f)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', nargs='+', help='target pose (x, y, z, yaw)')
    parser.add_argument('-t', type=int, help='execute time [ms]', default='20000')
    parser.add_argument('-f', type=float, help='publish frequency [Hz]', default='20.0')
    args = parser.parse_args()

    if len(args.p) != 4:
        print("[Error] incorrect number of target pose, please assign angle with order of (x, y, z, yaw)")
        exit(0)

    print("taget pose is {}".format(args.p))

    node = AeroMobilityNavigation()
    node.publishStateMsg(args.p, args.t, args.f)
