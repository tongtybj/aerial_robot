#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import argparse
import numpy as np

class LinearJointMotion:
    def __init__(self, freq):

        self.joint_seq = []
        self.index = 0

        self.pub = rospy.Publisher('/spidar/joints_ctrl', JointState, queue_size = 1)
        self.timer = rospy.Timer(rospy.Duration(1/freq), self.timerCallback)

    def start(self, final_joint_angles, duration):

        start_joint_state = rospy.wait_for_message('/spidar/joint_states', JointState, timeout=5)
        self.joint_names = ['joint' + str(i//2+1) + ('_yaw' if i % 2 == 0 else '_pitch')  \
                       for i in range(len(final_joint_angles))]

        seq_len = int(duration / self.timer._period.to_sec())

        if seq_len > 1:
            for angle, name in zip(final_joint_angles, self.joint_names):
                current_angle = start_joint_state.position[start_joint_state.name.index(name)]
                self.joint_seq.append(np.linspace(current_angle, float(angle), num = seq_len, endpoint = True))
            self.joint_seq = np.stack(self.joint_seq).transpose()
        else:
            self.joint_seq = [final_joint_angles]

        self.index = 0


    def timerCallback(self, event):

        if len(self.joint_seq) == 0:
            return

        joint_msg = JointState()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.position = self.joint_seq[self.index]
        joint_msg.name = self.joint_names
        self.pub.publish(joint_msg)

        self.index += 1

        if self.index == len(self.joint_seq):
            self.joint_seq = []


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-q', nargs='+', help='joint angle [rad]*16')
    parser.add_argument('-t', type=float, help='execute time [s]')
    parser.add_argument('-f', type=float, help='joint publish frequency [Hz]', default='20.0')
    args = parser.parse_args()

    if len(args.q) != 16 and len(args.q) != 4:
        print("[Error] incorrect number of joint angle, please assign angle with order of (joint_yaw, joint_pitch)")
        exit(0)

    if len(args.q) == 4:
        args.q = args.q * 4

    args.q = list(map(float, args.q))
    print("taget joint angles are {}".format(args.q))

    rospy.init_node('linear_joint_motion')
    node = LinearJointMotion(args.f)
    node.start(args.q, args.t)
    rospy.spin()
