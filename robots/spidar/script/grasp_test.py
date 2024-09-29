#!/usr/bin/env python

import rospy
import argparse
import numpy as np
from spidar_joint_motion import LinearJointMotion
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest
from geometry_msgs.msg import Transform, Inertia, Vector3, Quaternion
from sensor_msgs.msg import Joy

class GraspTest:

    def __init__(self):

        ## object configuration
        self.object_mass = rospy.get_param("~object_mass", 1.0)
        self.object_radius = rospy.get_param("~object_radius", 0.25)
        self.object_center = rospy.get_param("~object_center", [0, 0, -0.77])

        self.pregrasp_joint_angles = rospy.get_param("~pregrasp_joint_angles", [0, 0.0, 0, 1.57])
        if len(self.pregrasp_joint_angles) == 4:
            self.pregrasp_joint_angles = self.pregrasp_joint_angles * 4
        self.grasp_joint_angles = rospy.get_param("~grasp_joint_angles", [0, 0.8, 0, 1.57])
        if len(self.grasp_joint_angles) == 4:
            self.grasp_joint_angles = self.grasp_joint_angles * 4

        self.pregrasp_duration = rospy.get_param("~pregrasp_duration", 8.0)
        self.wait_duration = rospy.get_param("~wait_duration", 2.0)
        self.grasp_duration = rospy.get_param("~grasp_duration", 8.0)

        self.release_duration = rospy.get_param("~release_duration", 8.0)

        self.separate_mode = rospy.get_param("~separate_mode", False)


        joint_ctrl_freq = rospy.get_param("~joint_ctrl_freq", 20.0)
        self.joint_node = LinearJointMotion(joint_ctrl_freq)


        self.joy_buttons = None
        self.prev_joy_buttons = None
        self.joy_axes = None
        self.prev_joy_axes = None

        self.sub = rospy.Subscriber('/spidar/joy', Joy, self.joyCallback)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.eventCallback)

    def graspJointMotion(self):


        self.joint_node.start(self.pregrasp_joint_angles, self.pregrasp_duration) # move the pre-grasp pose

        # sleep to make sure the robot has reached the pre-grasp pose
        rospy.sleep(self.pregrasp_duration + self.wait_duration)
        rospy.loginfo("conplete the pre-grasp pose");

        self.joint_node.start(self.grasp_joint_angles, self.grasp_duration) # move the grasp pose

        if not self.separate_mode:
            grasp_wait_duration = 1.0
            rospy.sleep(self.grasp_duration + grasp_wait_duration)

            rospy.loginfo("add grasping object to the aerial robot");
            self.graspObject()


    def releaseJointMotion(self):

        release_joint_angles = [0] * 16
        self.joint_node.start(release_joint_angles, self.release_duration)

    def graspObject(self):
        # rosserice call
        try:
            add_object_client = rospy.ServiceProxy('/spidar/add_extra_module', AddExtraModule)

            req = AddExtraModuleRequest()
            req.action = req.ADD
            req.module_name= "grasp_object"
            req.parent_link_name= "center_link"
            req.transform.translation = Vector3(*self.object_center)
            req.transform.rotation.w = 1
            req.inertia.m = self.object_mass
            req.inertia.ixx = 0.4 * self.object_mass * self.object_radius * self.object_radius
            req.inertia.iyy = 0.4 * self.object_mass * self.object_radius * self.object_radius
            req.inertia.izz = 0.4 * self.object_mass * self.object_radius * self.object_radius
            req.size = Vector3(self.object_radius, 0, 0)

            add_object_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)


    def releaseObject(self):
        # rosserice call

        try:
            remove_object_client = rospy.ServiceProxy('/spidar/add_extra_module', AddExtraModule)

            req = AddExtraModuleRequest()
            req.action = req.REMOVE
            req.module_name= "grasp_object"

            remove_object_client(req)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)


    def eventCallback(self, event):

        if self.joy_buttons is None or self.joy_axes is None:
            return

        if self.joy_buttons[4] == 1: # L1

            # change the joint angles to grasp
            if self.joy_axes[9] == 1: # CROSS LEFT

                if self.prev_joy_axes[9] != 1: # only once
                    rospy.loginfo("change the joint angles to grasp");
                    self.graspJointMotion()

            # change the joitn angles to release
            if self.joy_axes[9] == -1: # CROSS RIGHT

                if self.prev_joy_axes[9] != -1: # only once
                    rospy.loginfo("change the joint angles to release, and remove object");
                    self.releaseObject() # in case that object is still graspped
                    self.releaseJointMotion()

        if self.joy_buttons[6] == 1: # L2

            # add grasping object to the aerial robot
            if self.joy_axes[9] == 1: # CROSS LEFT

                if self.prev_joy_axes[9] != 1: # only once
                    rospy.loginfo("add grasping object to the aerial robot");
                    self.graspObject()

            # remove grasping object from the aerial robot
            if self.joy_axes[9] == -1: # CROSS RIGHT

                if self.prev_joy_axes[9] != -1: # only once
                    rospy.loginfo("remove grasping object from the aerial robot");
                    self.releaseObject()


        self.prev_joy_buttons = self.joy_buttons
        self.prev_joy_axes = self.joy_axes


    def joyCallback(self, msg):
        self.joy_buttons = msg.buttons
        self.joy_axes = msg.axes

        if self.prev_joy_buttons is None:
            self.prev_joy_buttons = self.joy_buttons
            self.prev_joy_axes = self.joy_axes


if __name__ == '__main__':

    rospy.init_node('grasp_test')
    node = GraspTest()
    rospy.spin()
