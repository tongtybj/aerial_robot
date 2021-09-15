#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from spinal.msg import FlightConfigCmd, FourAxisCommand, UavInfo, PwmInfo, RollPitchYawTerm, RollPitchYawTerms
from aerial_robot_control.cfg import PDGainConfig
from dynamic_reconfigure.server import Server


class QuadrotorAttitudeTest(object):

    def __init__(self):

        self.base_throttle = rospy.get_param('~base_throttle', 1500) # PWM
        self.max_angle = rospy.get_param('~max_angle', 0.4) # rad
        self.min_pwm = rospy.get_param('~min_pwm', 0.55) # = pwm / 2000
        self.max_pwm = rospy.get_param('~max_pwm', 0.8) # = max / 2000
        self.motor_num = rospy.get_param('~motor_num', 4) # rad
        self.p_gain = rospy.get_param('~p_gain', 400.0)
        self.i_gain = rospy.get_param('~i_gain', 200.0)
        self.d_gain = rospy.get_param('~p_gain', 100.0)
        self.yaw_p_gain = rospy.get_param('~yaw_p_gain', 300.0)
        self.yaw_i_gain = rospy.get_param('~yaw_d_gain', 300.0)
        self.yaw_d_gain = rospy.get_param('~yaw_d_gain', 300.0)
        self.motor_arm = False
        self.motor_cmd_pub = rospy.Publisher('flight_config_cmd', FlightConfigCmd, queue_size = 1)
        self.flight_cmd_pub = rospy.Publisher('four_axes/command', FourAxisCommand, queue_size = 1)
        self.uav_info_pub = rospy.Publisher('uav_info', UavInfo, queue_size = 1)
        self.pwm_info_pub = rospy.Publisher('motor_info', PwmInfo, queue_size = 1)
        self.rpy_gain_pub = rospy.Publisher('rpy/gain', RollPitchYawTerms, queue_size = 1)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joyCallback)
        self.roll_target_angle = 0
        self.pitch_target_angle = 0

        rospy.sleep(5.0) # work around to wait for spinal_ros_bridge
        rospy.loginfo("publish uav and motor info to spinal")

        # send UAV info
        uav_info_msg = UavInfo(motor_num = 4, uav_model = UavInfo.DRONE, baselink = 0)
        self.uav_info_pub.publish(uav_info_msg)
        rospy.sleep(0.1)
        # send PWM info
        pwm_info_msg = PwmInfo(min_pwm = self.min_pwm, max_pwm = self.max_pwm, min_thrust = 0, pwm_conversion_mode = 2)
        self.pwm_info_pub.publish(pwm_info_msg)

        self.pd_gain_cfg_srv = Server(PDGainConfig, self.PDGainCfgCallback)

    def PDGainCfgCallback(self, config, lelvel):
        self.p_gain = config.p_gain
        self.d_gain = config.d_gain

        return config

    def sendPIDGains(self):
        # send rpy gain
        gains_msgs = RollPitchYawTerms()

        if self.motor_num != 4:
            return

        for i in range(self.motor_num):
            motor_gain = RollPitchYawTerm()
            if i == 0: # rear right
                roll_sign = -1
                pitch_sign = 1
                yaw_sign = 1
            if i == 1: # front right
                roll_sign = -1
                pitch_sign = -1
                yaw_sign = -1
            if i == 2: # front left
                roll_sign =  1
                pitch_sign = -1
                yaw_sign = 1
            if i == 3: # rear left
                roll_sign = 1
                pitch_sign = 1
                yaw_sign = -1

            motor_gain.roll_p = roll_sign * self.p_gain
            motor_gain.roll_i = roll_sign * self.i_gain
            motor_gain.roll_d = roll_sign * self.d_gain
            motor_gain.pitch_p = pitch_sign * self.p_gain
            motor_gain.pitch_i = pitch_sign * self.i_gain
            motor_gain.pitch_d = pitch_sign * self.d_gain
            motor_gain.yaw_p = yaw_sign * self.yaw_p_gain
            motor_gain.yaw_i = yaw_sign * self.yaw_i_gain
            motor_gain.yaw_d = yaw_sign * self.yaw_d_gain

            gains_msgs.motors.append(motor_gain)

            # TODO: check whether need P and I control for yaw motion
        self.rpy_gain_pub.publish(gains_msgs)



    def joyCallback(self, msg):
        if msg.buttons[7] == 1: # R2
            if not self.motor_arm:
                # start arming motor
                self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.ARM_ON_CMD))
                # send start i control cmd
                rospy.sleep(0.1) 
                self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.INTEGRATION_CONTROL_ON_CMD))

            self.motor_arm = True
        else:
            if self.motor_arm:
                # stop motor
                self.motor_cmd_pub.publish(FlightConfigCmd(FlightConfigCmd.ARM_OFF_CMD))
            self.motor_arm = False

        self.roll_target_angle = -msg.axes[0]  * self.max_angle
        self.pitch_target_angle = msg.axes[1] * self.max_angle


    def main(self):
        r = rospy.Rate(40) # 40hz
        time = rospy.get_time()

        if self.motor_num != 4:
            rospy.logerr("we only support quadrotor");
            return

        while not rospy.is_shutdown():
            # send control gains
            if rospy.get_time() - time > 1:
                time = rospy.get_time()
                self.sendPIDGains()

            # send flight command
            cmd_msg = FourAxisCommand()
            cmd_msg.angles = [self.roll_target_angle, self.pitch_target_angle, 0]
            cmd_msg.base_thrust = [self.base_throttle] * self.motor_num
            self.flight_cmd_pub.publish(cmd_msg)
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('moltirotor_att_test')
    test = QuadrotorAttitudeTest()
    test.main()
