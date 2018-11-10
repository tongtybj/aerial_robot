// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <aerial_robot_base/control/flatness_pid_controller.h>
#include <aerial_robot_model/eigen_utils.h>
#include <dragon/dragon_robot_model.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <spinal/DesireCoord.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>

namespace control_plugin
{
  class DragonGimbalPitchOnly : public control_plugin::FlatnessPid
  {
  public:
    DragonGimbalPitchOnly();
    ~DragonGimbalPitchOnly(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    BasicEstimator* estimator, Navigator* navigator,
                    double ctrl_loop_rate);
    bool update();
    void reset()
    {
      FlatnessPid::reset();

      yaw_i_term_.assign(1, 0);
      target_yaw_.assign(1, 0);
    }

    void sendCmd();

  private:
    std::unique_ptr<DragonRobotModel> kinematics_;
    ros::Publisher gimbal_control_pub_;
    ros::Publisher joint_control_pub_;
    ros::Publisher gimbal_target_force_pub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber desire_tilt_sub_;

    void gimbalControl();
    void jointStateCallback(const sensor_msgs::JointStateConstPtr& state);
    void rosParamInit();

    void baselinkTiltCallback(const spinal::DesireCoordConstPtr & msg);
    void fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg);

    sensor_msgs::JointState joint_state_;
    //Eigen::MatrixXd P_xy_;

    /* desire tilt */
    std::vector<double> target_gimbal_angles_;

    bool simulation_;
    double gimbal_control_stamp_;

    bool control_verbose_;

    /* rosparam */
    string joints_torque_control_srv_name_;

    /* cfg */
    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>* yaw_pid_server_;
    dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>::CallbackType dynamic_reconf_func_yaw_pid_;
    void cfgYawPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level);
  };
};

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::DragonGimbalPitchOnly, control_plugin::ControlBase);
