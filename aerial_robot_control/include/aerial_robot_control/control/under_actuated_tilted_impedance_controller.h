// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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

#include <aerial_robot_control/control/under_actuated_impedance_controller.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_control
{
  class UnderActuatedTiltedImpedanceController: public UnderActuatedImpedanceController
  {
  public:
    UnderActuatedTiltedImpedanceController() {}
    virtual ~UnderActuatedTiltedImpedanceController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:

    ros::Publisher desired_baselink_rot_pub_;
    ros::Subscriber contact_flag_sub_;

    std::mutex wrench_mutex_;
    Eigen::VectorXd est_external_wrench_;
    Eigen::VectorXd est_external_wrench_clamped_;
    Eigen::VectorXd target_wrench_cog_;
    Eigen::VectorXd tao_;
    double omega_x_, omega_y_, omega_z_;

    double target_acc_x_, target_acc_y_, target_acc_z_;
    double mdx_, mdy_, mdz_, Idx_, Idy_, Idz_;
    double x_y_p_, z_p_, roll_pitch_p_, yaw_p_, joints_p_, pos_p_, x_y_zeta_, z_zeta_, roll_pitch_zeta_, yaw_zeta_, joints_d_, pos_d_;
    
    bool contact_flag_ = false;
    int contact_count_ = 0;
    int contact_control_ = 0;

    const Eigen::VectorXd getTargetWrenchCog()
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      return target_wrench_cog_;
    }
    void setTargetWrenchCog(const Eigen::VectorXd target_wrench_cog)
    {
      std::lock_guard<std::mutex> lock(wrench_mutex_);
      target_wrench_cog_ = target_wrench_cog;
    }
    double z_limit_;
    void clampEstExternalWrench();
    void sendFourAxisCommand() override;
    void controlCore() override;
    bool optimalGain() override;
    void publishGain() override;
    void rosParamInit() override;
    void contactFlagCallback(const std_msgs::Empty msg);
  };
};
