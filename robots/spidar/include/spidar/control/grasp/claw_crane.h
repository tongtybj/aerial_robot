// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2025, DRAGON Lab
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

#include <aerial_robot_base/extra_plugin.h>
#include <dragon/model/full_vectoring_robot_model.h>
#include <dragon/control/full_vectoring_control.h>
#include <dragon/dragon_navigation.h>
#include <algorithm>
#include <thread>

using DragonRobotModel = Dragon::FullVectoringRobotModel;
using DragonController = aerial_robot_control::DragonFullVectoringController;
using DragonNavigator  = aerial_robot_navigation::DragonNavigator;
using DragonRobotModelPtr = boost::shared_ptr<DragonRobotModel>;
using DragonControllerPtr = boost::shared_ptr<DragonController>;
using DragonNavigatorPtr  = boost::shared_ptr<DragonNavigator>;

namespace extra_plugin
{
  namespace grasp
    {
     class ClawCrane: public Base
     {
     public:
       ClawCrane();
       ~ClawCrane() override {}

       void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                       RobotModelPtr robot_model,
                       EstimatorPtr  estimator,
                       ControllerPtr controller,
                       NavigatorPtr  navigator) override;

       void update() override;

     private:
       DragonRobotModelPtr dragon_robot_model_;
       DragonControllerPtr dragon_controller_;
       DragonNavigatorPtr  dragon_navigator_;

       ros::Publisher extra_thrust_force_pub_;
       ros::Publisher total_joint_torque_pub_;
       std::thread thread_;
       void threadFunc();

       double plan_rate_;
       double grasp_force_;
       double thrust_force_weight_, joint_torque_weight_;
       double deform_thresh_;

       double default_mass_;
       bool grasp_flag_;
       double nominal_gripper_dist_;

       double calculateGripperDistance();
       void thrustControl();
       void optimizeGraspForce(const Eigen::MatrixXd& A1_fr, const Eigen::MatrixXd& A2_fr, \
                               const Eigen::VectorXd& extra_joint_torque, Eigen::VectorXd& extra_thrust);
     };
    };
};
