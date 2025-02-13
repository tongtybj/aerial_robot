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

#include <ros/ros.h>
#include <aerial_robot_control/control/base/base.h>
#include <aerial_robot_control/flight_navigation.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <aerial_robot_model/model/aerial_robot_model.h>

using RobotModelPtr = boost::shared_ptr<aerial_robot_model::RobotModel>;
using EstimatorPtr  = boost::shared_ptr<aerial_robot_estimation::StateEstimator>;
using ControllerPtr = boost::shared_ptr<aerial_robot_control::ControlBase>;
using NavigatorPtr  = boost::shared_ptr<aerial_robot_navigation::BaseNavigator>;

namespace extra_plugin
{
  class Base
  {
  public:
    Base()
    {}

    virtual ~Base(){}
    void virtual initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                            RobotModelPtr robot_model,
                            EstimatorPtr  estimator,
                            ControllerPtr controller,
                            NavigatorPtr  navigator)
    {
      nh_ = nh;
      nhp_ = nhp;

      robot_model_ = robot_model;
      estimator_ = estimator;
      controller_ = controller;
      navigator_ = navigator;

      getParam<bool>(nhp_, "param_verbose", param_verbose_, false);
    }

    virtual bool update() = 0;

  protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    RobotModelPtr robot_model_;
    EstimatorPtr  estimator_;
    ControllerPtr controller_;
    NavigatorPtr  navigator_;

    bool param_verbose_;

    template<class T> void getParam(ros::NodeHandle nh, std::string param_name, T& param, T default_value)
    {
      nh.param<T>(param_name, param, default_value);

      if(param_verbose_)
        ROS_INFO_STREAM("[" << nh.getNamespace() << "] " << param_name << ": " << param);
    }
  };
};
