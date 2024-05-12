// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, DRAGON Lab
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

#include <spidar/navigation/terrestrial/base.h>

namespace aerial_robot_navigation
{
  namespace Spider
  {
    namespace Terrestrial
    {
      class QuadrupedPrimitiveWalk : public Base
      {
        enum
          {
            PHASE0, // idle, all leg contact with ground
            PHASE1, // raise leg
            PHASE2, // lower leg
            PHASE3, // move center link
          };

      public:
        QuadrupedPrimitiveWalk();
        ~QuadrupedPrimitiveWalk(){}

        void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

        void update() override;

      private:

        ros::Subscriber move_sub_;

        uint8_t phase_;
        bool move_flag_;
        int cycle_cnt_;
        double stride_;
        int leg_id_;
        bool move_baselink_;

        double raise_angle_orig_;

        // param
        int total_cycle_;
        bool debug_;
        int debug_legs_;
        bool cycle_reset_baselink_;
        bool cycle_reset_leg_end_;
        double converge_du_;
        double baselink_converge_thresh_;
        double move_leg_joint_err_thresh_;
        double front_leg_raise_ratio_;

        void stateMachine();

        void rosParamInit() override;
        void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override;
        void moveCallback(const std_msgs::BoolConstPtr& msg);

        void resetStateMachine();

      };
    };
  };
};
