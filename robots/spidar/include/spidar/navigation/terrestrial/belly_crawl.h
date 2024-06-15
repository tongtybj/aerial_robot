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

      class BellyCrawl : public Base
      {

        enum
          {
           PHASE0, // initialize phase
           PHASE1, // limb motion
           PHASE2, // belly motion
          };

      public:
        BellyCrawl();
        ~BellyCrawl(){}

        void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                        double loop_du) override;

        void update() override;

        void setDeltaPos(tf::Vector3 delta_pos);
        void setPose(tf::Transform pose);
        void setPose(tf::Vector3 pos, double yaw = 0);
        void reset();

      private:

        ros::Subscriber move_sub_;

        tf::Vector3 final_target_pos_;
        tf::Vector3 target_pos_;
        tf::Vector3 init_pos_;
        tf::Vector3 prev_pos_;
        double final_target_yaw_;
        double target_yaw_;
        std::vector<double> prev_target_joint_angles_;
        std::vector<double> final_target_joint_angles_;

        int phase_;
        bool move_flag_;

        // param
        double stride_;
        bool cycle_reset_leg_end_;
        bool cycle_reset_baselink_;
        bool belly_debug_;
        bool limb_debug_;

        // limb move
        struct Limb {
          enum
            {
             PHASE0, // idle
             PHASE1, // raise
             PHASE2, // horizon move
             PHASE3, // descend
             PHASE4, // servo off
            };

          uint8_t phase_;

          double prev_t_;
          double servo_switch_t_;

          // param
          double loop_duration_;
          double servo_switch_duration_;
          double joint_err_thresh_;
        };

        Limb limb_;

        // belly move
        struct Belly {
          enum
            {
             PHASE0, // idle
             PHASE1, // raise
             PHASE2, // horizon move
             PHASE3, // descend
             PHASE4, // servo off
            };

          uint8_t phase_;

          double prev_t_;
          double servo_switch_t_;

          double raise_height_;
          double raise_thresh_;
          double move_thresh_;
          double descend_thresh_;
          double loop_duration_;
          double servo_switch_duration_;

        };

        Belly belly_;


        void raiseAllLimbs();
        void lowerAllLimbs();
        void stateMachine();
        void limbSubStateMachine();
        void bellySubStateMachine();

        void iterativeUpdateTargetPos();

        void rosParamInit() override;
        void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override;
        void moveCallback(const geometry_msgs::Vector3StampedConstPtr& msg);

      };
    };
  };
};

