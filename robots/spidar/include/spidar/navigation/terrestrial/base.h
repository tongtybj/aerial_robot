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

#include <aerial_robot_control/flight_navigation.h>
#include <spidar/model/ground_robot_model.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <spinal/FlightConfigCmd.h>

namespace aerial_robot_control
{
  namespace Spider
  {
    class WalkController;
  };
};

namespace aerial_robot_navigation
{
  namespace Spider
  {
    namespace Terrestrial
    {
      class Base : public BaseNavigator
      {
      public:
        Base();
        ~Base(){}

        virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                double loop_du) override;

        virtual void update() override;

        tf::Vector3 getCurrentBaselinkPos();

        inline tf::Vector3 getTargetBaselinkPos() {return target_baselink_pos_;}
        inline tf::Vector3 getTargetBaselinkRpy() {return target_baselink_rpy_;}
        inline tf::Vector3 getTargetBaselinkVel() {return target_baselink_vel_;}
        inline sensor_msgs::JointState getTargetJointState() {return target_joint_state_;}
        inline std::vector<KDL::Rotation> getTargetLinkRots() {return target_link_rots_;}
        inline std::vector<KDL::Frame> getTargetLegEnds() {return target_leg_ends_;}
        inline bool getRaiseLegFlag() const { return raise_leg_flag_; }
        inline bool getLowerLegFlag() const { return lower_leg_flag_; }
        inline bool isRaiseLegConverge() const { return raise_converge_; }
        inline int getFreeleg() const { return free_leg_id_; }
        void setTargetBaselinkPos(tf::Vector3 pos, bool update_joint_angle = true);
        void addTargetBaselinkPos(tf::Vector3 delta_pos, bool update_joint_angle = true);
        void setTargetBaselinkPose(tf::Vector3 pos, tf::Vector3 rpy, bool update_joint_angle = true);
        void setTargetLegEnds(std::vector<KDL::Frame> frames, bool update_joint_angle = true);
        void resetTargetLegEnds();


        void setController(aerial_robot_control::Spider::WalkController* controller){
          walk_controller_ = controller;
        }


      protected:

        tf::Vector3 target_baselink_pos_;
        tf::Vector3 target_baselink_vel_;
        tf::Vector3 target_baselink_rpy_;

        std::vector<int> joint_index_map_;
        sensor_msgs::JointState target_joint_state_;
        std::vector<KDL::Frame> target_leg_ends_;
        std::vector<KDL::Rotation> target_link_rots_;

        bool reset_baselink_flag_;
        bool reset_leg_ends_flag_;

        int free_leg_id_; // start from 0: [0, leg_num -1]
        bool raise_leg_flag_;
        bool lower_leg_flag_;
        bool raise_converge_;
        double raise_angle_;
        double front_leg_raise_ratio_;
        double raise_converge_thresh_;
        double lower_touchdown_thresh_;
        double constant_angle_thresh_;
        double baselink_rot_thresh_;
        double opposite_raise_leg_thresh_;
        double check_interval_;
        double converge_time_thresh_;
        double joint_angle_limit_;

        double contact_check_start_t_, contact_check_prev_t_;
        std::vector<double> contact_check_prev_joint_angles_;

        bool kinematic_simulation_;
        sensor_msgs::JointState sim_joint_state_;
        double sim_joint_lpf_rate_;

        double converge_timestamp_;

        ros::Subscriber target_baselink_pos_sub_;
        ros::Subscriber target_baselink_delta_pos_sub_;

        ros::Subscriber raise_leg_sub_;
        ros::Subscriber lower_leg_sub_;

        ros::Publisher target_leg_ends_pub_;
        ros::Publisher sim_baselink_pose_pub_;
        ros::Publisher sim_joint_angles_pub_;
        ros::Publisher sim_flight_config_pub_;
        ros::Subscriber sim_flight_config_sub_;

        boost::shared_ptr<::Spider::GroundRobotModel> spidar_robot_model_;
        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_nav_;
        aerial_robot_control::Spider::WalkController* walk_controller_;

        void halt() override;

        virtual void rosParamInit() override;

        virtual void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg) override;

        void targetBaselinkPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
        void targetBaselinkDeltaPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
        void raiseLegCallback(const std_msgs::UInt8ConstPtr& msg);
        void lowerLegCallback(const std_msgs::EmptyConstPtr& msg);
        void walkCallback(const std_msgs::BoolConstPtr& msg);
        void simulateFlightConfigCallback(const spinal::FlightConfigCmdConstPtr& msg);

        bool checkKinematics();

        void raiseLeg(int leg_id);
        void lowerLeg();
        void resetContactStatus();
        void contactLeg();

        void updateRobotModelForNav();
        bool updateJoinAngleFrominverseKinematics(bool allow_fail = false);
        void freeLegAction();
        void failSafeAction();

        void kinematicSimulate();

        // utils
        void setJointIndexMap();
        std::vector<double> getCurrentJointAngles();
        bool getCurrentLegEndsPos(std::vector<KDL::Frame>& leg_ends_pos);
      };
    };
  };
};
