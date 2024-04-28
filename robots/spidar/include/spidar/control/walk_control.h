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

#include <aerial_robot_control/control/pose_linear_controller.h>
#include <spidar/model/ground_robot_model.h>
#include <spidar/navigation/walk_navigation.h>
#include <spinal/FourAxisCommand.h>
#include <std_msgs/Float32MultiArray.h>
#include <spinal/ServoTorqueCmd.h>
#include <std_msgs/Empty.h>
#include <std_srvs/SetBool.h>
#include <OsqpEigen/OsqpEigen.h>

namespace aerial_robot_control
{
  namespace Spider
  {
    class WalkController: public PoseLinearController
    {
    public:
      WalkController();
      ~WalkController() {}

      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                      boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                      double ctrl_loop_rate) override;
      bool update() override;
      void reset() override;

      void startRaiseLeg();
      void startLowerLeg();
      void startContactTransition(int leg_id);

      bool getContactTransition();
      void setOldQuadrupedWalkMode(bool flag);

    private:

      ros::Publisher flight_cmd_pub_; //for spinal
      ros::Publisher gimbal_control_pub_;
      ros::Publisher joint_angle_pub_;
      ros::Publisher joint_torque_pub_;
      ros::Publisher target_vectoring_force_pub_;
      ros::Publisher extra_joint_torque_pub_;
      ros::Publisher joint_servo_enable_pub_;
      ros::Subscriber joint_no_load_sub_;
      ros::ServiceServer joint_yaw_torque_srv_, joint_pitch_torque_srv_;

      boost::shared_ptr<::Spider::GroundRobotModel> spidar_robot_model_;
      boost::shared_ptr<aerial_robot_navigation::Spider::WalkNavigator> spidar_walk_navigator_;

      std::vector<PID> walk_pid_controllers_;
      std::vector<boost::shared_ptr<PidControlDynamicConfig> > walk_pid_reconf_servers_;

      std::vector<PID> joint_torque_controllers_;

      std::vector<int> joint_index_map_;

      Eigen::VectorXd static_thrust_force_;
      Eigen::VectorXd static_joint_torque_;

      std::vector<float> target_base_thrust_;
      std::vector<double> target_gimbal_angles_;
      std::vector<std::string> target_gimbal_names_;
      Eigen::VectorXd target_vectoring_f_;
      Eigen::VectorXd target_extra_joint_torque_;

      sensor_msgs::JointState target_joint_angles_;
      sensor_msgs::JointState target_joint_torques_;
      std::vector<double> prev_navi_target_joint_angles_;
      sensor_msgs::JointState prior_raise_leg_target_joint_angles_;
      double joint_ctrl_rate_;
      double tor_kp_;

      bool pedipulate_mode_;
      bool old_quadruped_walk_mode_;

      bool set_init_servo_torque_;
      double joint_static_torque_limit_;
      double raise_joint_static_torque_limit_;
      double pedipulate_joint_static_torque_limit_;
      double servo_max_torque_;
      double servo_angle_bias_;
      double servo_angle_bias_torque_;
      double z_offset_;

      double angle_scale_;
      double torque_load_scale_;

      double thrust_force_weight_;
      double joint_torque_weight_;

      double joint_error_angle_thresh_;
      double contact_leg_force_i_gain_;
      double free_leg_force_ratio_;

      Eigen::VectorXd raise_static_thrust_force_;

      double prev_t_;
      double prev_v_;
      double check_interval_;

      bool contact_transition_;
      int contact_leg_id_;

      Eigen::VectorXd pusedo_baselink_wrench_;

      void rosParamInit();
      virtual void sendCmd() override;

      bool servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& name);

      void cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices) override;

      void calcStaticBalance();
      void jointControl();

      void pedipulateThrustControl();
      void pedipulateSingleArmThrustControl(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b1, const Eigen::MatrixXd& A2, const Eigen::VectorXd& b2, const int& joint_id, Eigen::VectorXd& f_all);
      void pedipulateAllArmThrustControl(const Eigen::MatrixXd& A1, const Eigen::VectorXd& b1, const Eigen::MatrixXd& A2, const Eigen::VectorXd& b2, const Eigen::MatrixXd& rot, bool baselink_balance, Eigen::VectorXd& f_all);

      void quadrupedThrustControl();
      void quadrupedCalcStaticBalance();


      // utils
      void setJointIndexMap();
      std::vector<double> getCurrentJointAngles();
      inline double clamp(double v, double b) { return std::min(std::max(v, -b), b); }
      Eigen::VectorXd clamp(Eigen::VectorXd v, double b);
      bool samejointAngles(std::vector<double> group_a, std::vector<double> group_b);
    };
  };
};
