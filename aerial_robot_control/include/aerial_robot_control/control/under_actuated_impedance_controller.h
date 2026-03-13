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

#include <aerial_robot_control/control/under_actuated_controller.h>
#include <aerial_robot_control/control/utils/care.h>
#include <aerial_robot_control/LQIConfig.h>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <aerial_robot_msgs/ImpedanceControl.h>
#include <dynamic_reconfigure/server.h>
#include <spinal/FourAxisCommandImpedance.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <spinal/RollPitchYawTerms.h>
#include <ros/ros.h>
#include <thread>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Point.h>
namespace aerial_robot_control
{
  class UnderActuatedImpedanceController: public UnderActuatedController
  {

  public:
    UnderActuatedImpedanceController();
    virtual ~UnderActuatedImpedanceController();

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

    void activate() override;

  protected:

    ros::Publisher flight_cmd_pub_; // for spinal
    ros::Publisher flight_impedance_cmd_pub_; // for rosbag
    ros::Publisher rpy_gain_pub_; // for spinal
    ros::Publisher four_axis_gain_pub_;
    ros::Publisher p_matrix_pseudo_inverse_inertia_pub_;
    ros::Publisher imp_command_pub_; 
    ros::Subscriber external_wrench_sub_;
     
    bool verbose_;
    boost::shared_ptr<dynamic_reconfigure::Server<aerial_robot_control::LQIConfig> > lqi_server_;
    dynamic_reconfigure::Server<aerial_robot_control::LQIConfig>::CallbackType dynamic_reconf_func_lqi_;

    double target_roll_, target_pitch_;
    double candidate_yaw_term_;
    std::vector<float> target_base_thrust_;
    std::vector<float> target_z_thrust_;
    std::vector<float> target_roll_thrust_;
    std::vector<float> target_pitch_thrust_;
    std::vector<float> target_yaw_thrust_;


    int lqi_mode_;
    bool clamp_gain_;
    Eigen::MatrixXd K_;

    Eigen::Vector3d roll_pitch_weight_, yaw_weight_, z_weight_;
    std::vector<double> r_; // matrix R

    std::vector<Eigen::Vector3d> pitch_gains_, roll_gains_, yaw_gains_, z_gains_;

    bool realtime_update_;
    bool gyro_moment_compensation_;
    std::thread gain_generator_thread_;

    std::vector<std::string> joint_name_;

    Eigen::VectorXd joint_pos_ = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd joint_vel_ = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd target_joint_pos_ = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd target_joint_vel_ = Eigen::VectorXd::Zero(3);;
    Eigen::VectorXd target_joint_acc_ = Eigen::VectorXd::Zero(3);;
    Eigen::MatrixXd Pre_Bpr_ = Eigen::MatrixXd::Zero(3, 6);
    Eigen::MatrixXd Pre_Bn_ = Eigen::MatrixXd::Zero(6, 6);
    Eigen::VectorXd Pre_Pe_ = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd Pre_xi_ = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd target_thrust_z_term_;
    Eigen::VectorXd target_thrust_roll_term_;
    Eigen::VectorXd target_thrust_pitch_term_;
    Eigen::VectorXd target_thrust_yaw_term_;
    ros::Time time_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber joint_cmd_sub_;
    ros::Subscriber pos_cmd_sub_;
    ros::Subscriber mode_sub_;
    std::vector<ros::Publisher> joint_cmd_pubs_;
    std::vector<ros::Publisher> pos_pubs_;
    geometry_msgs::Point pos_cmd_;
    std_msgs::UInt8 mode_;
    aerial_robot_msgs::ImpedanceControl imp_cmd_;

    geometry_msgs::WrenchStamped external_wrench_;
    //private functions

    virtual bool checkRobotModel();
    void resetGain() { K_ = Eigen::MatrixXd(); }

    virtual void rosParamInit();
    virtual void controlCore() override;



    // virtual Eigen::MatrixXd getCmatrix(Eigen::MatrixXd delta_M, Eigen::VectorXd delta_xi,  Eigen::VectorXd xi_dot);

    virtual void sendCmd() override;
    virtual void sendFourAxisCommand();

    Eigen::MatrixXd getQInv();

    void allocateThrustTerm(Eigen::VectorXd &target_thrust_z_term);
  
    void sendRotationalInertiaComp();

    void gainGeneratorFunc();

    void addExternalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg);
  };
};
