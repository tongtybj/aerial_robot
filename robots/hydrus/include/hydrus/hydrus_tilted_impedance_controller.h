// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#include <aerial_robot_control/control/under_actuated_tilted_impedance_controller.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <hydrus/imu.h>
#include <std_msgs/Bool.h>
#include <thread>

namespace differential_kinematics 
{
  class EndEffectorIKSolverCore;
}

namespace aerial_robot_control
{
  class HydrusTiltedImpedanceController: public UnderActuatedTiltedImpedanceController
  {
  public:
    HydrusTiltedImpedanceController();
    ~HydrusTiltedImpedanceController()
    {
      wrench_estimate_thread_.interrupt();
      wrench_estimate_thread_.join();
    }


    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

  protected:

    ros::Publisher estimate_external_wrench_pub_;
    ros::Subscriber ext_force_sub_;
    ros::Subscriber plan_flag_sub_;
    ros::Subscriber end_wrench_sub_;

 

    Eigen::MatrixXd Pre_J_ = Eigen::MatrixXd::Zero(6, 6);



    boost::thread wrench_estimate_thread_;
    Eigen::VectorXd init_sum_momentum_;
    Eigen::VectorXd integrate_term_;
    double prev_est_wrench_timestamp_;
    Eigen::MatrixXd momentum_observer_matrix_;

    Eigen::Vector3d xd_ddot_, xd_dot_, xd_, xref_;
    Eigen::Vector3d Fext_;

    ros::Time time_; 

    geometry_msgs::Wrench ext_force_;
    ros::Duration ext_duration_;

    ros::Publisher ee_pos_pub_;
    ros::Publisher joints_ctrl_pub_;
    ros::Publisher flight_nav_pub_;

    sensor_msgs::JointState joint_cmd_;

    bool plan_flag_ = false;

    double p_;

    geometry_msgs::WrenchStamped end_external_wrench_;

    double ma_, ca_, ka_;
    double fref_;
    double kot_, kor_;

    void externalWrenchEstimate();
    KDL::JntArray q_result_;
    bool inited = false;
    bool checkRobotModel() override;
    virtual void controlCore() override;
    virtual void rosParamInit() override;
    void extForceCallback(const geometry_msgs::WrenchConstPtr& cmd);
    void planStartCallback(const std_msgs::BoolConstPtr& msg);
    void endWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& cmd);

  };
   
};
