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

#include <OsqpEigen/OsqpEigen.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <numeric>

// alias
using RobotModelPtr = boost::shared_ptr<aerial_robot_model::transformable::RobotModel>;
using PrimeBoundMap = std::map<int, std::pair<double, double>>;

namespace allocation
{

  void updateJointTorqueMatrices(RobotModelPtr robot_model, \
                                 const KDL::JntArray& gimbal_processed_joint, \
                                 const std::vector<Eigen::Matrix3d>& links_rotation_from_cog, \
                                 const std::vector<int>& roll_locked_gimbal, \
                                 const std::vector<double>& gimbal_nominal_angles, \
                                 const double thrust_force_weight, const double joint_torque_weight,  \
                                 Eigen::MatrixXd& A1, Eigen::VectorXd& b1, Eigen::MatrixXd& Psi);


  void compensateJointTorque(const Eigen::MatrixXd& A1, const Eigen::MatrixXd& Psi, \
                             const Eigen::VectorXd& b1, const Eigen::VectorXd& b2, \
                             const Eigen::MatrixXd& full_q_mat_inv, const Eigen::MatrixXd& full_q_mat, \
                             const double joint_torque_weight, \
                             Eigen::VectorXd& target_vectoring_f);

  namespace constraint
  {
    bool vectoring(RobotModelPtr robot_model,        \
                   const Eigen::MatrixXd& full_q_mat,           \
                   const Eigen::VectorXd& target_wrench,        \
                   const std::vector<int>& roll_locked_gimbal,  \
                   const PrimeBoundMap& rotor_bound_map,        \
                   Eigen::VectorXd& vec_f);
  };
};
