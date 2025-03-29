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

#include <dragon/util/allocation.h>

namespace allocation
{

  void updateAllocationMatrices(TransformableModelPtr robot_model, \
                                Eigen::MatrixXd& A1, Eigen::VectorXd& b1)
  {
    auto dragon_robot_model = boost::dynamic_pointer_cast<Dragon::FullVectoringRobotModel>(robot_model);

    auto gimbal_processed_joint = dragon_robot_model->getGimbalProcessedJoint<KDL::JntArray>();
    auto links_rotation_from_cog = dragon_robot_model->getLinksRotationFromCog<Eigen::Matrix3d>();
    auto roll_locked_gimbal = dragon_robot_model->getRollLockedGimbal();
    auto gimbal_nominal_angles = dragon_robot_model->getGimbalNominalAngles();

    updateAllocationMatrices(robot_model, gimbal_processed_joint, links_rotation_from_cog, \
                             roll_locked_gimbal, gimbal_nominal_angles, A1, b1);
  }

  void updateAllocationMatrices(TransformableModelPtr robot_model,      \
                                const KDL::JntArray& gimbal_processed_joint, \
                                const std::vector<Eigen::Matrix3d>& links_rotation_from_cog, \
                                const std::vector<int>& roll_locked_gimbal, \
                                const std::vector<double>& gimbal_nominal_angles, \
                                Eigen::MatrixXd& A1, Eigen::VectorXd& b1)
  {
    int gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);

    // for considering joint torque
    robot_model->calcBasicKinematicsJacobian(); // for get joint torque

    const auto& thrust_coord_jacobians = robot_model->getThrustCoordJacobians();
    const int joint_num = robot_model->getJointNum();
    const int link_joint_num = robot_model->getLinkJointIndices().size();
    const int rotor_num = robot_model->getRotorNum();
    const int f_ndof = 3 * rotor_num - gimbal_lock_num;

    Eigen::MatrixXd A1_all = Eigen::MatrixXd::Zero(joint_num, f_ndof);
    int cnt = 0;
    for (int i = 0; i < rotor_num; i++) {
      Eigen::MatrixXd a = -thrust_coord_jacobians.at(i).topRows(3).rightCols(joint_num).transpose();
      Eigen::MatrixXd r = links_rotation_from_cog.at(i);
      if(roll_locked_gimbal.at(i) == 0) { /* 3DoF */
        // describe force w.r.t. local (link) frame
        A1_all.middleCols(cnt, 3) = a * r;
        cnt += 3;
      }
      else { /* gimbal lock: 2Dof */
        // describe force w.r.t. local (link) frame
        Eigen::MatrixXd mask(3,2);
        mask << 1, 0, 0, 0, 0, 1;
        Eigen::MatrixXd r_dash = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles.at(i * 2), 0, 0));
        A1_all.middleCols(cnt, 2) = a * r * r_dash * mask;
        cnt += 2;
      }
    }
    Eigen::VectorXd b1_all = Eigen::VectorXd::Zero(joint_num);

    Eigen::VectorXd g = robot_model->getGravity();
    for(const auto& inertia : robot_model->getInertiaMap()) {
      Eigen::MatrixXd cog_coord_jacobian = robot_model->TransformableModel::getJacobian(gimbal_processed_joint, inertia.first, inertia.second.getCOG());
      b1_all -= cog_coord_jacobian.rightCols(joint_num).transpose() * inertia.second.getMass() * (-g);
    }

    // only consider link joint
    A1 = Eigen::MatrixXd::Zero(link_joint_num, f_ndof);
    b1 = Eigen::VectorXd::Zero(link_joint_num);
    cnt = 0;
    for(int i = 0; i < joint_num; i++) {
      if(robot_model->getJointNames().at(i) == robot_model->getLinkJointNames().at(cnt))
        {
          A1.row(cnt) = A1_all.row(i);
          b1(cnt) = b1_all(i);
          cnt++;
        }
      if(cnt == link_joint_num) break;
    }
  }

  void compensateJointTorque(const Eigen::MatrixXd& A1, const Eigen::MatrixXd& Psi, \
                             const Eigen::VectorXd& b1, const Eigen::VectorXd& b2, \
                             const Eigen::MatrixXd& full_q_mat_inv, const Eigen::MatrixXd& full_q_mat, \
                             const double joint_torque_weight, \
                             Eigen::VectorXd& target_vectoring_f)
  {
    Eigen::MatrixXd A2 = full_q_mat;
    Eigen::MatrixXd C = Psi * A2.transpose() * (A2 * Psi * A2.transpose()).inverse();
    Eigen::MatrixXd E = Eigen::MatrixXd::Identity(Psi.rows(), Psi.rows());
    Eigen::MatrixXd W2 = joint_torque_weight * Eigen::MatrixXd::Identity(b1.size(), b1.size());

    target_vectoring_f += full_q_mat_inv * b2;
    target_vectoring_f += (- C * b2 - (E - C * A2) * Psi * A1.transpose() * W2 * b1);

    // ROS_INFO_STREAM_THROTTLE(1.0, "total acc is : " << target_wrench_acc_cog.transpose() << "; diff is: " << (A2 * target_vectoring_f_ + b2).transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "total joint torque is: " << (A1 * target_vectoring_f_ + b1).transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "target thrust is: " << target_vectoring_f_.transpose());
  }


  namespace constraint
  {

    bool vectoring(TransformableModelPtr robot_model, \
                   const Eigen::MatrixXd& full_q_mat, \
                   const Eigen::VectorXd& target_wrench, \
                   const std::vector<int>& roll_locked_gimbal, \
                   const PrimeBoundMap& rotor_bound_map, \
                   Eigen::VectorXd& vec_f)
    {

      int gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);

      int rotor_num = robot_model->getRotorNum();
      int f_ndof = 3 * rotor_num - gimbal_lock_num;
      int cons_num = 6 + f_ndof;
      cons_num += rotor_bound_map.size() * 2; // consider the interfere map

      OsqpEigen::Solver qp_solver;
      qp_solver.settings()->setVerbosity(false);
      qp_solver.settings()->setWarmStart(true);

      qp_solver.data()->setNumberOfVariables(f_ndof);
      qp_solver.data()->setNumberOfConstraints(cons_num);

      // 2.1 cost function
      Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(f_ndof, f_ndof);
      Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
      Eigen::VectorXd gradient = Eigen::VectorXd::Zero(f_ndof);
      qp_solver.data()->setHessianMatrix(hessian_sparse);
      qp_solver.data()->setGradient(gradient);

      // 2.2 constraint (except of range)
      Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(cons_num, f_ndof);
      constraints.topRows(6) = full_q_mat;
      constraints.middleRows(6, f_ndof) = Eigen::MatrixXd::Identity(f_ndof, f_ndof);
      double thrust_range = 40; // parameter
      Eigen::VectorXd lower_bound = Eigen::VectorXd::Ones(cons_num) * -thrust_range;
      Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(cons_num) * thrust_range;
      lower_bound.head(6) = target_wrench;
      upper_bound.head(6) = target_wrench;

      if (rotor_bound_map.size() > 0)
        {
          int row = 6 + f_ndof;
          int col = 0;

          for (int i = 0; i < rotor_num; i ++)
            {
              auto map_it = rotor_bound_map.find(i);
              if (map_it != rotor_bound_map.end())
                {
                  double prime_bound = map_it->second.first;
                  double sub_bound = map_it->second.second;
                  double direction = sub_bound - prime_bound;

                  int z_index = 2;
                  if (roll_locked_gimbal.at(i)) z_index = 1;

                  double prime_pitch_angle = prime_bound;
                  constraints(row, col) = cos(prime_pitch_angle); // x
                  constraints(row, col + z_index) = -sin(prime_pitch_angle); // z
                  if (direction > 0) lower_bound(row) = 0;
                  else upper_bound(row) = 0;

                  double sub_pitch_angle = sub_bound;
                  constraints(row + 1, col) = cos(sub_pitch_angle); // x
                  constraints(row + 1, col + z_index) = -sin(sub_pitch_angle); // z
                  if (direction > 0) upper_bound(row+1) = 0;
                  else lower_bound(row+1) = 0;
                  row += 2;

                }

              if (roll_locked_gimbal.at(i)) col +=2;
              else col +=3;
            }
        }


      Eigen::SparseMatrix<double> constraint_sparse = constraints.sparseView();
      qp_solver.data()->setLinearConstraintsMatrix(constraint_sparse);
      qp_solver.data()->setLowerBound(lower_bound);
      qp_solver.data()->setUpperBound(upper_bound);

      if(!qp_solver.initSolver()) {
        ROS_ERROR_STREAM("can not initialize qp solver");
        return false;
      }

      double s_t = ros::WallTime::now().toSec();
      bool res = qp_solver.solve(); // with large range: x 1.5

      if(!res) {
        ROS_ERROR_STREAM("can not solve QP");
        return false;
      } else {
        vec_f = qp_solver.getSolution();
        return true;
      }
    }
  };
};
