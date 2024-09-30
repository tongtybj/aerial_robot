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

#include <dragon/model/full_vectoring_robot_model.h>

using namespace Dragon;

namespace
{
  double minimumControlWrench(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
  {
    FullVectoringRobotModel *model = reinterpret_cast<FullVectoringRobotModel*>(ptr);
    int rotor_num = model->getRotorNum();
    std::vector<Eigen::Matrix3d> link_rot = model->getLinksRotationFromCog<Eigen::Matrix3d>();
    std::vector<Eigen::Vector3d> gimbal_roll_pos = model->getGimbalRollOriginFromCog<Eigen::Vector3d>();
    auto model_for_plan = model->getRobotModelForPlan();
    std::vector<Eigen::Vector3d> rotor_pos = model_for_plan->getRotorsOriginFromCog<Eigen::Vector3d>();

    std::vector<int> roll_locked_gimbal = model->getRollLockedGimbalForPlan();
    for(int i = 0; i < rotor_num; i++)
    {
      /*
         Since the position of the force acting point would change according to the gimbal roll,
there is a diffiretial chain about the roll angle. But we here approximate it to the gimbal roll frame along the link axis, rather than the true force acting point (gimbal pitch frame). The offset is 0.037m, which is a small offset in the optimization problem.
       */
      if(roll_locked_gimbal.at(i) == 1) rotor_pos.at(i) = gimbal_roll_pos.at(i);
    }

    const auto f_min_list = model->calcFeasibleControlFxyDists(roll_locked_gimbal, x, rotor_num, link_rot);
    const auto t_min_list = model->calcFeasibleControlTDists(roll_locked_gimbal, x, rotor_num, rotor_pos, link_rot);

    return model->getMinForceNormalizedWeight() * f_min_list.minCoeff() +  model->getMinTorqueNormalizedWeight() * t_min_list.minCoeff();
  }
}

FullVectoringRobotModel::FullVectoringRobotModel(bool init_with_rosparam, bool verbose, double edf_radius, double edf_max_tilt) :
  HydrusLikeRobotModel(init_with_rosparam, verbose, 0, 0, 10, edf_radius, edf_max_tilt)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  /* gimbal roll lock */
  const int rotor_num = getRotorNum();
  roll_lock_status_accumulator_.resize(rotor_num, 0);
  roll_lock_angle_smooth_.resize(rotor_num, 0);
  prev_roll_locked_gimbal_.resize(rotor_num, 0);
  roll_locked_gimbal_.resize(rotor_num, 0);
  gimbal_roll_origin_from_cog_.resize(rotor_num);
  setGimbalNominalAngles(std::vector<double>(0)); // for online initialize

  robot_model_for_plan_ = boost::make_shared<aerial_robot_model::transformable::RobotModel>();

  if(debug_verbose_)
    {
      if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
        ros::console::notifyLoggerLevelsChanged();
    }
}

void FullVectoringRobotModel::getParamFromRos()
{
  ros::NodeHandle nh;

  nh.param("debug_verbose", debug_verbose_, false);

  nh.param("gimbal_lock_threshold", gimbal_lock_threshold_, 0.3); // > 10deg
  nh.param("link_att_change_threshold", link_att_change_threshold_, 0.2); // 10deg
  nh.param("lock_status_change_threshold", lock_status_change_threshold_, 10); // 10deg
  nh.param("gimbal_delta_angle", gimbal_delta_angle_, 0.3); // 10deg
  nh.param("robot_model_refine_max_iteration", robot_model_refine_max_iteration_, 1);
  nh.param("robot_model_refine_threshold", robot_model_refine_threshold_, 0.01); // m
  nh.param("gimbal_roll_change_threshold", gimbal_roll_change_threshold_, 0.02); // rad/s
  nh.param("min_force_weight", min_force_weight_, 1.0);
  nh.param("min_torque_weight", min_torque_weight_, 1.0);

  nh.param("thrust_force_weight", thrust_force_weight_, 1.0);
  nh.param("joint_torque_weight", joint_torque_weight_, 1.0);
}

void FullVectoringRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::Rotation cog_desire_orientation = getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_plan_->setCogDesireOrientation(cog_desire_orientation); // update the cog orientation

  /* 1. first assume the gimbals are level */
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_rot = f_baselink.M * cog_desire_orientation.Inverse();

  const auto joint_index_map = getJointIndexMap();
  KDL::JntArray gimbal_processed_joint = joint_positions;
  std::vector<double> gimbal_prime_angles(0);
  std::vector<KDL::Rotation> links_rotation_from_cog(0);
  std::vector<int> roll_locked_gimbal(getRotorNum(), 0);

  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f;
      fk_solver.JntToCart(joint_positions, f, std::string("link") + s);

      links_rotation_from_cog.push_back(cog_rot.Inverse() * f.M);
      double r, p, y;
      links_rotation_from_cog.back().GetRPY(r, p, y);

      gimbal_prime_angles.push_back(-r);
      gimbal_prime_angles.push_back(-p);
    }
  setLinksRotationFromCog(links_rotation_from_cog);

  // online initialization
  if(getGimbalNominalAngles().size() == 0)
    {
      for(int i = 0; i < getRotorNum(); ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_prime_angles.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_prime_angles.at(i * 2 + 1);
        }
      robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);

      setGimbalNominalAngles(gimbal_prime_angles);
    }


  /* 2. check the orientation (pitch angle) of link to decide whether to lock gimbal roll */
  // TODO: we should not use the link pitch angle to decide, should use the angle between link and gimbal (thus means the gimbal pitch angle)
  double max_pitch = 0;
  bool roll_lock_status_change = false;
  for(int i = 0; i < getRotorNum(); ++i)
    {
      if(fabs(gimbal_prime_angles.at(i * 2 + 1)) > max_pitch)
        max_pitch = fabs(gimbal_prime_angles.at(i * 2 + 1));

      if(fabs(fabs(gimbal_prime_angles.at(i * 2 + 1)) - M_PI /2) < gimbal_lock_threshold_)
        {
          ROS_DEBUG_STREAM_NAMED("robot_model", "link" << i+1 << " pitch: " << -gimbal_prime_angles.at(i * 2 + 1) << " exceeds");
          roll_locked_gimbal.at(i) = 1;
        }
      else
        {
          roll_locked_gimbal.at(i) = 0;
        }

      /* case1: the locked rotors have change  */
      if(prev_roll_locked_gimbal_.at(i) != roll_locked_gimbal.at(i))
        {
          ROS_DEBUG_STREAM_NAMED("robot_model", "roll locked status change, gimbal " << i+1 <<" , cnt: " << roll_lock_status_accumulator_.at(i));

          /* robust1: deal with the chattering around the changing bound by adding momentum */
          roll_lock_status_accumulator_.at(i)++;
          if(roll_lock_status_accumulator_.at(i) > lock_status_change_threshold_)
            {
              roll_lock_status_accumulator_.at(i) = 0;
              roll_lock_status_change = true;
              ROS_INFO_STREAM_NAMED("robot_model", "rotor " << i + 1 << ": the gimbal roll lock status changes from " << prev_roll_locked_gimbal_.at(i) << " to " << roll_locked_gimbal.at(i));
              if(roll_locked_gimbal.at(i) == 0)
                {
                  roll_lock_angle_smooth_.at(i) = 1;
                }
            }
          else
            {
              /* force change back the roll lock status */
              roll_locked_gimbal.at(i) = prev_roll_locked_gimbal_.at(i);
            }
        }
      else
        {
          roll_lock_status_accumulator_.at(i) = 0;
        }

    }
  ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "robot_model", "max link pitch: " << max_pitch);

  /* 3. calculate the optimized locked gimbal roll angles */
  std::vector<double>  gimbal_nominal_angles = getGimbalNominalAngles();
  for(int i = 0; i < getRotorNum(); ++i)
    {
      getShortestPath(gimbal_prime_angles.at(i * 2), gimbal_nominal_angles.at(i * 2),
                      gimbal_prime_angles.at(i * 2 + 1), gimbal_nominal_angles.at(i * 2 + 1));
    }
  int gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);

  if(gimbal_lock_num > 0)
    {
      /* case2: the link attitude change more than a threshold  */
      /* robust: deal with the vibration of the link orientation from joint angles and cog attitude, not update the gimbal roll lock angles so often */
      for(int i = 0; i < getRotorNum(); i++)
        {
          tf::Quaternion delta_q;
          tf::quaternionKDLToTF(prev_links_rotation_from_cog_.at(i).Inverse() * links_rotation_from_cog.at(i), delta_q);
          if(fabs(delta_q.getAngle()) > link_att_change_threshold_)
            {
              ROS_INFO_STREAM_NAMED("robot_model", "link " << i + 1 << ": the orientation is change more than threshold: " << delta_q.getAngle());
              roll_lock_status_change = true;
            }
        }

      if(roll_lock_status_change)
        {
          /* roughly update the CoG and gimbal roll origin based on the level (horizontal) nominal gimbal angles and joints for search the optimiazed locked gimbal roll angles*/
          for(int i = 0; i < getRotorNum(); ++i)
            {
              std::string s = std::to_string(i + 1);
              gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_prime_angles.at(i * 2);
              gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_prime_angles.at(i * 2 + 1);
            }
          robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);
          std::map<std::string, KDL::Frame> seg_tf_map = robot_model_for_plan_->getSegmentsTf();
          for(int i = 0; i < getRotorNum(); ++i)
            {
              std::string s = std::to_string(i + 1);
              std::string gimbal_roll = std::string("gimbal") + s + std::string("_roll_module");
              KDL::Frame f  = seg_tf_map.at(gimbal_roll);
              gimbal_roll_origin_from_cog_.at(i) = (robot_model_for_plan_->getCog<KDL::Frame>().Inverse() * f).p;
            }

          setRollLockedGimbalForPlan(roll_locked_gimbal);
          calcBestLockGimbalRoll(roll_locked_gimbal, prev_roll_locked_gimbal_, gimbal_nominal_angles, locked_angles_);
          prev_roll_locked_gimbal_ = roll_locked_gimbal;
          prev_links_rotation_from_cog_ = links_rotation_from_cog;
        }
    }
  else
    {
      locked_angles_.resize(0);
      prev_roll_locked_gimbal_ = roll_locked_gimbal;
      prev_links_rotation_from_cog_ = links_rotation_from_cog;
    }

  /* 4: smooth the nominal gimbal angles, to avoid sudden change */
  int gimbal_lock_index = 0;
  for(int i = 0; i < getRotorNum(); ++i)
    {
      /* only smooth roll angles */
      if(roll_locked_gimbal.at(i) == 0)
        {
          if(roll_lock_angle_smooth_.at(i) == 1)
            {
              ROS_DEBUG_NAMED("robot_model", "smooth the gimbal roll angle %d, for after free the lock", i+1);
              roll_locked_gimbal.at(i) = 1; // force change back to lock status

              double diff = gimbal_prime_angles.at(i * 2) - gimbal_nominal_angles.at(i * 2);
              if (diff > M_PI) diff -= 2 * M_PI;
              if (diff < -M_PI) diff += 2 * M_PI;

              if(diff > gimbal_roll_change_threshold_)
                gimbal_nominal_angles.at(i * 2) += gimbal_roll_change_threshold_;
              else if(diff < - gimbal_roll_change_threshold_)
                gimbal_nominal_angles.at(i * 2) -= gimbal_roll_change_threshold_;
              else
                {
                  gimbal_nominal_angles.at(i * 2) = gimbal_prime_angles.at(i * 2);
                  roll_lock_angle_smooth_.at(i) = 0; // stop smoothing
                  if(debug_verbose_) ROS_INFO_STREAM_NAMED("robot_model", "free the gimbal roll after the smoothing for rotor" << i+1);
                }

              if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "smoothing rotor " << i + 1 << ", desired roll angle: " << gimbal_prime_angles.at(i * 2) << ", curr angle: " << gimbal_nominal_angles.at(i * 2));

            }
          else
            {
              gimbal_nominal_angles.at(i * 2) = gimbal_prime_angles.at(i * 2);
            }
        }
      else
        {
          assert(roll_lock_angle_smooth_.at(i) == 1);

          double diff = locked_angles_.at(gimbal_lock_index) - gimbal_nominal_angles.at(i * 2);
          if(diff > gimbal_roll_change_threshold_)
            gimbal_nominal_angles.at(i * 2) += gimbal_roll_change_threshold_;
          else if(diff < - gimbal_roll_change_threshold_)
            gimbal_nominal_angles.at(i * 2) -= gimbal_roll_change_threshold_;
          else
            gimbal_nominal_angles.at(i * 2) = locked_angles_.at(gimbal_lock_index);

          if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "smoothing rotor " << i + 1 << ", desired roll angle: " << locked_angles_.at(gimbal_lock_index) << ", curr angle: " << gimbal_nominal_angles.at(i * 2));

          gimbal_lock_index++;
        }
      gimbal_nominal_angles.at(i * 2 + 1) = gimbal_prime_angles.at(i * 2 + 1);
    }
  gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0); // update
  const int f_ndof = 3 * getRotorNum() - gimbal_lock_num;

  /* 5: (new) refine the rotor origin from cog */
  /* 5.1. init update the CoG and gimbal roll origin based on the level (horizontal) nominal gimbal angles and joints */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles.at(i * 2);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_nominal_angles.at(i * 2 + 1);
    }
  robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);

  // workround: rotor interfere avoid
  PrimeBoundMap rotor_bound_map;
  rotorInterfereAvoid(rotor_bound_map, roll_locked_gimbal, gimbal_nominal_angles);

  /* 5.2. convergence  */
  double t = ros::Time::now().toSec();

  Eigen::MatrixXd A1;
  Eigen::VectorXd b1;
  Eigen::MatrixXd Psi;
  const auto links_rotation = aerial_robot_model::kdlToEigen(links_rotation_from_cog);
  updateJointTorqueMatrices(robot_model_for_plan_,                 \
                            gimbal_processed_joint, links_rotation, \
                            roll_locked_gimbal, gimbal_nominal_angles, \
                            thrust_force_weight_, joint_torque_weight_, \
                            A1, b1, Psi);


  for(int j = 0; j < robot_model_refine_max_iteration_; j++)
    {
      /* 5.2.1. update the wrench allocation matrix  */
      std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_plan_->getRotorsOriginFromCog<Eigen::Vector3d>();
      Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, f_ndof);
      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd mask(3,2);
      mask << 1, 0, 0, 0, 0, 1;
      int last_col = 0;
      for(int i = 0; i < getRotorNum(); i++)
        {
          wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));

          if(roll_locked_gimbal.at(i) == 0)
            {
              /* 3DoF */
              full_q_mat.block(0, last_col, 6, 3) = wrench_map * aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i));
              last_col += 3;
            }
          else
            {
              /* gimbal lock: 2Dof */
              full_q_mat.block(0, last_col, 6, 2) = wrench_map * aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i)) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles.at(i * 2), 0, 0)) * mask;
              last_col += 2;
            }
        }

      /* 5.2.2. update the vectoring force for hovering and the gimbal angles */
      Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
      Eigen::VectorXd gravity_force = getGravity() * robot_model_for_plan_->getMass();
      Eigen::VectorXd hover_vectoring_f = full_q_mat_inv * gravity_force;

      compensateJointTorque(A1, Psi, b1, -gravity_force, full_q_mat_inv, full_q_mat, \
                            joint_torque_weight_, hover_vectoring_f);


      // find the joint torque:
      Eigen::VectorXd extra_joint_torque = A1 * hover_vectoring_f + b1;
      // ROS_INFO_STREAM_THROTTLE(1.0, "extra joint torque: " << extra_joint_torque.transpose());
      ROS_INFO_STREAM_ONCE("extra joint torque by general hovering process: " << extra_joint_torque.transpose());


      // 2. solve by QP
      int rotor_num = getRotorNum();
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
      lower_bound.head(6) = gravity_force;
      upper_bound.head(6) = gravity_force;

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

                  double prime_pitch_angle = gimbal_nominal_angles.at(i * 2 + 1) + prime_bound;
                  constraints(row, col) = cos(prime_pitch_angle); // x
                  constraints(row, col + 1) = -sin(prime_pitch_angle); // z
                  if (direction > 0) lower_bound(row) = 0;
                  else upper_bound(row) = 0;

                  double sub_pitch_angle = gimbal_nominal_angles.at(i * 2 + 1) + sub_bound;
                  constraints(row + 1, col) = cos(sub_pitch_angle); // x
                  constraints(row + 1, col + 1) = -sin(sub_pitch_angle); // z
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
        return;
      }

      double s_t = ros::WallTime::now().toSec();
      bool res = qp_solver.solve(); // with large range: x 1.5

      if(!res) {
        ROS_ERROR_STREAM("can not solve QP");
      } else {
        hover_vectoring_f = qp_solver.getSolution();
      }

      // find the joint torque:
      extra_joint_torque = A1 * hover_vectoring_f + b1;
      ROS_INFO_STREAM_THROTTLE(1.0, "extra joint torque: " << extra_joint_torque.transpose());
      //ROS_INFO_STREAM_ONCE("extra joint torque by qp process with the consideration of rotor interference avoidance: " << extra_joint_torque.transpose());


      // grasp control
      //graspControl(A1, full_q_mat, extra_joint_torque);


      Eigen::VectorXd static_thrust = Eigen::VectorXd::Zero(getRotorNum());
      if(debug_verbose_) ROS_DEBUG_STREAM("vectoring force for hovering in iteration "<< j+1 << ": " << hover_vectoring_f.transpose());
      last_col = 0;
      for(int i = 0; i < getRotorNum(); i++)
        {
          double prev_roll_angle = gimbal_nominal_angles.at(2 * i);
          double prev_pitch_angle = gimbal_nominal_angles.at(2 * i + 1);

          if(roll_locked_gimbal.at(i) == 0)
            {
              static_thrust(i) = hover_vectoring_f.segment(last_col, 3).norm();
              Eigen::Vector3d f = hover_vectoring_f.segment(last_col, 3);

              double roll_angle = atan2(-f.y(), f.z());
              double pitch_angle = atan2(f.x(), -f.y() * sin(roll_angle) + f.z() * cos(roll_angle));

              gimbal_nominal_angles.at(2 * i) = roll_angle;
              gimbal_nominal_angles.at(2 * i + 1) = pitch_angle;

              last_col += 3;
            }
          else
            {
              static_thrust(i) = hover_vectoring_f.segment(last_col, 2).norm();
              Eigen::Vector2d f = hover_vectoring_f.segment(last_col, 2);

              // roll is locked
              gimbal_nominal_angles.at(i * 2 + 1) = atan2(f[0], f[1]);
              last_col += 2;
            }

          getShortestPath(gimbal_nominal_angles.at(2 * i), prev_roll_angle, \
                          gimbal_nominal_angles.at(2 * i + 1), prev_pitch_angle);
        }

      /* 5.2.3. check the change of the rotor origin whether converge*/
      std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog = rotors_origin_from_cog;
      for(int i = 0; i < getRotorNum(); ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = gimbal_nominal_angles.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = gimbal_nominal_angles.at(i * 2 + 1);
        }
      robot_model_for_plan_->updateRobotModel(gimbal_processed_joint);
      rotors_origin_from_cog = robot_model_for_plan_->getRotorsOriginFromCog<Eigen::Vector3d>();

      double max_diff = 1e-6;
      for(int i = 0; i < getRotorNum(); i++)
        {
          double diff = (rotors_origin_from_cog.at(i) - prev_rotors_origin_from_cog.at(i)).norm();
          if(diff > max_diff) max_diff = diff;
        }
      if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "refine rotor origin: iteration "<< j+1 << ", max_diff: " << max_diff);

      if(max_diff < robot_model_refine_threshold_)
        {
          if(debug_verbose_) ROS_DEBUG_STREAM_NAMED("robot_model", "refine rotor origin: converge in iteration " << j+1 << " max_diff " << max_diff << ", use " << ros::Time::now().toSec() - t << "sec");
          setStaticThrust(static_thrust);

          setVectoringForceWrenchMatrix(full_q_mat);

          // convert hovering vectoring f to CoG coord
          hover_vectoring_f_ = Eigen::VectorXd::Zero(3 * getRotorNum());
          Eigen::MatrixXd mask(3,2); mask << 1, 0, 0, 0, 0, 1;
          int col = 0;
          for(int i = 0; i < getRotorNum(); i++)
            {
              if(roll_locked_gimbal.at(i) == 0)
                {
                  hover_vectoring_f_.segment(3 * i, 3) = aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i)) * hover_vectoring_f.segment(col, 3);
                  col += 3;
                }
              else
                { /* gimbal lock: 2Dof */
                  hover_vectoring_f_.segment(3 * i, 3) =  aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i)) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles.at(i * 2), 0, 0)) * mask * hover_vectoring_f.segment(col, 2);
                  col += 2;
                }
            }
          break;
        }

      if(j == robot_model_refine_max_iteration_ - 1)
        {
          setStaticThrust(static_thrust);
          setVectoringForceWrenchMatrix(full_q_mat);

          // convert hovering vectoring f to CoG coord
          hover_vectoring_f_ = Eigen::VectorXd::Zero(3 * getRotorNum());
          Eigen::MatrixXd mask(3,2); mask << 1, 0, 0, 0, 0, 1;
          int col = 0;
          for(int i = 0; i < getRotorNum(); i++)
            {
              if(roll_locked_gimbal.at(i) == 0)
                {
                  hover_vectoring_f_.segment(3 * i, 3) = aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i)) * hover_vectoring_f.segment(col, 3);
                  col += 3;
                }
              else
                { /* gimbal lock: 2Dof */
                  hover_vectoring_f_.segment(3 * i, 3) =  aerial_robot_model::kdlToEigen(links_rotation_from_cog.at(i)) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles.at(i * 2), 0, 0)) * mask * hover_vectoring_f.segment(col, 2);
                  col += 2;
                }
            }

          ROS_WARN_STREAM_NAMED("robot_model", "refine rotor origin: can not converge in iteration " << j+1 << " max_diff " << max_diff);
        }
    }

  /* 7. update */
  aerial_robot_model::RobotModel::updateRobotModelImpl(gimbal_processed_joint);

  std::vector<KDL::Vector> f_edfs;
  auto seg_tf_map = getSegmentsTf();
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      std::string edf = std::string("edf") + s + std::string("_left");
      KDL::Frame f  = seg_tf_map.at(edf);
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * f).p);

      edf = std::string("edf") + s + std::string("_right");
      f  = seg_tf_map.at(edf);
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * f).p);

      std::string gimbal_roll = std::string("gimbal") + s + std::string("_roll_module");
      f  = seg_tf_map.at(gimbal_roll);
      gimbal_roll_origin_from_cog_.at(i) = ((getCog<KDL::Frame>().Inverse() * f).p);
    }
  setEdfsOriginFromCog(f_edfs);

  setGimbalNominalAngles(gimbal_nominal_angles);
  setGimbalProcessedJoint(gimbal_processed_joint);
  setRollLockedGimbal(roll_locked_gimbal);

}

Eigen::VectorXd FullVectoringRobotModel::calcFeasibleControlFxyDists(const std::vector<int>& roll_locked_gimbal, const std::vector<double>& locked_angles, int rotor_num, const std::vector<Eigen::Matrix3d>& link_rot)
{
  /* only consider F_x and F_y */

  std::vector<Eigen::Vector2d> u(0);

  int gimbal_lock_index = 0;
  for (int i = 0; i < rotor_num; ++i)
    {
      if(roll_locked_gimbal.at(i) == 0)
        {
          // ominidirectional: 2DoF
          u.push_back(Eigen::Vector2d(1, 0)); // from the tilted x force
          u.push_back(Eigen::Vector2d(0, 1)); // from the tilted y force
        }
      else
        {
          // lock gimbal roll: 1DOF
          Eigen::Vector3d gimbal_roll_z_axis = link_rot.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(locked_angles.at(gimbal_lock_index), 0, 0)) * Eigen::Vector3d(0, 0, 1);
          u.push_back(Eigen::Vector2d(gimbal_roll_z_axis(0), gimbal_roll_z_axis(1)).normalized());
          gimbal_lock_index++;
        }
    }

  Eigen::VectorXd f_min(u.size()); // f_min_i; i in [0, u.size()]

  for (int i = 0; i < u.size(); ++i)
    {
      double f_min_ij = 0.0;
      for (int j = 0; j < u.size(); ++j)
        {
          if (i == j) continue;
          f_min_ij += fabs(u.at(i).x()*u.at(j).y() - u.at(j).x()*u.at(i).y()); // we omit the norm of u.at(i), since u is unit vector
        }
      f_min(i) = f_min_ij;
    }

  return f_min;
}

Eigen::VectorXd FullVectoringRobotModel::calcFeasibleControlTDists(const std::vector<int>& roll_locked_gimbal, const std::vector<double>& locked_angles, int rotor_num, const std::vector<Eigen::Vector3d>& rotor_pos, const std::vector<Eigen::Matrix3d>& link_rot)
{
  std::vector<Eigen::Vector3d> v(0);

  int gimbal_lock_index = 0;
  for (int i = 0; i < rotor_num; ++i)
    {
      if(roll_locked_gimbal.at(i) == 0)
        {
          // ominidirectional: 3DoF
          v.push_back(rotor_pos.at(i).cross(Eigen::Vector3d(1, 0, 0))); // from the tilted x force
          v.push_back(rotor_pos.at(i).cross(Eigen::Vector3d(0, 1, 0))); // from the tilted y force
          v.push_back(rotor_pos.at(i).cross(Eigen::Vector3d(0, 0, 1))); // from the z force
        }
      else
        {
          // lock gimbal roll: 2DOF
          Eigen::Matrix3d gimbal_roll_rot =  link_rot.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(locked_angles.at(gimbal_lock_index), 0, 0));
          v.push_back(rotor_pos.at(i).cross(gimbal_roll_rot * Eigen::Vector3d(1, 0, 0))); // from the x force
          v.push_back(rotor_pos.at(i).cross(gimbal_roll_rot * Eigen::Vector3d(0, 0, 1))); // from the z force
          gimbal_lock_index++;
        }
    }

  Eigen::VectorXd t_min(v.size() * (v.size() - 1) / 2); // t_min_ij; i in [0, v.size()], i > j

  int t_min_index = 0;

  for (int i = 0; i < v.size(); ++i)
    {
      for (int j = i + 1; j < v.size(); ++j)
        {
          double t_min_ij = 0.0;
          if(v.at(i).cross(v.at(j)).norm() < 1e-5)
            {
              // assume v_i and v_j has same direction, so this is not plane can generate
              t_min_ij = 1e6;
              ROS_DEBUG_NAMED("robot_model", "the direction of v%d and v%d are too close, so no plane can generate by these two vector", i, j);
            }
          else
            {
              for (int k = 0; k < v.size(); ++k)
                {

                  if (i == k || j == k) continue;
                  double v_triple_product = calcTripleProduct(v.at(i), v.at(j), v.at(k));
                  t_min_ij += fabs(v_triple_product);
                }
            }
          t_min(t_min_index) = t_min_ij;

          t_min_index++;
        }
    }

  return t_min;
}


bool FullVectoringRobotModel::calcBestLockGimbalRoll(const std::vector<int>& roll_locked_gimbal, const std::vector<int>& prev_roll_locked_gimbal, const std::vector<double>& prev_angles, std::vector<double>& locked_angles)
{
  const int rotor_num = getRotorNum();
  const auto gimbal_roll_pos = getGimbalRollOriginFromCog<Eigen::Vector3d>();
  const auto link_rot = getLinksRotationFromCog<Eigen::Matrix3d>();

  /****
       Since the position of the force acting point would change according to the gimbal roll,
       there is a diffiretial chain about the roll angle.
       But we here approximate it to the gimbal roll frame along the link axis,
       rather than the true force acting point (gimbal pitch frame).
       The offset is 0.037m, which is a small offset in the optimization problem.
  ****/
  auto rotor_pos = robot_model_for_plan_->getRotorsOriginFromCog<Eigen::Vector3d>();
  for(int i = 0; i < rotor_num; i++)
    {
      if(roll_locked_gimbal.at(i) == 0) continue;
      rotor_pos.at(i) = gimbal_roll_pos.at(i);
    }


  /* nonlinear optimization for vectoring angles planner */
  const int lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);
  /* TODO: use nonlinear differentiable optimization methods, such as SQP */
  nlopt::opt nl_solver(nlopt::LN_COBYLA, lock_num);
  nl_solver.set_max_objective(minimumControlWrench, this);
  nl_solver.set_xtol_rel(1e-4); //1e-4
  nl_solver.set_maxeval(1000); // 1000 times

  std::vector<double> lb(0);
  std::vector<double> ub(0);

  locked_angles.resize(0);
  for(int i = 0; i < rotor_num; i++)
    {
      const int lock_status = roll_locked_gimbal.at(i);
      const int prev_lock_status = prev_roll_locked_gimbal.at(i);
      const double prev_angle = prev_angles.at(2*i);

      if(!lock_status) continue;

      double margin = gimbal_delta_angle_;
      if(lock_status != prev_lock_status) margin = M_PI/2;

      locked_angles.push_back(prev_angle);
      lb.push_back(prev_angle - margin);
      ub.push_back(prev_angle + margin);
    }

  nl_solver.set_lower_bounds(lb);
  nl_solver.set_upper_bounds(ub);

  /* normalized the weight */
  min_force_normalized_weight_ = min_force_weight_ / rotor_num;
  double max_min_torque = calcFeasibleControlTDists(std::vector<int>(rotor_num, 0), std::vector<double>(), rotor_num, rotor_pos, link_rot).minCoeff();
  ROS_DEBUG_STREAM_NAMED("robot_model", "max_min_torque: " << max_min_torque);

  min_torque_normalized_weight_ = min_torque_weight_ / max_min_torque;

  double max_min_control_wrench;
  double start_t = ros::WallTime::now().toSec();
  nlopt::result result = nl_solver.optimize(locked_angles, max_min_control_wrench);

  if (result < 0)
    {
      ROS_WARN_STREAM_NAMED("robot_model", "nlopt failure to solve the optimal lock roll angle");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("robot_model", "nlopt process time: " << ros::WallTime::now().toSec() - start_t);
  ROS_DEBUG_STREAM_NAMED("robot_model", "nlopt: opt result: " << max_min_control_wrench);

  std::stringstream ss;
  for(auto angle: locked_angles) ss << angle << ", ";
  ROS_INFO_STREAM_NAMED("robot_model", "nlopt: locked angles: " << ss.str());

  const auto f_min_list = calcFeasibleControlFxyDists(roll_locked_gimbal, locked_angles, rotor_num, link_rot);
  const auto t_min_list = calcFeasibleControlTDists(roll_locked_gimbal, locked_angles, rotor_num, rotor_pos, link_rot);

  ROS_DEBUG_NAMED("robot_model", "opt F min: %f, opt T min: %f", f_min_list.minCoeff(), t_min_list.minCoeff());

  return true;
}

void FullVectoringRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // TODO: redundant, but necessary for calculate external wrench comp thrust static for current mode.
}

void FullVectoringRobotModel::addCompThrustToStaticThrust()
{
  vectoring_thrust_ = wrench_comp_thrust_ + hover_vectoring_f_;

  Eigen::VectorXd static_thrust = getStaticThrust();
  for(int i = 0; i < getRotorNum(); i++)
    static_thrust(i) = vectoring_thrust_.segment(3 * i, 3).norm();
  setStaticThrust(static_thrust);
}

void FullVectoringRobotModel::calcJointTorque(const bool update_jacobian)
{
  const auto& sigma = getRotorDirection();
  const auto& joint_positions = getJointPositions();
  const auto& thrust_coord_jacobians = getThrustCoordJacobians();
  const auto& inertia_map = getInertiaMap();
  const int joint_num = getJointNum();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  if(update_jacobian)
    calcBasicKinematicsJacobian(); // update thrust_coord_jacobians_

  Eigen::VectorXd joint_torque = Eigen::VectorXd::Zero(joint_num);

  // update coord jacobians for cog point and convert to joint torque
  std::vector<Eigen::MatrixXd> cog_coord_jacobians;
  for(const auto& inertia : inertia_map)
    {
      cog_coord_jacobians.push_back(RobotModel::getJacobian(joint_positions, inertia.first, inertia.second.getCOG()));
      joint_torque -= cog_coord_jacobians.back().rightCols(joint_num).transpose() * inertia.second.getMass() * (-getGravity());
      // auto t = cog_coord_jacobians.back().rightCols(joint_num).transpose() * inertia.second.getMass() * (-getGravity());
      // ROS_INFO_STREAM("segment: " << inertia.first << "; cog torque: " << t.head(10).transpose());
    }
  setCOGCoordJacobians(cog_coord_jacobians); // TODO: should not update jacobian here

  // ROS_INFO_STREAM_THROTTLE(1.0, "cog joint torque: " << joint_torque.transpose());

  // thrust
  for (int i = 0; i < rotor_num; ++i) {
    joint_torque -= thrust_coord_jacobians.at(i).topRows(3).rightCols(joint_num).transpose() * hover_vectoring_f_.segment(3 * i, 3);
  }

  setJointTorque(joint_torque);
}

bool FullVectoringRobotModel::stabilityCheck(bool verbose)
{
  return aerial_robot_model::RobotModel::stabilityCheck(verbose);
}

void FullVectoringRobotModel::getShortestPath(double& roll_angle, const double prev_roll_angle, \
                                              double& pitch_angle, const double prev_pitch_angle)
{
  double roll_diff = roll_angle - prev_roll_angle;
  double pitch_diff = pitch_angle - prev_pitch_angle;

  // 1. solve the problem of discontinuity of radian
  roll_angle -= std::round(roll_diff / (2 * M_PI)) * 2 * M_PI;
  pitch_angle -= std::round(pitch_diff / (2 * M_PI)) * 2 * M_PI;

  // 2. solve the dual solution issue of 2-DoF gimbal
  // there are two option to achieve the same thrust direction (roll, pitch, roll) ad (roll + PI, pitch + PI)
  roll_diff = roll_angle - prev_roll_angle;
  pitch_diff = pitch_angle - prev_pitch_angle;
  if (fabs(roll_diff) > M_PI/2)
    {
      if (roll_diff > M_PI/2) roll_angle -= M_PI;
      if (roll_diff < -M_PI/2) roll_angle += M_PI;

      if (pitch_angle > 0) pitch_angle = M_PI - pitch_angle;
      else pitch_angle = -M_PI - pitch_angle;
    }

  // 3. solve the problem of discontinuity of radian again
  roll_diff = roll_angle - prev_roll_angle;
  pitch_diff = pitch_angle - prev_pitch_angle;
  if (roll_diff > M_PI) roll_angle -= 2 * M_PI;
  if (roll_diff < - M_PI) roll_angle += 2 * M_PI;
  if (pitch_diff > M_PI) pitch_angle -= 2 * M_PI;
  if (pitch_diff < - M_PI) pitch_angle += 2 * M_PI;
}

void FullVectoringRobotModel::updateJointTorqueMatrices(RobotModelPtr robot_model, \
                                                              const KDL::JntArray& gimbal_processed_joint, \
                                                              const std::vector<Eigen::Matrix3d>& links_rotation_from_cog, \
                                                              const std::vector<int>& roll_locked_gimbal, \
                                                              const std::vector<double>& gimbal_nominal_angles, \
                                                              const double thrust_force_weight, const double joint_torque_weight,  \
                                                              Eigen::MatrixXd& A1, Eigen::VectorXd& b1, Eigen::MatrixXd& Psi)
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
    Eigen::MatrixXd cog_coord_jacobian = robot_model->getJacobian(gimbal_processed_joint, inertia.first, inertia.second.getCOG());
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
  Eigen::MatrixXd W1 = thrust_force_weight * Eigen::MatrixXd::Identity(f_ndof, f_ndof);
  Eigen::MatrixXd W2 = joint_torque_weight * Eigen::MatrixXd::Identity(link_joint_num, link_joint_num);
  Psi = (W1 + A1.transpose() * W2 * A1).inverse();
}

void FullVectoringRobotModel::compensateJointTorque(const Eigen::MatrixXd& A1, const Eigen::MatrixXd& Psi, \
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

void FullVectoringRobotModel::graspControl(const Eigen::MatrixXd& A1_fr, const Eigen::MatrixXd& A2_fr, const Eigen::VectorXd& extra_joint_torque)
{
  // internal wrench
  // determine the direction and point for grasping.

  if (getSegmentsTf().size() == 0) return;

  using orig = aerial_robot_model::transformable::RobotModel;
  const KDL::JntArray gimbal_processed_joint = getGimbalProcessedJoint<KDL::JntArray>();
  const int joint_num = getJointNum();
  const int link_joint_num = getLinkJointIndices().size();
  const int rotor_num = getRotorNum();
  const int fr_ndof = 3 * rotor_num;

  const int fc_num =  rotor_num / 2;
  const int fc_ndof = 1; //3 * fe_num;

  Eigen::MatrixXd A1_fe_all = Eigen::MatrixXd::Zero(joint_num, fc_ndof);
  for (int i = 0; i < fc_num; i++) {

    std::string name = std::string("link") + std::to_string((i + 1) *2) + std::string("_foot");

    Eigen::MatrixXd jac
      = (orig::getJacobian(gimbal_processed_joint, name)).topRows(3);

    // Contact normal: radial direction from origin
    auto pos = getSegmentTf(name).p;
    pos.z(0); // make the postion vector lateral to get the direciton of this force
    pos.Normalize(); // nomrlize the force direction
    Eigen::Vector3d normal = aerial_robot_model::kdlToEigen(pos);

    A1_fe_all -= jac.rightCols(joint_num).transpose() * normal;
  }

  // only consider link joint
  Eigen::MatrixXd A1_fe = Eigen::MatrixXd::Zero(link_joint_num, fc_ndof);
  int cnt = 0;
  for(int i = 0; i < joint_num; i++) {
    if(getJointNames().at(i) == getLinkJointNames().at(cnt))
      {
        A1_fe.row(cnt) = A1_fe_all.row(i);
        cnt++;
      }
    if(cnt == link_joint_num) break;
  }


  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(link_joint_num, fr_ndof + fc_ndof);
  A1.leftCols(fr_ndof) = A1_fr;
  A1.rightCols(fc_ndof) = A1_fe;

  Eigen::VectorXd b1 = extra_joint_torque;

  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6, fr_ndof + fc_ndof);
  A2.leftCols(fr_ndof) = A2_fr;

  Eigen::MatrixXd W1 = Eigen::MatrixXd::Zero(fr_ndof + fc_ndof, fr_ndof + fc_ndof);
  W1.topLeftCorner(fr_ndof, fr_ndof) = thrust_force_weight_ * Eigen::MatrixXd::Identity(fr_ndof, fr_ndof);
  Eigen::MatrixXd W2 = joint_torque_weight_ * Eigen::MatrixXd::Identity(link_joint_num, link_joint_num);

  // 4. use thrust force and joint torque, cost and constraint for joint torque
  OsqpEigen::Solver qp_solver;
  qp_solver.settings()->setVerbosity(false);
  qp_solver.settings()->setWarmStart(true);
  qp_solver.data()->setNumberOfVariables(fr_ndof + fc_ndof);
  qp_solver.data()->setNumberOfConstraints(6 + fc_ndof);

  /*
    cost function:
    f^T W1 f + (A1 f + b1)^T W2 (A1 f + b1)
    = f^T (W1 + A1^T W2 A1) f + 2 b1^T A1 f + cons
  */
  Eigen::MatrixXd hessian = W1 + A1.transpose() * W2 * A1;
  Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
  Eigen::VectorXd gradient = b1.transpose() * W2 * A1;
  qp_solver.data()->setHessianMatrix(hessian_sparse);
  qp_solver.data()->setGradient(gradient);

  /* equality constraint: zero total wnrech */
  Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(6 + fc_ndof, fr_ndof + fc_ndof);
  constraints.topRows(6) = A2;
  constraints(6, fr_ndof) = 1;
  Eigen::SparseMatrix<double> constraint_sparse = constraints.sparseView();
  qp_solver.data()->setLinearConstraintsMatrix(constraint_sparse);

  Eigen::VectorXd b = Eigen::VectorXd::Zero(6 + fc_ndof);
  Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(6 + fc_ndof);
  lower_bound(6) = 0.1; // contact force TODO: parameter
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(6 + fc_ndof);
  upper_bound(6) = 1e6;
  qp_solver.data()->setLowerBound(lower_bound);
  qp_solver.data()->setUpperBound(upper_bound);

  std::string prefix("[Grasp]");
  if(!qp_solver.initSolver()) {
    ROS_ERROR_STREAM(prefix << " can not initialize qp solver");
    return;
  }

  double s_t = ros::Time::now().toSec();
  bool res = qp_solver.solve();
  ROS_INFO_STREAM_ONCE(prefix << " QP solve time: " << ros::Time::now().toSec() - s_t);

  if(!res) {
    ROS_ERROR_STREAM(prefix << "can not solve QP");
    return;
  }

  Eigen::VectorXd f_all = qp_solver.getSolution();
  Eigen::VectorXd fr = f_all.head(fr_ndof);
  double fc = f_all(fr_ndof);
  Eigen::VectorXd tau = A1 * f_all + b1;

  // ROS_INFO_STREAM_THROTTLE(1.0, prefix << " Thrust force for grasp: " << fr.transpose());
  // ROS_INFO_STREAM_THROTTLE(1.0, prefix << " Contact force for grasp: " << fc);
  // ROS_INFO_STREAM_THROTTLE(1.0, prefix << " Joint Torque: " << tau.transpose());

  ROS_INFO_STREAM_ONCE(prefix << " Thrust force for grasp: " << fr.transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Contact force for grasp: " << fc);
  ROS_INFO_STREAM_ONCE(prefix << " Joint Torque: " << tau.transpose());
}

void FullVectoringRobotModel::rotorInterfereAvoid(PrimeBoundMap& prime_bound_map, std::vector<int>& roll_locked_gimbal, std::vector<double>& gimbal_nominal_angles)
{
  int rotor_num = getRotorNum();
  std::string thrust_link = getThrustLinkName();
  const auto& seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  const auto rotors_pos = robot_model_for_plan_->getRotorsOriginFromCog<KDL::Vector>();

  if (seg_tf_map.size() == 0) return;

  double angle_max_thresh = 1.2; // TODO: parameter
  double collision_padding_rate = 1.2; // TODO: parameter
  double roll_angle_thresh = 0.4;  // TODO: parameter

  prime_bound_map = PrimeBoundMap{};
  std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> interfere_raw_map{};
  std::map<int, std::vector<std::pair<double, double>>> bounds_map{};

  auto tangent_calc = [] (auto r1, auto r2, auto rel_pos)
           {
             double x1 = 0;
             double y1 = 0;
             double x2 = rel_pos.x();
             double y2 = rel_pos.z();

             double b = y1 - y2;

             double a1 = x2 + r2 - x1;
             double l1 = sqrt(pow(a1, 2) + pow(b, 2));
             double phi1 = atan2(r1, l1);
             double theta1 = atan2(a1, b) + phi1;
             theta1 = -theta1;

             double a2 = x2 - r2 - x1;
             double l2 = sqrt(pow(a2, 2) + pow(b, 2));
             double phi2 = atan2(r1, l2);
             double theta2 = atan2(a2, b) - phi2;
             theta2 = -theta2;

             return std::make_pair(theta1, theta2);
           };

  std::stringstream ss_map;
  ss_map << "\n";

  for(int i = 0; i < rotor_num; i++)
    {
      auto pos_i = rotors_pos.at(i); // position w.r.t. CoG

      std::string rotor_i = thrust_link + std::to_string(i + 1);
      auto pose_i = seg_tf_map.at(rotor_i); // pose w.r.t. Baselink (Root)

      std::vector<std::pair<int, Eigen::Vector3d>> bound_list;

      for (int j = 0; j < rotor_num; j++)
        {
          if (i == j) continue;


          std::string rotor_j = thrust_link + std::to_string(j + 1);
          auto pose_j = seg_tf_map.at(rotor_j); // pose w.r.t. Baselink (Root)

          // relativel position from the i-th rotor
          auto rel_pose = pose_i.Inverse() * pose_j;
          auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

          // skip if the j-th rotor is not align in the same plane;
          double roll_angle = atan2(fabs(rel_pos.y()), fabs(rel_pos.z()));
          if (roll_angle > roll_angle_thresh) continue;

          double pitch_angle = atan2(-rel_pos.x(), -rel_pos.z());
          // skip if the j-th rotor is far from i-th
          if (fabs(pitch_angle) > angle_max_thresh) continue;

          // calculate the bound angle
          double r = getEdfRadius() * collision_padding_rate;
          auto res = tangent_calc(r, r, rel_pos);
          double theta1 = res.first;
          double theta2 = res.second;

          Eigen::Vector3d angle_v(pitch_angle, theta1, theta2);
          bound_list.push_back(std::make_pair(j, angle_v));
        }

      // check the overlap with center link if it exists
      std::string name("center_link");
      auto it = seg_tf_map.find(name);
      if (it != seg_tf_map.end())
        {
          auto pose_c = it->second; // pose w.r.t. Baselink (Root)
          auto rel_pose = pose_i.Inverse() * pose_c; // relativel position from the i-th rotor
          auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

          double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle

          if (fabs(angle) < angle_max_thresh)
            {
              // calculate the bound angle
              auto geo = getUrdfModel().getLink(name)->collision->geometry;
              auto type = geo->type;

              double r_r = getEdfRadius() * collision_padding_rate;
              double r_c = 0;

              if (type == urdf::Geometry::CYLINDER)
                {
                  r_c = std::dynamic_pointer_cast<urdf::Cylinder>(geo)->radius;
                }
              else if (type == urdf::Geometry::SPHERE)
                {
                  r_c = std::dynamic_pointer_cast<urdf::Sphere>(geo)->radius;
                }
              else if (type == urdf::Geometry::BOX)
                {
                  auto dim = std::dynamic_pointer_cast<urdf::Box>(geo)->dim;
                  r_c = std::hypot(dim.x, dim.y);
                }
              else
                {
                  ROS_WARN_THROTTLE(1.0, "Currently, urdf geometry type %d is not supported for rotor interference avoidance", type);
                }

              if (r_c > 0)
                {
                  auto res = tangent_calc(r_r, r_c, rel_pos);
                  double theta1 = res.first;
                  double theta2 = res.second;
                  Eigen::Vector3d angle_v(angle, theta1, theta2);
                  bound_list.push_back(std::make_pair(-1, angle_v));
                }
            }
        }

      // check the overlap with extra module if it exists
      int extra_cnt = 0;
      for (const auto& it: getExtraModuleMap())
        {
          auto pose_parent = seg_tf_map.find(it.second.first.getName())->second;
          auto pose_e = pose_parent * it.second.first.getFrameToTip(); // pose w.r.t. Baselink (Root)
          auto rel_pose = pose_i.Inverse() * pose_e; // relativel position from the i-th rotor
          auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

          double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle
          if (fabs(angle) > angle_max_thresh) continue;

          // calculate the bound angle
          double r_e = 0;
          auto dim = it.second.second;
          if (dim(1) == 0 && dim(2) == 0) r_e = dim(0); // sphere
          if (dim(1) > 0 && dim(2) == 0) r_e = dim(1); // clyinder
          if (dim(1) > 0 && dim(2) > 0) r_e = std::hypot(dim(0), dim(1)); // box

          double r_r = getEdfRadius() * collision_padding_rate;

          auto res = tangent_calc(r_r, r_e, rel_pos);
          double theta1 = res.first;
          double theta2 = res.second;

          Eigen::Vector3d angle_v(angle, theta1, theta2);
          // ROS_INFO_STREAM("rotor " << i+1 << ": " << it.first << ": rel_pose" << rel_pos.transpose() << ": angle_v: " << angle_v.transpose());
          extra_cnt ++;
          bound_list.push_back(std::make_pair(-1 - extra_cnt, angle_v));
        }

      if (bound_list.size() == 0) continue;
      interfere_raw_map.insert(std::make_pair(i, bound_list));

      // update bounds
      std::vector<std::pair<double, double>> bounds {std::make_pair(-M_PI/2, M_PI/2)};
      double area_thresh = 0.4; // TODO: rosparam
      for (auto& new_bound : bound_list)
        {
          double theta1, theta2;
          if (new_bound.second(1) < new_bound.second(2))
            {
              theta1 = new_bound.second(1);
              theta2 = new_bound.second(2);
            }
          else
            {
              theta1 = new_bound.second(2);
              theta2 = new_bound.second(1);
            }

          std::vector<std::pair<double, double>> bounds_temp {};
          for (auto& curr_bound: bounds)
            {
              double curr_theta1 = curr_bound.first;
              double curr_theta2 = curr_bound.second;

              // 6 cases:
              if (theta2 < curr_theta1)
                {
                  bounds_temp.push_back(curr_bound);
                }

              if (theta1 < curr_theta1 && theta2 > curr_theta1 && theta2 < curr_theta2)
                {
                  if (curr_theta2 - theta2 < area_thresh) continue;

                  bounds_temp.push_back(std::make_pair(theta2, curr_theta2));
                }

              if (theta1 < curr_theta1 && theta2 > curr_theta2)
                {
                  // skip
                  continue;
                }

              if (theta1 > curr_theta1 && theta2 < curr_theta2)
                {
                  if (theta1 - curr_theta1 > area_thresh)
                    {
                      bounds_temp.push_back(std::make_pair(curr_theta1, theta1));
                    }
                  if (curr_theta2 - theta2 > area_thresh)
                    {
                      bounds_temp.push_back(std::make_pair(theta2, curr_theta2));
                    }
                }

              if (theta1 > curr_theta1 && theta1 < curr_theta2 && theta2 > curr_theta2)
                {
                  if (theta1 - curr_theta1 < area_thresh) continue;

                  bounds_temp.push_back(std::make_pair(curr_theta1, theta1));
                }

              if (theta1 > curr_theta2)
                {
                  bounds_temp.push_back(curr_bound);
                }
            }

          bounds = bounds_temp;
        }
      bounds_map.insert(std::make_pair(i, bounds));

      double lower = 1e6;
      double upper = 0;
      for (const auto& bound: bounds)
        {
          double theta1 = bound.first;
          double theta2 = bound.second;

          if (fabs(theta1) < fabs(lower))
            {
              lower = theta1;
              upper = theta2;
            }

          if (fabs(theta2) < fabs(lower))
            {
              lower = theta2;
              upper = theta1;
            }
        }

      // skip if the range is enough for normal tilting
      if (lower * upper < 0 && fabs(lower) > area_thresh / 2) continue;

      prime_bound_map.insert(std::make_pair(i, std::make_pair(lower, upper)));


      if (roll_locked_gimbal.at(i) == 0)
        {
          roll_locked_gimbal.at(i) = 1;
          gimbal_nominal_angles.at(2 * i) = 0;
        }
    }

  if (interfere_raw_map.size() > 0)
    {
      for (const auto& it: interfere_raw_map)
        {
          int rotor_id = it.first;
          ss_map << "rotor" << rotor_id+1 << ": \n";
          for(const auto& bound: it.second)
            {
              int index = bound.first;
              if (index >= 0)
                {
                  ss_map << "\t rotor" << bound.first + 1 << ": " << bound.second.transpose() << " \n";
                }
              else if (index == -1)
                {
                  ss_map << "\t center link: " << bound.second.transpose() << " \n";
                }
              else
                {
                  ss_map << "\t extra module: " << bound.second.transpose() << " \n";
                }
            }

          for(const auto& bound: bounds_map.at(rotor_id))
            {
              ss_map << "\t candidate bound: (" << bound.first << ", " << bound.second << ") \n";
            }

          auto bound = prime_bound_map.find(rotor_id);
          if (bound != prime_bound_map.end())
            {
              ss_map << "\t prime bound: (" << bound->second.first << ", " << bound->second.second << ") \n";
            }
        }

      // ROS_INFO_STREAM_THROTTLE(1.0, "\033[32m" << ss_map.str() << "\033[0m");
    }
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Dragon::FullVectoringRobotModel, aerial_robot_model::RobotModel);
