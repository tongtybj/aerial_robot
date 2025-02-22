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

#include <spidar/control/grasp/claw_crane.h>

using namespace extra_plugin::grasp;

ClawCrane::ClawCrane():default_mass_(0), grasp_flag_(false)
{
}

void ClawCrane::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                           RobotModelPtr robot_model,
                           EstimatorPtr  estimator,
                           ControllerPtr controller,
                           NavigatorPtr  navigator)
{
  Base::initialize(nh, nhp, robot_model, estimator, controller, navigator);

  dragon_robot_model_ = boost::dynamic_pointer_cast<DragonRobotModel>(robot_model);
  dragon_controller_  = boost::dynamic_pointer_cast<DragonController>(controller);
  dragon_navigator_   = boost::dynamic_pointer_cast<DragonNavigator>(navigator);

  ros::NodeHandle grasp_nh(nh_, "grasp/claw_crane");
  grasp_nh.param("plan_rate", plan_rate_, 20.0);    // [Hz]
  grasp_nh.param("graso_force", grasp_force_, 1.0); // [N]
  grasp_nh.param("thrust_force_weight", thrust_force_weight_, 1.0);
  grasp_nh.param("joint_torque_weight", joint_torque_weight_, 1.0);


  extra_thrust_force_pub_ = grasp_nh.advertise<std_msgs::Float32MultiArray>("extra_thrust_force", 1);
  total_joint_torque_pub_ = grasp_nh.advertise<std_msgs::Float32MultiArray>("total_joint_torque", 1);

  thread_ = std::thread(boost::bind(&ClawCrane::threadFunc, this));
}

void ClawCrane::update()
{
  // do nothing
}

void ClawCrane::threadFunc()
{
  ros::Rate loop_rate(plan_rate_);

  while(ros::ok())
    {
      thrustControl();

      loop_rate.sleep();
    }
}


void ClawCrane::thrustControl()
{
  if (default_mass_ == 0)
    {
      default_mass_ = robot_model_->getMass();
    }

  // Phase 0: check whether there is extra module need to grasp
  bool find_grasp_obj = false;
  for (const auto& it: robot_model_->getExtraModuleMap())
    {
      auto name = it.first;

      if(name.find("grasp") != std::string::npos)
        {
          find_grasp_obj = true;
          break;
        }
    }

  if (find_grasp_obj)
    {
      if (!grasp_flag_)
        {
          grasp_flag_ = true;
          ROS_INFO("[Claw Crane] start grasping object");
        }
    }

  if (!find_grasp_obj)
    {
      if (grasp_flag_)
        {
          grasp_flag_ = false;
          dragon_controller_->resetExtraThrustForce();
          ROS_INFO("[Claw Crane] stop grasping object");
        }
      return;
    }

  if (default_mass_ == robot_model_->getMass())
    {
      // workaround: wait for the update of vectoring_bounds in other thread
      return;
    }

  // Phase 1 (TODO): decide the grasping configuration according to the geometric infor of target object

  // Phase 2: calculate the hovering thrust with the vectoring bound constrint
  Eigen::MatrixXd full_q_mat = dragon_robot_model_->getVectoringForceWrenchMatrix();

  auto vectoring_bounds = dragon_robot_model_->getVectoringBounds();
  auto roll_locked_gimbal = dragon_robot_model_->getRollLockedGimbal();
  Eigen::VectorXd gravity_force = robot_model_->getGravity() * robot_model_->getMass();
  Eigen::VectorXd hovering_thrust;

  bool ret = allocation::constraint::vectoring(dragon_robot_model_, full_q_mat,
                                               gravity_force,  roll_locked_gimbal, \
                                               vectoring_bounds, hovering_thrust);

  if (!ret) {
    ROS_WARN("[Claw Crane] the constraint based allocation for hovering is invalid");
    return;
  }


  Eigen::MatrixXd A1;
  Eigen::VectorXd b1;
  allocation::updateAllocationMatrices(dragon_robot_model_, A1, b1);
  ROS_INFO_STREAM_ONCE("hovering_thrust: \n" << hovering_thrust.transpose());

  // Phase2.5. calcualte the nominal thrust force and nominal joitn torque against gravity
  Eigen::VectorXd extra_joint_torque = A1 * hovering_thrust + b1;
  ROS_INFO_STREAM_ONCE("extra_joint_torque: \n" << extra_joint_torque.transpose());


  // Phase3. calculate the extra thrust force for grasping
  Eigen::VectorXd extra_thrust;
  optimizeGraspForce(A1, full_q_mat, extra_joint_torque, extra_thrust);

  ROS_INFO_STREAM_ONCE("the extra thrust for grasping: " << extra_thrust.transpose());

  // Phase4. set the extra thrust to controller
  dragon_controller_->addExtraThrustForce(extra_thrust);
}

void ClawCrane::optimizeGraspForce(const Eigen::MatrixXd& A1_fr, const Eigen::MatrixXd& A2_fr, const Eigen::VectorXd& extra_joint_torque, Eigen::VectorXd& extra_thrust)
{
  // internal wrench
  // TODO: determine the direction and point for grasping.

  const auto& seg_tf_map = dragon_robot_model_->getSegmentsTf();
  const auto gimbal_processed_joint = dragon_robot_model_->getGimbalProcessedJoint<KDL::JntArray>();

  const int joint_num = dragon_robot_model_->getJointNum();
  const int link_joint_num = dragon_robot_model_->getLinkJointIndices().size();
  const int rotor_num = dragon_robot_model_->getRotorNum();

  const int fr_ndof = A1_fr.cols();

  const int fc_num =  rotor_num / 2;
  const int fc_ndof = 1; // Theoretically: 3 * fe_num; we only determine the monimal grasping norm forces

  Eigen::MatrixXd A1_fe_all = Eigen::MatrixXd::Zero(joint_num, fc_ndof);
  for (int i = 0; i < fc_num; i++) {

    std::string name = std::string("link") + std::to_string((i + 1) *2) + std::string("_foot");

    Eigen::MatrixXd jac
      = (dragon_robot_model_->TransformableModel::getJacobian(gimbal_processed_joint, name)).topRows(3);

    // Claw Crane Hypothiese:
    // Contact normal: radial direction from origin
    auto pos = seg_tf_map.at(name).p;
    pos.z(0); // make the postion vector lateral to get the direciton of this force
    pos.Normalize(); // nomrlize the force direction
    Eigen::Vector3d normal = aerial_robot_model::kdlToEigen(pos);

    Eigen::MatrixXd jac_dash = jac.rightCols(joint_num).transpose() * normal;

    // ROS_INFO_STREAM("external force" << i+1 << "\n"
    //                 << "normal: " << normal.transpose()  << "\n"
    //                 << "jac_dash: " << jac_dash.transpose());

    A1_fe_all -= jac_dash;
  }

  // only consider link joint
  Eigen::MatrixXd A1_fe = Eigen::MatrixXd::Zero(link_joint_num, fc_ndof);
  int cnt = 0;
  for(int i = 0; i < joint_num; i++) {
    if(dragon_robot_model_->getJointNames().at(i) == dragon_robot_model_->getLinkJointNames().at(cnt))
      {
        A1_fe.row(cnt) = A1_fe_all.row(i);
        cnt++;
      }
    if(cnt == link_joint_num) break;
  }
  // ROS_DEBUG_STREAM_ONCE("A1_fe: \n" << A1_fe);


  Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(link_joint_num, fr_ndof + fc_ndof);
  A1.leftCols(fr_ndof) = A1_fr;
  A1.rightCols(fc_ndof) = A1_fe;

  Eigen::VectorXd b1 = extra_joint_torque;

  Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(6, fr_ndof + fc_ndof);
  A2.leftCols(fr_ndof) = A2_fr;

  Eigen::MatrixXd W1 = Eigen::MatrixXd::Zero(fr_ndof + fc_ndof, fr_ndof + fc_ndof);
  W1.topLeftCorner(fr_ndof, fr_ndof) = thrust_force_weight_ * Eigen::MatrixXd::Identity(fr_ndof, fr_ndof);
  Eigen::MatrixXd W2 = joint_torque_weight_ * Eigen::MatrixXd::Identity(link_joint_num, link_joint_num);

  // ROS_DEBUG_STREAM_ONCE("W1: \n" << W1);
  // ROS_DEBUG_STREAM_ONCE("W2: \n" << W2);

  // ROS_DEBUG_STREAM_ONCE("A1: \n" << A1);
  // ROS_DEBUG_STREAM_ONCE("b1: \n" << b1.transpose());

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
  lower_bound(6) = grasp_force_;
  Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(6 + fc_ndof);
  upper_bound(6) = 1e6;
  qp_solver.data()->setLowerBound(lower_bound);
  qp_solver.data()->setUpperBound(upper_bound);

  // TODO: should also consider that vectoring bounds

  std::string prefix("[Grasp]");
  if(!qp_solver.initSolver()) {
    ROS_ERROR_STREAM(prefix << " can not initialize qp solver");
    extra_thrust = Eigen::VectorXd::Zero(fr_ndof);
    return;
  }

  double s_t = ros::Time::now().toSec();
  bool res = qp_solver.solve();
  ROS_INFO_STREAM_ONCE(prefix << " QP solve time: " << ros::Time::now().toSec() - s_t);

  if(!res) {
    ROS_ERROR_STREAM(prefix << "can not solve QP");
    extra_thrust = Eigen::VectorXd::Zero(fr_ndof);
    return;
  }

  Eigen::VectorXd f_all = qp_solver.getSolution();
  Eigen::VectorXd fr = f_all.head(fr_ndof);
  double fc = f_all(fr_ndof);
  Eigen::VectorXd tau = A1 * f_all + b1;

  ROS_INFO_STREAM_ONCE(prefix << " Thrust force for grasp: " << fr.transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Contact force for grasp: " << fc);
  ROS_INFO_STREAM_ONCE(prefix << " A1_fr * fr: " << (A1_fr * fr).transpose());
  ROS_INFO_STREAM_ONCE(prefix << " A1 * f: " << (A1 * f_all).transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Joint Torque: " << tau.transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Wrench: " << (A2_fr * fr).transpose());

  std_msgs::Float32MultiArray f_msg;
  for(int i = 0; i < fr.size(); i++) f_msg.data.push_back(fr(i));
  extra_thrust_force_pub_.publish(f_msg);

  std_msgs::Float32MultiArray t_msg;
  for(int i = 0; i < tau.size(); i++) t_msg.data.push_back(tau(i));
  total_joint_torque_pub_.publish(t_msg);

  extra_thrust = fr;
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(extra_plugin::grasp::ClawCrane, extra_plugin::Base);




