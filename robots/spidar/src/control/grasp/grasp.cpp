// TODO: create as a classe
void DragonFullVectoringController::graspControl(const KDL::JntArray& gimbal_processed_joint, const Eigen::MatrixXd& A1_fr, const Eigen::MatrixXd& A2_fr)
{
  // Phase 0. 

  Eigen::MatrixXd A1;
  Eigen::VectorXd b1;
  Eigen::MatrixXd Psi;
  const auto links_rotation = aerial_robot_model::kdlToEigen(links_rotation_from_cog);
  allocation::updateJointTorqueMatrices(robot_model_for_plan_,      \
                                        gimbal_processed_joint, links_rotation, \
                                        roll_locked_gimbal, gimbal_nominal_angles, \
                                        thrust_force_weight_, joint_torque_weight_, \
                                        A1, b1, Psi);

  // Phase1. calcualte the nominal thrust force and nominal joitn torque against gravity
  Eigen::VectorXd extra_joint_torque = A1 * hover_vectoring_f + b1;
  graspControl(gimbal_processed_joint, A1, full_q_mat, extra_joint_torque);



  // Phase2. calculate the extra thrust force for grasping
  // find the joint torque:


  // internal wrench
  // determine the direction and point for grasping.

  const auto& seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  if (seg_tf_map.size() == 0) return;

  ROS_INFO_STREAM_ONCE("extra_joint_torque: \n" << extra_joint_torque.transpose());

  const int joint_num = getJointNum();
  const int link_joint_num = getLinkJointIndices().size();
  const int rotor_num = getRotorNum();
  const int fr_ndof = A1_fr.cols();

  const int fc_num =  rotor_num / 2;
  const int fc_ndof = 1; //3 * fe_num;

  Eigen::MatrixXd A1_fe_all = Eigen::MatrixXd::Zero(joint_num, fc_ndof);
  for (int i = 0; i < fc_num; i++) {

    std::string name = std::string("link") + std::to_string((i + 1) *2) + std::string("_foot");

    Eigen::MatrixXd jac
      = (robot_model_for_plan_->getJacobian(gimbal_processed_joint, name)).topRows(3);

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
    if(getJointNames().at(i) == getLinkJointNames().at(cnt))
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
  lower_bound(6) = 1; // contact force TODO: parameter
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
  ROS_INFO_STREAM_ONCE(prefix << " A1_fr * fr: " << (A1_fr * fr).transpose());
  ROS_INFO_STREAM_ONCE(prefix << " A1 * f: " << (A1 * f_all).transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Joint Torque: " << tau.transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Wrench: " << (A2_fr * fr).transpose());
}


// origin
void FullVectoringRobotModel::graspControl(const KDL::JntArray& gimbal_processed_joint, const Eigen::MatrixXd& A1_fr, const Eigen::MatrixXd& A2_fr, const Eigen::VectorXd& extra_joint_torque)
{
  // internal wrench
  // determine the direction and point for grasping.

  const auto& seg_tf_map = robot_model_for_plan_->getSegmentsTf();
  if (seg_tf_map.size() == 0) return;

  ROS_INFO_STREAM_ONCE("extra_joint_torque: \n" << extra_joint_torque.transpose());

  const int joint_num = getJointNum();
  const int link_joint_num = getLinkJointIndices().size();
  const int rotor_num = getRotorNum();
  const int fr_ndof = A1_fr.cols();

  const int fc_num =  rotor_num / 2;
  const int fc_ndof = 1; //3 * fe_num;

  Eigen::MatrixXd A1_fe_all = Eigen::MatrixXd::Zero(joint_num, fc_ndof);
  for (int i = 0; i < fc_num; i++) {

    std::string name = std::string("link") + std::to_string((i + 1) *2) + std::string("_foot");

    Eigen::MatrixXd jac
      = (robot_model_for_plan_->getJacobian(gimbal_processed_joint, name)).topRows(3);

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
    if(getJointNames().at(i) == getLinkJointNames().at(cnt))
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
  lower_bound(6) = 1; // contact force TODO: parameter
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
  ROS_INFO_STREAM_ONCE(prefix << " A1_fr * fr: " << (A1_fr * fr).transpose());
  ROS_INFO_STREAM_ONCE(prefix << " A1 * f: " << (A1 * f_all).transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Joint Torque: " << tau.transpose());
  ROS_INFO_STREAM_ONCE(prefix << " Wrench: " << (A2_fr * fr).transpose());
}
