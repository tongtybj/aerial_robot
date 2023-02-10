#include <hydrus/hydrus_robot_model.h>

using namespace aerial_robot_model;

HydrusRobotModel::HydrusRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double fc_rp_min_thre, double epsilon, int wrench_dof):
  RobotModel(init_with_rosparam, verbose, 0, fc_t_min_thre, epsilon),
  fc_rp_min_thre_(fc_rp_min_thre),
  wrench_dof_(wrench_dof)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  if(wrench_dof_ == 3) setFeasibleControlTMinThre(0);

  fc_rp_dists_.resize(getRotorNum());
  approx_fc_rp_dists_.resize(getRotorNum());

}

void HydrusRobotModel::calcFeasibleControlRollPitchAngAcc()
{
  /* angular acceleration for roll and pitch */
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();

  Eigen::VectorXd dists = Eigen::VectorXd::Zero(getRotorNum());

  auto v = calcV();

  for (auto& v_i: v) {
      Eigen::Vector3d v_temp = v_i;
      v_i = getInertia<Eigen::Matrix3d>().inverse() * v_temp;
      v_i.z() = 0;
    }

  for (int i = 0; i < rotor_num; ++i) {
    double t_min_i = 0.0;
    auto v_i_normalized = v.at(i).normalized();
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double cross_product = (v.at(j).cross(v_i_normalized)).z();
      t_min_i += std::max(0.0, cross_product * thrust_max);
    }
    dists(i) = t_min_i;
  }
  fc_ang_acc_rp_min_ = dists.minCoeff();
}

void HydrusRobotModel::calcFeasibleControlRollPitchDists()
{
  /* only consider Moment for roll and pitch */
  const int rotor_num = getRotorNum();
  const double thrust_max = getThrustUpperLimit();

  auto v = calcV();

  for (auto& v_i: v) v_i.z() = 0;

  for (int i = 0; i < rotor_num; ++i) {
    double t_min_i = 0.0;
    auto v_i_normalized = v.at(i).normalized();
    for (int j = 0; j < rotor_num; ++j) {
      if (i == j) continue;
      double cross_product = (v.at(j).cross(v_i_normalized)).z();

      t_min_i += std::max(0.0, cross_product * thrust_max);
      //ROS_INFO("i: %d, j: %d, cross_product: %f, t_min_ij: %f" , i, j, cross_product, std::max(0.0, cross_product * thrust_max));
    }
    fc_rp_dists_(i) = t_min_i;
  }

  //  ROS_INFO_STREAM("fc_rp_distsj_: " << fc_rp_distsj_.transpose());
  fc_rp_min_ = fc_rp_dists_.minCoeff();
}

void HydrusRobotModel::calcFeasibleControlRollPitchDistsJacobian()
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int ndof = 6 + joint_num;
  const auto& p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& sigma = getRotorDirection();
  const double thrust_max = getThrustUpperLimit();
  const double m_f_rate = getMFRate();
  const auto& u_jacobians = getUJacobians();
  const auto& p_jacobians = getPJacobians();
  const double epsilon = getEpsilon();

  fc_rp_dists_jacobian_.resize(rotor_num, 6 + joint_num);

  auto v = calcV();
  for (auto& v_i: v) v_i.z() = 0; // 2D

  std::vector<Eigen::MatrixXd> v_jacobians;
  for (int i = 0; i < rotor_num; ++i) {
    v_jacobians.push_back(-skew(u.at(i)) * p_jacobians.at(i) + skew(p.at(i)) * u_jacobians.at(i) + m_f_rate * sigma.at(i + 1) * u_jacobians.at(i));
    v_jacobians.back().row(2).setZero(); // 2D
  }

  for (int i = 0; i < rotor_num; ++i)
    {
      const Eigen::Vector3d& v_i = v.at(i);
      const Eigen::Vector3d v_i_normalized = v.at(i).normalized();
      const Eigen::MatrixXd& d_v_i = v_jacobians.at(i);

      double approx_dist = 0.0;
      Eigen::MatrixXd d_dist = Eigen::MatrixXd::Zero(1, ndof);
      for (int j = 0; j < rotor_num; ++j)
        {
          if (i == j) continue;

          const Eigen::Vector3d& v_j = v.at(j);
          const Eigen::MatrixXd& d_v_j = v_jacobians.at(j);
          const double v_cross_product = (v_j.cross(v_i_normalized)).z();
          Eigen::MatrixXd d_v_cross_product = -skew(v_i_normalized) * d_v_j + skew(v_j) * (1 / v_i.norm() * d_v_i - v_i_normalized / v_i.squaredNorm() * v_i.transpose() * d_v_i);
          approx_dist += reluApprox(v_cross_product * thrust_max, epsilon);
          d_dist += sigmoid(v_cross_product * thrust_max, epsilon) * d_v_cross_product.row(2) * thrust_max;
        } //j

      approx_fc_rp_dists_(i) = approx_dist;
      fc_rp_dists_jacobian_.row(i) = d_dist;
    } //i
}

void HydrusRobotModel::calcWrenchMatrixOnRoot()
{
  aerial_robot_model::RobotModel::calcWrenchMatrixOnRoot();
  const auto wrench_mat = getThrustWrenchMatrix();
  setThrustWrenchMatrix(wrench_mat.middleRows(2, wrench_dof_));
}

void HydrusRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // update Q matrix

  Eigen::VectorXd wrench_g = calcGravityWrenchOnRoot();
  const auto& wrench_mat = getThrustWrenchMatrix();

  // under-actuated
  Eigen::VectorXd static_thrust = aerial_robot_model::pseudoinverse(wrench_mat) * (-wrench_g.segment(2, wrench_dof_));
  setStaticThrust(static_thrust);
}

void HydrusRobotModel::getParamFromRos()
{
  ros::NodeHandle nh;
  nh.param("fc_ang_acc_rp_min_thre", fc_ang_acc_rp_min_thre_, 1e-6);
  nh.param("fc_rp_min_thre", fc_rp_min_thre_, 1e-6);
  if(nh.hasParam("wrench_dof"))  nh.getParam("wrench_dof", wrench_dof_);
}



bool HydrusRobotModel::stabilityCheck(bool verbose)
{
  if(!aerial_robot_model::RobotModel::stabilityCheck(verbose)) return false;

  if(fc_ang_acc_rp_min_ < fc_ang_acc_rp_min_thre_)
    {
      if(verbose) ROS_ERROR_STREAM("fc_ang_acc_rp_min " << fc_ang_acc_rp_min_ << " is lower than the threshold " <<  fc_ang_acc_rp_min_thre_);
      return false;
    }

  if(fc_rp_min_ < fc_rp_min_thre_)
    { // only roll & pitch
      if(verbose) ROS_ERROR_STREAM("fc_rp_min " << fc_rp_min_ << " is lower than the threshold " <<  fc_rp_min_thre_);
      return false;
    }

  return true;
}

void HydrusRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);
  calcFeasibleControlRollPitchDists();
  calcFeasibleControlRollPitchAngAcc();
}

void HydrusRobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model)
{
  aerial_robot_model::RobotModel::updateJacobians(joint_positions, update_model);

  calcFeasibleControlRollPitchDistsJacobian();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HydrusRobotModel, aerial_robot_model::RobotModel);
