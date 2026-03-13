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

#include <aerial_robot_control/control/under_actuated_tilted_impedance_controller.h>

using namespace aerial_robot_control;

void UnderActuatedTiltedImpedanceController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  UnderActuatedImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_baselink_rot_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);
  z_limit_ = pid_controllers_.at(Z).getLimitSum();
  pid_controllers_.at(Z).setLimitSum(1e6); // do not clamp the sum of PID terms for z axis
  target_wrench_cog_ = Eigen::VectorXd::Zero(6);
  est_external_wrench_clamped_ = Eigen::VectorXd::Zero(6);
  tao_ = Eigen::VectorXd::Zero(6);

  omega_x_ = 0.0;
  omega_y_ = 0.0;
  omega_z_ = 0.0;

  contact_flag_sub_ = nh_.subscribe("contact_flag", 1, &UnderActuatedTiltedImpedanceController::contactFlagCallback, this);
}

void UnderActuatedTiltedImpedanceController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  spinal::FourAxisCommandImpedance flight_command_impedance_data;
  flight_command_data.angles[0] = rpy_.x();
  flight_command_data.angles[1] = rpy_.y();
  flight_command_data.angles[2] = 0.0;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_command_impedance_data.angles[0] = target_roll_;
  flight_command_impedance_data.angles[1] = target_pitch_;
  flight_command_impedance_data.angles[2] = target_rpy_.z();
  flight_command_impedance_data.base_thrust = target_base_thrust_;
  flight_command_impedance_data.z_thrust = target_z_thrust_;
  flight_command_impedance_data.roll_thrust = target_roll_thrust_;
  flight_command_impedance_data.pitch_thrust = target_pitch_thrust_;
  flight_command_impedance_data.yaw_thrust = target_yaw_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
  flight_impedance_cmd_pub_.publish(flight_command_impedance_data);
}


void UnderActuatedTiltedImpedanceController::controlCore()
{
  PoseLinearController::controlCore();
  // Mass params
  double uav_mass = robot_model_->getMass();
 
  Eigen::Vector3d md = Eigen::Vector3d::Zero();
  md(0) = mdx_ * uav_mass;
  md(1) = mdy_ * uav_mass;
  md(2) = mdz_ * uav_mass;

  // Inertia params
  Eigen::Matrix3d I = robot_model_->getInertia<Eigen::Matrix3d>();
 
  Eigen::Matrix3d Id = Eigen::Matrix3d::Zero();
  Id(0, 0) = Idx_ * I(0, 0);
  Id(1, 1) = Idy_ * I(1, 1);
  Id(2, 2) = Idz_ * I(2, 2);

  // Control gains 
  // Translational gains
  Eigen::Vector3d Kpt = Eigen::Vector3d::Zero();
  Kpt(0) = x_y_p_;
  Kpt(1) = x_y_p_;
  Kpt(2) = z_p_;

  Eigen::Vector3d Kdt = Eigen::Vector3d::Zero();
  Kdt(0) = 2 * x_y_zeta_ * sqrt(x_y_p_);
  Kdt(1) = 2 * x_y_zeta_ * sqrt(x_y_p_);
  Kdt(2) = 2 * z_zeta_ * sqrt(z_p_);
  // Rotational gains
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd Zeta = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(3, 3);
  Kp.block(0, 0, 2, 2) = roll_pitch_p_ * Eigen::Matrix2d::Identity();
  Kp(2, 2) = yaw_p_;

  Zeta.block(0, 0, 2, 2) = roll_pitch_zeta_ * Eigen::Matrix2d::Identity();
  Zeta(2, 2) = yaw_zeta_;
  Kd = 2 * Zeta * Kp.sqrt();
  // Cauclate rotation matrix from world to CoG
  tf::Matrix3x3 cog = estimator_->getOrientation(Frame::COG, estimate_mode_);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R(i, j) = cog[i][j];
    }
  }

  Eigen::VectorXd delta_p = Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd delta_v = Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd acc = Eigen::VectorXd::Zero(6); 

  
  Eigen::Vector3d omega;
  omega(0) = omega_.x();
  omega(1) = omega_.y();
  omega(2) = omega_.z();
  tf::Vector3 target_omega_cog = cog.inverse() * target_omega_;

  // TODO ee-centric control
  // Eigen::Vector3d pe = robot_model_->getPosition("end_frame");
  // KDL::Frame cog_frame = robot_model_->getCog<KDL::Frame>();
  // Eigen::Vector3d pc = aerial_robot_model::kdlToEigen(cog_frame.p);
  // Eigen::Matrix3d Rc = aerial_robot_model::kdlToEigen(cog_frame.M);
  // Eigen::Vector3d pc_world = Eigen::Vector3d::Zero();
  // pc_world[0] = pos_.x();
  // pc_world[1] = pos_.y();
  // pc_world[2] = pos_.z();
  // Eigen::Vector3d pe_cog = R*Rc.transpose()*(pe-pc);
  // Eigen::Vector3d pe_world = pc_world+R*Rc.transpose()*(pe-pc);


  // if (contact_flag_)
  // {
  //   delta_p(0) = pe_world[0] - target_pos_.x();
  //   delta_p(1) = pe_world[1] - target_pos_.y();
  // }
  // else
  // {

  delta_p(0) = pos_.x() - target_pos_.x();
  delta_p(1) = pos_.y() - target_pos_.y();
  

  delta_p(2) = pos_.z() - target_pos_.z();
  delta_v(0) = vel_.x() - target_vel_.x();
  delta_v(1) = vel_.y() - target_vel_.y();
  delta_v(2) = vel_.z() - target_vel_.z();
  clampEstExternalWrench();

  // Calculate target acceleration in world frame
  Eigen::VectorXd acc_cmd = Eigen::VectorXd::Zero(3);
  for (int i = 0; i < 3; i++)
    acc_cmd(i) = (1 / md(i) - 1 / uav_mass) * est_external_wrench_clamped_[i] + (-Kdt(i) * delta_v(i) - Kpt(i) * delta_p(i));
  acc_cmd(2) += aerial_robot_estimation::G;
  Eigen::VectorXd limit_t = Eigen::VectorXd::Constant(3, 1.4);
  limit_t(2) = 15.0;
  clampCommand(acc_cmd, limit_t);

  // Calculate thurst for translation
  tf::Vector3 target_acc_w(acc_cmd(0),
                          acc_cmd(1),
                          acc_cmd(2));

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  Eigen::VectorXd f = robot_model_->getStaticThrust();
  Eigen::VectorXd g = robot_model_->getGravity();
  Eigen::VectorXd allocate_scales = f / g.norm();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w.length();
  // Slowly take off
  double rate = pid_controllers_.at(Z).result() / (aerial_robot_estimation::G + 1.5);
  target_thrust_z_term = rate * target_thrust_z_term;
 

  target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
  target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
  
  if(navigator_->getForceLandingFlag())
  {
    target_pitch_ = 0;
    target_roll_ = 0;
  }

  Eigen::Matrix3d target_R = (Eigen::AngleAxisd(navigator_->getTargetRPY().z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(target_pitch_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(target_roll_, Eigen::Vector3d::UnitX())).toRotationMatrix();
  Eigen::Matrix3d eR = (target_R.transpose() * R - R.transpose() * target_R) / 2;
  delta_p(3) = (eR(2, 1) - eR(1, 2)) / 2;
  delta_p(4) = (eR(0, 2) - eR(2, 0)) / 2;
  delta_p(5) = (eR(1, 0) - eR(0, 1)) / 2;
  double alpha = 0.7;
  omega_x_ = alpha * omega_.x() + (1 - alpha) * omega_x_;
  omega_y_ = alpha * omega_.y() + (1 - alpha) * omega_y_;
  omega_z_ = alpha * omega_.z() + (1 - alpha) * omega_z_;
  delta_v(3) = omega_.x() - target_omega_cog.x();
  delta_v(4) = omega_.y() - target_omega_cog.y();
  delta_v(5) = omega_.z() - target_omega_cog.z();
  Eigen::VectorXd tau_cmd = Eigen::VectorXd::Zero(3);
  tau_cmd = (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3) + I *(-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  Eigen::VectorXd limit_r = Eigen::VectorXd::Constant(3, 6.0);
  clampCommand(tau_cmd, limit_r);

  //Calculate command for impedance control
  imp_cmd_.full_cmd.force.x = (uav_mass / md(0) - 1) * est_external_wrench_clamped_[0] + uav_mass * (-Kdt(0) * delta_v(0) - Kpt(0) * delta_p(0));
  imp_cmd_.full_cmd.force.y = (uav_mass / md(1) - 1) * est_external_wrench_clamped_[1] + uav_mass * (-Kdt(1) * delta_v(1) - Kpt(1) * delta_p(1));
  imp_cmd_.full_cmd.force.z = (uav_mass / md(2) - 1) * est_external_wrench_clamped_[2] + uav_mass * (-Kdt(2) * delta_v(2) - Kpt(2) * delta_p(2));
  imp_cmd_.pd_cmd.force.x = uav_mass * (-Kdt(0) * delta_v(0) - Kpt(0) * delta_p(0));
  imp_cmd_.pd_cmd.force.y = uav_mass * (-Kdt(1) * delta_v(1) - Kpt(1) * delta_p(1));
  imp_cmd_.pd_cmd.force.z = uav_mass * (-Kdt(2) * delta_v(2) - Kpt(2) * delta_p(2));
  imp_cmd_.imp_cmd.force.x = (uav_mass / md(0) - 1) * est_external_wrench_clamped_[0];
  imp_cmd_.imp_cmd.force.y = (uav_mass / md(1) - 1) * est_external_wrench_clamped_[1];
  imp_cmd_.imp_cmd.force.z = (uav_mass / md(2) - 1) * est_external_wrench_clamped_[2];

  Eigen::Vector3d full_cmd = (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3) + (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3));
  Eigen::Vector3d pd_cmd = I*(-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3));  
  Eigen::Vector3d imp_cmd = (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3);
  imp_cmd_.full_cmd.torque.x = tau_cmd(0);
  imp_cmd_.full_cmd.torque.y = tau_cmd(1);
  imp_cmd_.full_cmd.torque.z = tau_cmd(2);
  imp_cmd_.pd_cmd.torque.x = pd_cmd(0);
  imp_cmd_.pd_cmd.torque.y = pd_cmd(1);
  imp_cmd_.pd_cmd.torque.z = pd_cmd(2);
  imp_cmd_.imp_cmd.torque.x = imp_cmd(0);
  imp_cmd_.imp_cmd.torque.y = imp_cmd(1);
  imp_cmd_.imp_cmd.torque.z = imp_cmd(2);

 // Calculate thrust for rotation
  Eigen::MatrixXd Q = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd Q_rot_inv = aerial_robot_model::pseudoinverse(Q.bottomRows(3));
 
  target_thrust_roll_term_ = Q_rot_inv.col(0) * tau_cmd(0);
  target_thrust_pitch_term_ = Q_rot_inv.col(1) * tau_cmd(1);
  target_thrust_yaw_term_ = Q_rot_inv.col(2) * tau_cmd(2);
   
  // Calculate desired wrench on CoG (for external wrench estimation)
  Eigen::Vector3d add_force = (Q*target_thrust_roll_term_+Q*target_thrust_pitch_term_+Q*target_thrust_yaw_term_).segment(0, 3);
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  t(2) = target_acc_w.length() * uav_mass;
  target_wrench_cog_.segment(0, 3) = add_force + t;
  target_wrench_cog_.segment(3, 3) = tau_cmd;


  // constraint z (also  I term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - z_limit_;
  
  if(residual > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
      target_thrust_z_term *= (1 - residual / max_term);
    }
  for(int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = target_thrust_z_term(i) + target_thrust_roll_term_(i) + target_thrust_pitch_term_(i) + target_thrust_yaw_term_(i);
      target_z_thrust_.at(i) = target_thrust_z_term(i);
      target_roll_thrust_.at(i) = target_thrust_roll_term_(i);
      target_pitch_thrust_.at(i) = target_thrust_pitch_term_(i);
      target_yaw_thrust_.at(i) = target_thrust_yaw_term_(i);
      pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
    }
    //  target_base_thrust_.at(0) = target_base_thrust_.at(0);
    //  target_base_thrust_.at(1) = target_base_thrust_.at(1);
    //  target_base_thrust_.at(2) = target_base_thrust_.at(2);
    //  target_base_thrust_.at(3) = target_base_thrust_.at(3);

  Eigen::MatrixXd q_mat_inv = getQInv();
  double ff_ang_yaw = navigator_->getTargetAngAcc().z();
  Eigen::VectorXd ff_ang_yaw_term = q_mat_inv.col(3) * ff_ang_yaw;
  target_thrust_yaw_term_ += ff_ang_yaw_term;
  // constraint yaw (also I term)
  int index_yaw;
  double max_yaw_term = target_thrust_yaw_term_.cwiseAbs().maxCoeff(&index_yaw);
  double yaw_residual = max_yaw_term - pid_controllers_.at(YAW).getLimitSum();
  if(yaw_residual > 0)
    {
      pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getPrevErrI());
      target_thrust_yaw_term_ *= (1 - yaw_residual / max_yaw_term);
    }
  // special process for yaw since the bandwidth between PC and spinal
  //std::cout<<"P"<<robot_model_->calcWrenchMatrixOnCoG()<<std::endl;
  // candidate_yaw_term_ = target_thrust_yaw_term_(0);
  candidate_yaw_term_ = 0.0;

  // message for impedance control target command
  imp_command_pub_.publish(imp_cmd_);
}

void UnderActuatedTiltedImpedanceController::clampCommand(Eigen::VectorXd &cmd, const Eigen::Vector3d &limit)
{
  if (cmd.size() != limit.size())
  {
    ROS_ERROR("Size of command and limit do not match!");
    return;
  }
  
  for (int i = 0; i < cmd.size(); i++)
  {
    if (cmd[i] > limit[i])
    {
      cmd[i] = limit[i];
    }
    else if (cmd[i] < -limit[i])
    {
      cmd[i] = -limit[i];
    }
  }
}

void UnderActuatedTiltedImpedanceController::clampEstExternalWrench()
{
  est_external_wrench_clamped_ = est_external_wrench_;
  for (int i = 0; i < 6; i++)
  {
    if (i < 3)
    {
      if (est_external_wrench_clamped_[i] > 20)
      {
        est_external_wrench_clamped_[i] = 20;
      }
      else if (est_external_wrench_clamped_[i] < -20)
      {
        est_external_wrench_clamped_[i] = -20;
      }
      est_external_wrench_clamped_[i] /= 1 + exp(-10*(abs(est_external_wrench_clamped_[i]) - 0.5));
    }
    else
    {
      if (est_external_wrench_clamped_[i] > 6)
      {
        est_external_wrench_clamped_[i] = 6;
      }
      else if (est_external_wrench_clamped_[i] < -6)
      {
        est_external_wrench_clamped_[i] = -6;
      }
      est_external_wrench_clamped_[i] /= 1 + exp(-10*(abs(est_external_wrench_clamped_[i]) - 0.2));
    }
  }
}

bool UnderActuatedTiltedImpedanceController::optimalGain()
{
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_inv = aerial_robot_model::pseudoinverse(P);


  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(0,  0, 0);
      pitch_gains_.at(i) = Eigen::Vector3d(0, 0, 0);
      yaw_gains_.at(i) = Eigen::Vector3d(0, 0, 0);

    }

  return true;
}

void UnderActuatedTiltedImpedanceController::publishGain()
{
  UnderActuatedImpedanceController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_baselink_rot_pub_.publish(coord_msg);
}

void UnderActuatedTiltedImpedanceController::rosParamInit()
{
  UnderActuatedImpedanceController::rosParamInit();

}

void UnderActuatedTiltedImpedanceController::contactFlagCallback(const std_msgs::Empty msg)
{
  contact_flag_ = true;
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedTiltedImpedanceController, aerial_robot_control::ControlBase);