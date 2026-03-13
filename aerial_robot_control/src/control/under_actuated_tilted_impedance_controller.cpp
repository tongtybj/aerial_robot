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
<<<<<<< HEAD
=======
  omega_x_ = 0.0;
  omega_y_ = 0.0;
  omega_z_ = 0.0;
>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
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


  // Inerial params
  double uav_mass = robot_model_->getMass();
  // if (est_external_wrench_[5] < -0.3 && est_external_wrench_[5] > -1.3)
  //   mdx_ = 0.8 + 0.3 * (est_external_wrench_[5] + 0.3);
  // else if (est_external_wrench_[5] <= -1.3)
  //   mdx_ = 0.5;
  // else
  //   mdx_ = 0.8;
  double mx = mdx_ * uav_mass;
  // std::cout<<"ext_: "<<est_external_wrench_[5]<<std::endl;
  // std::cout<<"mdx_: "<<mdx_<<std::endl;
  double my = mdy_ * uav_mass;
  double mz = mdz_ * uav_mass;
  Eigen::Matrix3d I = robot_model_->getInertia<Eigen::Matrix3d>();

  //Eigen::Matrix3d Id = Eigen::Matrix3d::Identity();
 
  Eigen::Matrix3d Id = Eigen::Matrix3d::Zero();
  Id(0, 0) = Idx_ * I(0, 0);
  Id(1, 1) = Idy_ * I(1, 1);
  Id(2, 2) = Idz_ * I(2, 2);
  //Eigen::Matrix3d Id = Eigen::Matrix3d::Zero();
  // Id(0, 0) = 0.4;
  // Id(1, 1) = 0.4;
  // Id(2, 2) = 0.8;
  // Control params
  // std::cout<<"idx: "<<Idx_<<std::endl;
  double Kpx = x_y_p_;
  double Kpy = x_y_p_;
  double Kpz = z_p_;
  double Kdx = 2 * x_y_zeta_ * sqrt(x_y_p_);
  double Kdy = 2 * x_y_zeta_ * sqrt(x_y_p_);
  double Kdz = 2 * z_zeta_ * sqrt(z_p_);
  Eigen::MatrixXd Kp = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd Zeta = Eigen::MatrixXd::Zero(3, 3);
  Eigen::MatrixXd Kd = Eigen::MatrixXd::Zero(3, 3);
  Kp.block(0, 0, 2, 2) = roll_pitch_p_ * Eigen::Matrix2d::Identity();
  Kp(2, 2) = yaw_p_;
 
  Zeta.block(0, 0, 2, 2) = roll_pitch_zeta_ * Eigen::Matrix2d::Identity();
  Zeta(2, 2) = yaw_zeta_;
  Kd = 2 * Zeta * Kp.sqrt();
 
  tf::Matrix3x3 cog = estimator_->getOrientation(Frame::COG, estimate_mode_);
  Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R(i, j) = cog[i][j];
    }
  }

  Eigen::VectorXd delta_p =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd delta_v =  Eigen::VectorXd::Zero(6); 
  Eigen::VectorXd acc =  Eigen::VectorXd::Zero(6); 

  // tf::Vector3 target_vel_ = navigator_->getTargetVel();
  // tf::Vector3 target_acc_ = navigator_->getTargetAcc();
  // tf::Vector3 target_omega_ = navigator_->getTargetOmega();
  // tf::Vector3 target_ang_acc_ = navigator_->getTargetAngAcc();


  // pos_ = estimator_->getPos(Frame::COG, estimator_->getEstimateMode());
  // vel_ = estimator_->getVel(Frame::COG, estimator_->getEstimateMode());
  // rpy_ = estimator_->getEuler(Frame::COG, estimator_->getEstimateMode());
  // omega_ = estimator_->getAngularVel(Frame::COG, estimator_->getEstimateMode());
  Eigen::Vector3d omega;
  omega(0) = omega_.x();
  omega(1) = omega_.y();
  omega(2) = omega_.z();
  tf::Vector3 target_omega_cog = cog.inverse() * target_omega_;

  Eigen::Vector3d pe = robot_model_->getPosition("end_frame");
  KDL::Frame cog_frame = robot_model_->getCog<KDL::Frame>();
  Eigen::Vector3d pc = aerial_robot_model::kdlToEigen(cog_frame.p);
  Eigen::Matrix3d Rc = aerial_robot_model::kdlToEigen(cog_frame.M);
  Eigen::Vector3d pc_world = Eigen::Vector3d::Zero();
  pc_world[0] = pos_.x();
  pc_world[1] = pos_.y();
  pc_world[2] = pos_.z();
  Eigen::Vector3d pe_cog = R*Rc.transpose()*(pe-pc);
  Eigen::Vector3d pe_world = pc_world+R*Rc.transpose()*(pe-pc);
<<<<<<< HEAD

  //std::cout<<"pe_world "<<pe_world.transpose()<<std::endl;

  double vx = abs(vel_.x());
  //std::cout<<"contact_count_"<<contact_count_<<std::endl;
  // if (vx < 0.03 && est_external_wrench_(0) < -0.5)
  //   contact_count_++;
  // else 
  //   contact_count_ = 0;


  // if (contact_count_ > 20)
  //   contact_flag_ = true;
  
  // tf::Vector3 target_rpy = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z())) * target_rpy_cog;

  // Eigen::Matrix3d eR = (target_R.transpose() * R - R.transpose() * target_R) / 2;
=======
  // std::cout<<"pe_world "<<pe_world.transpose()<<std::endl;

  double vx = abs(vel_.x());
  // std::cout<<"contact_count_"<<contact_count_<<std::endl;
  if (vx < 0.03 && est_external_wrench_(0) < -0.5)
    contact_count_++;
  // else 
  //   contact_count_ = 0;
  
  // tf::Vector3 target_rpy = tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z())) * target_rpy_cog;

>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
  if (contact_flag_)
  {
    delta_p(0) = pe_world[0] - target_pos_.x();
    delta_p(1) = pe_world[1] - target_pos_.y();
  }
  else
  {
    delta_p(0) = pos_.x() - target_pos_.x();
    delta_p(1) = pos_.y() - target_pos_.y();
  }
<<<<<<< HEAD

=======
>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
  delta_p(2) = pos_.z() - target_pos_.z();
  delta_v(0) = vel_.x() - target_vel_.x();
  delta_v(1) = vel_.y() - target_vel_.y();
  delta_v(2) = vel_.z() - target_vel_.z();
  // acc(0) = target_acc_.x();
  // acc(1) = target_acc_.y();
  // acc(2) = target_acc_.z();
  clampEstExternalWrench();

  double target_acc_x = (1 / mx - 1 / uav_mass) * est_external_wrench_clamped_[0] + (-Kdx * delta_v(0) - Kpx * delta_p(0));
  double target_acc_y = (1 / my - 1 / uav_mass) * est_external_wrench_clamped_[1] + (-Kdy * delta_v(1) - Kpy * delta_p(1));
  double target_acc_z = (1 / mz - 1 / uav_mass) * est_external_wrench_clamped_[2] + (-Kdz * delta_v(2) - Kpz * delta_p(2)) + aerial_robot_estimation::G;



  // double target_acc_x = (-Kdx * delta_v(0) - Kpx * delta_p(0));
  // double target_acc_y = (-Kdy * delta_v(1) - Kpy * delta_p(1));
  // double target_acc_z = (-Kdz * delta_v(2) - Kpy * delta_p(2)) + aerial_robot_estimation::G;


  if (target_acc_x > 1.4)
    target_acc_x = 1.4;
  if (target_acc_x < -1.4)
    target_acc_x = -1.4;

  if (target_acc_y > 1.4)
    target_acc_y = 1.4;
  if (target_acc_y < -1.4)
    target_acc_y = -1.4;

  if (target_acc_z > 15.0)
    target_acc_z = 15.0;
  if (target_acc_z < -15.0)
    target_acc_z = -15.0;

  // std::cout<<"y"<<target_acc_y<<std::endl;
  // std::cout<<"y"<<(1 / my - 1 / uav_mass) * est_external_wrench_clamped_[1]<<std::endl;
  // std::cout<<"y"<<(-Kdx * delta_v(1) - Kpx * delta_p(1));
  // std::cout<<"w"<<acc(1);
  // std::cout<<"delta_p"<<delta_p<<std::endl;
  // std::cout<<"pos_.x()"<<pos_.x()<<std::endl;
  // std::cout<<"target_pos_.x()"<<target_pos_.x()<<std::endl;
  // std::cout<<"vel_.x()"<<vel_.x()<<std::endl;
  // std::cout<<"target_vel_.x()"<<target_vel_.x()<<std::endl;
  tf::Vector3 target_acc_w(target_acc_x,
                          target_acc_y,
                          target_acc_z);

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  Eigen::VectorXd f = robot_model_->getStaticThrust();
  Eigen::VectorXd g = robot_model_->getGravity();
<<<<<<< HEAD
  // Eigen::MatrixXd Q = robot_model_->calcWrenchMatrixOnCoG();

=======
  Eigen::MatrixXd Q = robot_model_->calcWrenchMatrixOnCoG();
  // std::cout<<"static "<<Q * f<<std::endl;
  // std::cout<<"target_acc_z"<<target_acc_z<<std::endl;
  // std::cout<<"target_acc_w.length()"<<target_acc_w.length()<<std::endl;
>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
  Eigen::VectorXd allocate_scales = f / g.norm();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w.length();
  // std::cout<<"est_external_wrench_clamped_"<<est_external_wrench_clamped_<<std::endl; 


 
  target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
  target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));

<<<<<<< HEAD
  
  double rate = pid_controllers_.at(Z).result() / (aerial_robot_estimation::G + 1.5);
  target_thrust_z_term = rate * target_thrust_z_term;
  //std::cout<<"rate "<<1.3*rate<<std::endl;

  // std::cout<<"target_acc_z "<<target_acc_z<<std::endl;
=======
  double rate = pid_controllers_.at(Z).result() / (aerial_robot_estimation::G + 0.5);
  // if (rate > 1.0)
  //   rate = 1.0;
  target_thrust_z_term = rate * target_thrust_z_term;
  std::cout<<"rate "<<rate<<std::endl;

>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
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
  // delta_p(3) = rpy_.x() - target_roll_;
  // delta_p(4) = rpy_.y() - target_pitch_;
  // delta_p(5) = rpy_.z() - navigator_->getTargetRPY().z();
  double alpha = 0.7;
  // filtered_omega_(0) = alpha * omega_.x() + (1 - alpha) * filtered_omega_(0);
  // filtered_omega_(1) = alpha * omega_.y() + (1 - alpha) * filtered_omega_(1);
  // filtered_omega_(2) = alpha * omega_.z() + (1 - alpha) * filtered_omega_(2);
  omega_x_ = alpha * omega_.x() + (1 - alpha) * omega_x_;
  omega_y_ = alpha * omega_.y() + (1 - alpha) * omega_y_;
  omega_z_ = alpha * omega_.z() + (1 - alpha) * omega_z_;
  delta_v(3) = omega_.x() - target_omega_cog.x();
  delta_v(4) = omega_.y() - target_omega_cog.y();
  delta_v(5) = omega_.z() - target_omega_cog.z();
  // acc(3) = target_ang_acc_.x();
  // acc(4) = target_ang_acc_.y();
  // acc(5) = target_ang_acc_.z();
  Eigen::Vector3d tau_cmd = Eigen::Vector3d::Zero();
  // unwrap

 // Eigen::Vector3d euler = R1.eulerAngles(0, 1, 2);
 
<<<<<<< HEAD
  tau_cmd = (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3) + I *(-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  if (tau_cmd(0) > 6.0)
    tau_cmd(0) = 6.0;
  if (tau_cmd(0) < -6.0)
    tau_cmd(0) = -6.0;

  if (tau_cmd(1) > 6.0)
    tau_cmd(1) = 6.0;
  if (tau_cmd(1) < -6.0)
    tau_cmd(1) = -6.0;


  if (tau_cmd(2) > 6.0)
    tau_cmd(2) = 6.0;
  if (tau_cmd(2) < -6.0)
    tau_cmd(2) = -6.0;
  //std::cout<<"mass: "<<uav_mass<<std::endl;
  // std::cout<<"a: "<<target_acc_x <<" "<< target_acc_y<<std::endl;
  //imp_cmd_.full_cmd.force.x = tau_cmd[2];
  // if (tau_cmd[2]>0.5)
  //   tau_cmd[2] = 0.5;
  // if (tau_cmd[2]<-0.5)
  //   tau_cmd[2] = -0.5;
  // std::cout<<"tau_cmd: "<<tau_cmd.transpose()<<std::endl;
    //tau_cmd = (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  imp_cmd_.full_cmd.force.x = target_acc_x;
  imp_cmd_.full_cmd.force.y = target_acc_y;
  imp_cmd_.full_cmd.force.z = target_acc_z;
=======
  tau_cmd = (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3) + I * (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  // std::cout<<"tau_cmd: "<<tau_cmd.transpose()<<std::endl;
  // std::cout<<"tau_cmd: "<<uav_mass * (-pe_cog[1] * target_acc_x + pe_cog[0] * target_acc_y)<<std::endl;
  // std::cout<<"a: "<<target_acc_x <<" "<< target_acc_y<<std::endl;
//  if (contact_flag_)
   // tau_cmd[2] += 0.1 * uav_mass * (-pe_cog[1] * target_acc_x + pe_cog[0] * target_acc_y);
  //tau_cmd = (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega;
  imp_cmd_.full_cmd.force.x = (1 / mx - 1 / uav_mass) * est_external_wrench_clamped_[0] + (-Kdx * delta_v(0) - Kpx * delta_p(0));
  imp_cmd_.full_cmd.force.y = (1 / my - 1 / uav_mass) * est_external_wrench_clamped_[1] + (-Kdx * delta_v(1) - Kpx * delta_p(1));
  imp_cmd_.full_cmd.force.z = (1 / mz - 1 / uav_mass) * est_external_wrench_clamped_[2] + (-Kdx * delta_v(2) - Kpx * delta_p(2));
>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d

  imp_cmd_.pd_cmd.force.x = (-Kdx * delta_v(0) - Kpx * delta_p(0));
  imp_cmd_.pd_cmd.force.y = (-Kdy * delta_v(1) - Kpy * delta_p(1));
  imp_cmd_.pd_cmd.force.z = (-Kdz * delta_v(2) - Kpz * delta_p(2));
  imp_cmd_.imp_cmd.force.x = (1 / mx - 1 / uav_mass) * est_external_wrench_clamped_[0];
  imp_cmd_.imp_cmd.force.y = (1 / my - 1 / uav_mass) * est_external_wrench_clamped_[1];
  imp_cmd_.imp_cmd.force.z = (1 / mz - 1 / uav_mass) * est_external_wrench_clamped_[2];
<<<<<<< HEAD
  //imp_cmd_.full_cmd.force.y = 0.3 * uav_mass * (-pe_cog[1] * target_acc_x + pe_cog[0] * target_acc_y);
  //imp_cmd_.full_cmd.force.z = delta_p(5);
  // imp_cmd_.pd_cmd.force.x = euler[0];
  // imp_cmd_.pd_cmd.force.y = euler[1];
  // imp_cmd_.pd_cmd.force.z = euler[2];
=======
  // imp_cmd_.full_cmd.force.x = delta_p(3);
  // imp_cmd_.full_cmd.force.y = delta_p(4);
  // imp_cmd_.full_cmd.force.z = delta_p(5);
  // imp_cmd_.pd_cmd.force.x = rpy_.x() - target_roll_;
  // imp_cmd_.pd_cmd.force.y = rpy_.y() - target_pitch_;
  // imp_cmd_.pd_cmd.force.z = rpy_.z() - navigator_->getTargetRPY().z();
 // imp_cmd_.pd_cmd.force.z = euler[2];
>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  
  tao_ = P*target_thrust_roll_term_+P*target_thrust_pitch_term_+P*target_thrust_yaw_term_+P*target_thrust_z_term;
  // std::cout<<"tao_: "<<tao_.transpose()<<std::endl;
  // std::cout<<P*target_thrust_roll_term_+P*target_thrust_pitch_term_+P*target_thrust_yaw_term_<<std::endl;
  
  Eigen::Vector3d xyz = R * tao_.segment(0, 3);
  Eigen::Vector3d full_cmd = (I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3) + (-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3));
  Eigen::Vector3d pd_cmd = I*(-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3));
  Eigen::Vector3d p_cmd = I*( - Kp * delta_p.segment(3, 3));
  
  Eigen::Vector3d d_cmd = I*(-Kd * delta_v.segment(3, 3));
  
  
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
<<<<<<< HEAD
  // imp_cmd_.pd_cmd.force.x = target_acc_w.length()*rpy_.y();
  // imp_cmd_.pd_cmd.force.y = -target_acc_w.length()*rpy_.x();
  // imp_cmd_.pd_cmd.force.z = tao_(2);
  // imp_cmd_.imp_cmd.force.x = xyz(0)/uav_mass;
  // imp_cmd_.imp_cmd.force.y = xyz(1)/uav_mass;
  // imp_cmd_.imp_cmd.force.z = xyz(2)/uav_mass;


  // Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  //std::cout<<"P "<<P<<std::endl;
  //use KKT to solve rotation thrust
  Eigen::MatrixXd W = Eigen::Matrix3d::Identity() * 0.2;
  Eigen::MatrixXd H = P.bottomRows(3).transpose() * P.bottomRows(3) + P.topRows(3).transpose() * W * P.topRows(3);
  Eigen::VectorXd b = P.bottomRows(3).transpose() * tau_cmd;

  Eigen::VectorXd sol = H.ldlt().solve(b);
=======


  // std::cout<<"dtheta"<<Kp * delta_p.segment(3, 3)<<std::endl;

  // std::cout<<"tau_cmd"<<tau_cmd<<std::endl;
  // std::cout<<"y"<<(I * Id.inverse() - Eigen::Matrix3d::Identity()) * est_external_wrench_clamped_.segment(3, 3)<<std::endl;
  // std::cout<<"z"<<(-Kd * delta_v.segment(3, 3) - Kp * delta_p.segment(3, 3)) + aerial_robot_model::skew(omega) * I * omega<<std::endl;
  // Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  //Eigen::MatrixXd P_rot_inv = aerial_robot_model::pseudoinverse(P.bottomRows(3));
  Eigen::MatrixXd P_rot_inv = aerial_robot_model::pseudoinverse(P.bottomRows(4)).block(0, 1, 4, 3);
>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
  
  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cod(P.topRows(3));
  //Eigen::MatrixXd P_rot_inva = aerial_robot_model::pseudoinverse(-P.bottomRows(3) * cod.matrixZ());
  Eigen::MatrixXd P_rot_inv = aerial_robot_model::pseudoinverse(P.bottomRows(3));
  Eigen::MatrixXd P_rot_inva = aerial_robot_model::pseudoinverse(P.bottomRows(4)).block(0, 1, 4, 3);
  Eigen::MatrixXd P_rot_invt = aerial_robot_model::pseudoinverse(P).block(0, 3, 4, 3);
  //Eigen::MatrixXd P_rot_invb = aerial_robot_model::pseudoinverse(P).block(0, 3, 4, 3);
  // Eigen::VectorXd target_total_thrust = P_inv.col(3) * u(0) + P_inv.col(4) * u(1) + P_inv.col(5) * u(2);
  // std::cout<<"target_thrust_roll_term_"<<sol<<std::endl;
  //  std::cout<<"ta_"<<tau_cmd<<std::endl;
  //  std::cout<<"target_"<<(P*sol).segment(3,3)<<std::endl;
  // std::cout<<"dtheta"<<P_rot_inv*tau_cmd<<std::endl;
  // std::cout<<"dthe"<<P_rot_invt*tau_cmd<<std::endl;
  // target_thrust_roll_term_ = sol;
  target_thrust_roll_term_ = P_rot_inv.col(0) * tau_cmd(0);
  target_thrust_pitch_term_ = P_rot_inv.col(1) * tau_cmd(1);
<<<<<<< HEAD
  target_thrust_yaw_term_ = P_rot_inv.col(2) * tau_cmd(2); 

  Eigen::Vector3d add_force = (P*target_thrust_roll_term_+P*target_thrust_pitch_term_+P*target_thrust_yaw_term_).segment(0, 3);
  Eigen::Vector3d add_force_world = R * add_force;
 
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  t(2) = target_acc_w.length() * uav_mass;
  target_wrench_cog_.segment(0, 3) = add_force + t;
  // imp_cmd_.pd_cmd.force.x = add_force_world[0]/uav_mass;
  // imp_cmd_.pd_cmd.force.y = add_force_world[1]/uav_mass;
  // imp_cmd_.pd_cmd.force.z = add_force_world[0]/full_cmd(2);
  // std::cout<<"tar"<<tau_cmd<<std::endl;
  // std::cout<<"dtheta"<<Kp * delta_p.segment(3, 3)<<std::endl;
  target_wrench_cog_.segment(3, 3) = tau_cmd;
=======
  target_thrust_yaw_term_ = P_rot_inv.col(2) * tau_cmd(2);
  Eigen::Vector3d add_force = (P*target_thrust_roll_term_+P*target_thrust_pitch_term_+P*target_thrust_yaw_term_).segment(0, 3);
  Eigen::Vector3d add_force_world = R * add_force;

  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  t(2) = target_acc_w.length() * uav_mass;
  //target_wrench_cog_.segment(0, 3) = add_force + t;
  target_wrench_cog_.segment(0, 3) = t;
  target_wrench_cog_.segment(3, 3) = tau_cmd;

>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d
  // std::cout<<"tau_cmd "<<tau_cmd<<std::endl;
  // std::cout<<"P "<<(P*target_thrust_roll_term_+P*target_thrust_pitch_term_+P*target_thrust_yaw_term_+P*target_thrust_z_term)<<std::endl;
  // std::cout<<"P "<<P*target_thrust_z_term<<std::endl;
  // std::cout<<"roll_ "<<target_thrust_roll_term_ <<std::endl;
  // std::cout<<"pitch_ "<<target_thrust_pitch_term_ <<std::endl;
  // std::cout<<"yaw_ "<<target_thrust_yaw_term_ <<std::endl;
  // std::cout<<"tau"<<tau_cmd<<std::endl;
  // std::cout<<"P "<<P <<std::endl;
  // std::cout<<"wrench"<<P * (target_thrust_roll_term_ + target_thrust_pitch_term_ + target_thrust_yaw_term_)<<std::endl;

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
     target_base_thrust_.at(0) = target_base_thrust_.at(0);
     target_base_thrust_.at(1) = target_base_thrust_.at(1);
     target_base_thrust_.at(2) = target_base_thrust_.at(2);
     target_base_thrust_.at(3) = target_base_thrust_.at(3);

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
<<<<<<< HEAD
=======

>>>>>>> 9dc48a1715533e59d0ea7771f1561de774bd206d

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedTiltedImpedanceController, aerial_robot_control::ControlBase);