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

#include <aerial_robot_control/control/under_actuated_impedance_controller.h>

using namespace aerial_robot_control;

UnderActuatedImpedanceController::UnderActuatedImpedanceController():
  target_roll_(0), target_pitch_(0), candidate_yaw_term_(0)
{
  roll_pitch_weight_.setZero();
  yaw_weight_.setZero();
  z_weight_.setZero();
}


void UnderActuatedImpedanceController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rosParamInit();


  //publisher

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  flight_impedance_cmd_pub_ = nh_.advertise<spinal::FourAxisCommandImpedance>("four_axes/imp_command", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);
  imp_command_pub_ = nh_.advertise<aerial_robot_msgs::ImpedanceControl>("imp_cmd", 1);


  target_thrust_z_term_ = Eigen::VectorXd::Zero(motor_num_);
  target_thrust_roll_term_ = Eigen::VectorXd::Zero(motor_num_);
  target_thrust_pitch_term_ = Eigen::VectorXd::Zero(motor_num_);
  target_thrust_yaw_term_ = Eigen::VectorXd::Zero(motor_num_);

  joint_pos_ = Eigen::VectorXd::Zero(robot_model_->getJointNum());
  joint_vel_ = Eigen::VectorXd::Zero(robot_model_->getJointNum());
 

  mode_.data = 0;




  //dynamic reconfigure server
  ros::NodeHandle control_nh(nh_, "controller");
  lqi_server_ = boost::make_shared<dynamic_reconfigure::Server<aerial_robot_control::LQIConfig> >(ros::NodeHandle(control_nh, "lqi"));
  

  //gains
  pitch_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));
  roll_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));
  yaw_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));

  //message
  target_base_thrust_.resize(motor_num_);
  target_z_thrust_.resize(motor_num_);
  target_roll_thrust_.resize(motor_num_);
  target_pitch_thrust_.resize(motor_num_);
  target_yaw_thrust_.resize(motor_num_);
  pid_msg_.z.total.resize(motor_num_);
  pid_msg_.z.p_term.resize(motor_num_);
  pid_msg_.z.i_term.resize(motor_num_);
  pid_msg_.z.d_term.resize(motor_num_);
  pid_msg_.yaw.total.resize(motor_num_);
  pid_msg_.yaw.p_term.resize(motor_num_);
  pid_msg_.yaw.i_term.resize(motor_num_);
  pid_msg_.yaw.d_term.resize(motor_num_);



  if (!robot_model_->isModelFixed()) realtime_update_ = true;
  if (realtime_update_) {
    gain_generator_thread_ = std::thread(boost::bind(&UnderActuatedImpedanceController::gainGeneratorFunc, this));
  }


}

UnderActuatedImpedanceController::~UnderActuatedImpedanceController()
{
  if(realtime_update_) gain_generator_thread_.join();
}

void UnderActuatedImpedanceController::gainGeneratorFunc()
{
  double rate;
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  lqi_nh.param("gain_generate_rate", rate, 15.0);
  ros::Rate loop_rate(rate);

  while(ros::ok())
    {
      if(checkRobotModel())
   {}

      loop_rate.sleep();
    }
}


void UnderActuatedImpedanceController::activate()
{
  ControlBase::activate();

  // publish gains in start phase for general multirotor

}

void UnderActuatedImpedanceController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();
  sendRotationalInertiaComp();
}

void UnderActuatedImpedanceController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void UnderActuatedImpedanceController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
  target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;

 
  //feed-forward term for z
  Eigen::MatrixXd q_mat_inv = getQInv();
  double ff_acc_z = navigator_->getTargetAcc().z();
  double ff_ang_yaw = navigator_->getTargetAngAcc().z();
  Eigen::VectorXd ff_acc_z_term = q_mat_inv.col(0) * ff_acc_z;
  Eigen::VectorXd ff_ang_yaw_term = q_mat_inv.col(3) * ff_ang_yaw;
  target_thrust_z_term_ += ff_acc_z_term;
  target_thrust_yaw_term_ += ff_ang_yaw_term;


  // constraint z and yaw (also  I term)
  int index_z, index_yaw;
  double max_z_term = target_thrust_z_term_.cwiseAbs().maxCoeff(&index_z);
  double max_yaw_term = target_thrust_yaw_term_.cwiseAbs().maxCoeff(&index_yaw);
  double z_residual = max_z_term - pid_controllers_.at(Z).getLimitSum();
  double yaw_residual = max_yaw_term - pid_controllers_.at(YAW).getLimitSum();
  if(z_residual > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
      target_thrust_z_term_ *= (1 - z_residual / max_z_term);
    }
  if(yaw_residual > 0)
    {
      pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getPrevErrI());
      target_thrust_yaw_term_ *= (1 - yaw_residual / max_yaw_term);
    }

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for(int i = 0; i < motor_num_; i++)
 
    {
      
      target_base_thrust_.at(i) = target_thrust_z_term_(i);
      pid_msg_.z.total.at(i) =  target_thrust_z_term_(i);
  
    }
   
    candidate_yaw_term_ = target_thrust_yaw_term_(0);
    // std::cout << target_thrust_yaw_term << std::endl;
    // std::cout << candidate_yaw_term_ << std::endl;
    //std::cout << "---------------------------------------" << std::endl;

}


Eigen::MatrixXd UnderActuatedImpedanceController::getQInv()
{
  // wrench allocation matrix
  const std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = robot_model_->getRotorDirection();
  const double m_f_rate = robot_model_->getMFRate();
  double uav_mass_inv = 1.0 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(4, motor_num_);
  for (unsigned int i = 0; i < motor_num_; ++i) {
    q_mat(0, i) = rotors_normal.at(i).z() * uav_mass_inv;
    q_mat.block(1, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
  }
  Eigen::MatrixXd q_mat_inv = aerial_robot_model::pseudoinverse(q_mat);
  return q_mat_inv;
}


bool UnderActuatedImpedanceController::checkRobotModel()
{
  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("Impedance controller", "Impedance controller: robot model is not initiliazed");
      return false;
    }

  return true;
}


void UnderActuatedImpedanceController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  ros::NodeHandle imp_nh(control_nh, "impedance");
  getParam<bool>(lqi_nh, "clamp_gain", clamp_gain_, true);
  getParam<bool>(lqi_nh, "realtime_update", realtime_update_, false);
  getParam<bool>(lqi_nh, "gyro_moment_compensation", gyro_moment_compensation_, false);

  /* propeller direction and lqi R */
  r_.resize(motor_num_); // motor_num is not set
  for(int i = 0; i < robot_model_->getRotorNum(); ++i) {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    getParam<double>(lqi_nh, std::string("r") + ss.str(), r_.at(i), 1.0);
  }

  getParam<double>(imp_nh, "roll_pitch_p", roll_pitch_weight_[0], 1.0);
  getParam<double>(imp_nh, "roll_pitch_i", roll_pitch_weight_[1], 1.0);
  getParam<double>(imp_nh, "roll_pitch_d", roll_pitch_weight_[2], 1.0);
  getParam<double>(imp_nh, "yaw_p", yaw_weight_[0], 1.0);
  getParam<double>(imp_nh, "yaw_i", yaw_weight_[1], 1.0);
  getParam<double>(imp_nh, "yaw_d", yaw_weight_[2], 1.0);
  getParam<double>(imp_nh, "z_p", z_weight_[0], 1.0);
  getParam<double>(imp_nh, "z_i", z_weight_[1], 1.0);
  getParam<double>(imp_nh, "z_d", z_weight_[2], 1.0);
  
}


void UnderActuatedImpedanceController::sendRotationalInertiaComp()
{
  if(!gyro_moment_compensation_) return;

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, 4));

  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg; // to spinal
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for(int i = 0; i < motor_num_; ++i)
    {
      /* the p matrix pseudo inverse and inertia */
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = 0.0;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = 0.0;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0.0;
    }

  /* the articulated inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = 0.0;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = 0.0;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = 0.0;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = 0.0;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = 0.0;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = 0.0;

  //p_matrix_pseudo_inverse_inertia_pub_.publish(p_pseudo_inverse_with_inertia_msg);
}




/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedImpedanceController, aerial_robot_control::ControlBase);
