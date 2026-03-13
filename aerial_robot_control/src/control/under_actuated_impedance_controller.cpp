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
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &UnderActuatedImpedanceController::jointStateCallback, this);
  joint_cmd_sub_ = nh_.subscribe("joints_ctrl", 1, &UnderActuatedImpedanceController::jointCmdCallback, this);
  pos_cmd_sub_ = nh_.subscribe("pos_cmds", 1, &UnderActuatedImpedanceController::posCmdCallback, this);
  mode_sub_ = nh_.subscribe("imp_mode", 1, &UnderActuatedImpedanceController::modeCallback, this);
  external_wrench_sub_ = nh_.subscribe("external_wrench", 1, &UnderActuatedImpedanceController::addExternalWrenchCallback, this);

  target_thrust_z_term_ = Eigen::VectorXd::Zero(motor_num_);
  target_thrust_roll_term_ = Eigen::VectorXd::Zero(motor_num_);
  target_thrust_pitch_term_ = Eigen::VectorXd::Zero(motor_num_);
  target_thrust_yaw_term_ = Eigen::VectorXd::Zero(motor_num_);

  joint_pos_ = Eigen::VectorXd::Zero(robot_model_->getJointNum());
  joint_vel_ = Eigen::VectorXd::Zero(robot_model_->getJointNum());
  // joint_pos_ = Eigen::VectorXd::Zero(6);
  // joint_vel_ = Eigen::VectorXd::Zero(6); 
  // // target_joint_pos_ = Eigen::VectorXd::Zero(6);
  // target_joint_vel_ = Eigen::VectorXd::Zero(6);
  // target_joint_acc_ = Eigen::VectorXd::Zero(6);

  mode_.data = 0;




  //dynamic reconfigure server
  ros::NodeHandle control_nh(nh_, "controller");
  lqi_server_ = boost::make_shared<dynamic_reconfigure::Server<aerial_robot_control::LQIConfig> >(ros::NodeHandle(control_nh, "lqi"));
  dynamic_reconf_func_lqi_ = boost::bind(&UnderActuatedImpedanceController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

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
        {
          if(optimalGain())
            {
              clampGain();
              publishGain();
            }
          else
            ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
        }
      else
        {
          resetGain();
        }

      loop_rate.sleep();
    }
}


void UnderActuatedImpedanceController::activate()
{
  ControlBase::activate();

  // publish gains in start phase for general multirotor
  if(optimalGain()) {
    clampGain();
    publishGain();
    ROS_INFO_NAMED("LQI gain generator", "LQI gain generator: send LQI gains");
  }
  else {
    ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
  }
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
      // pid_msg_.yaw.total.at(i) =  target_thrust_yaw_term_(i);

      // TODO There is not max_gain_thresh to limit yaw_term's value

      // if(abs(target_thrust_yaw_term(i)) > max_yaw_scale)
      //   {
          //max_yaw_scale = abs(target_thrust_yaw_term(i));
     
        // }
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



bool UnderActuatedImpedanceController::optimalGain()
{
  // referece:
  // M, Zhao, et.al, "Transformable multirotor with two-dimensional multilinks: modeling, control, and whole-body aerial manipulation"
  // Sec. 3.2

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();

  Eigen::MatrixXd P_inv = aerial_robot_model::pseudoinverse(P);


  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(P_inv(i,3) * roll_pitch_weight_(0),  0, P_inv(i,3) * roll_pitch_weight_(2));
      pitch_gains_.at(i) = Eigen::Vector3d(P_inv(i,4) * roll_pitch_weight_(0), 0, P_inv(i,4) * roll_pitch_weight_(2));
      yaw_gains_.at(i) = Eigen::Vector3d(P_inv(i,5) * yaw_weight_(0), 0, P_inv(i,5) * yaw_weight_(2));

    }

  return true;
}

void UnderActuatedImpedanceController::clampGain()
{
  /* avoid the violation of 16int_t range because of spinal::RollPitchYawTerms */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for(int i = 0; i < motor_num_; ++i)
    {
      if(max_roll_p_gain < fabs(roll_gains_.at(i)[0])) max_roll_p_gain = fabs(roll_gains_.at(i)[0]);
      if(max_roll_d_gain < fabs(roll_gains_.at(i)[2])) max_roll_d_gain = fabs(roll_gains_.at(i)[2]);
      if(max_pitch_p_gain < fabs(pitch_gains_.at(i)[0])) max_pitch_p_gain = fabs(pitch_gains_.at(i)[0]);
      if(max_pitch_d_gain < fabs(pitch_gains_.at(i)[2])) max_pitch_d_gain = fabs(pitch_gains_.at(i)[2]);
      if(max_yaw_d_gain < fabs(yaw_gains_.at(i)[2])) max_yaw_d_gain = fabs(yaw_gains_.at(i)[2]);
    }

  double roll_p_gain_scale = 1, roll_d_gain_scale = 1, pitch_p_gain_scale = 1, pitch_d_gain_scale = 1, yaw_d_gain_scale = 1;
  if(max_roll_p_gain > max_gain_thresh)
    {
      //ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max roll p gain violate the range of int16_t: " << max_roll_p_gain);
      roll_p_gain_scale = max_gain_thresh / max_roll_p_gain;
    }
  if(max_roll_d_gain > max_gain_thresh)
    {
      //ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max roll d gain violate the range of int16_t: " << max_roll_d_gain);
      roll_d_gain_scale = max_gain_thresh / max_roll_d_gain;
    }
  if(max_pitch_p_gain > max_gain_thresh)
    {
      //ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max pitch p gain violate the range of int16_t: " << max_pitch_p_gain);
      pitch_p_gain_scale = max_gain_thresh / max_pitch_p_gain;
    }
  if(max_pitch_d_gain > max_gain_thresh)
    {
      //ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max pitch d gain violate the range of int16_t: " << max_pitch_d_gain);
      pitch_d_gain_scale = max_gain_thresh / max_pitch_d_gain;
    }
  if(max_yaw_d_gain > max_gain_thresh)
    {
      //ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max yaw d gain violate the range of int16_t: " << max_yaw_d_gain);
      yaw_d_gain_scale = max_gain_thresh / max_yaw_d_gain;
    }

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i)[0] *= roll_p_gain_scale;
      roll_gains_.at(i)[2] *= roll_d_gain_scale;

      pitch_gains_.at(i)[0] *= pitch_p_gain_scale;
      pitch_gains_.at(i)[2] *= pitch_d_gain_scale;

      yaw_gains_.at(i)[2] *= yaw_d_gain_scale;
    }
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

void UnderActuatedImpedanceController::publishGain()
{
  spinal::RollPitchYawTerms rpy_gain_msg; // to spinal
  rpy_gain_msg.motors.resize(motor_num_);

  for(int i = 0; i < motor_num_; ++i)
    {
      /* to flight controller via rosserial scaling by 1000 */
      rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i)[0] * 1000;
      rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i)[1] * 1000;
      rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i)[2] * 1000;

      rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i)[0] * 1000;
      rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i)[1] * 1000;
      rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i)[2] * 1000;

      rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i)[2] * 1000;
    }
  //rpy_gain_pub_.publish(rpy_gain_msg);
}

void UnderActuatedImpedanceController::cfgLQICallback(aerial_robot_control::LQIConfig &config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if(config.lqi_flag)
    {
      switch(level)
        {
        case Levels::RECONFIGURE_LQI_ROLL_PITCH_P:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of roll and pitch from " << roll_pitch_weight_.x() <<  " to "  << config.roll_pitch_p);
          roll_pitch_weight_.x() = config.roll_pitch_p;
          break;
        case Levels::RECONFIGURE_LQI_ROLL_PITCH_I:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of roll and pitch from " << roll_pitch_weight_.y() <<  " to "  << config.roll_pitch_i);
          roll_pitch_weight_.y() = config.roll_pitch_i;
          break;
        case Levels::RECONFIGURE_LQI_ROLL_PITCH_D:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of roll and pitch from " << roll_pitch_weight_.z() <<  " to "  << config.roll_pitch_d);
          roll_pitch_weight_.z() = config.roll_pitch_d;
          break;
        case Levels::RECONFIGURE_LQI_YAW_P:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of yaw from " << yaw_weight_.x() <<  " to "  << config.yaw_p);
          yaw_weight_.x() = config.yaw_p;
          break;
        case Levels::RECONFIGURE_LQI_YAW_I:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of yaw from " << yaw_weight_.y() <<  " to "  << config.yaw_i);
          yaw_weight_.y() = config.yaw_i;
          break;
        case Levels::RECONFIGURE_LQI_YAW_D:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of yaw from " << yaw_weight_.z() <<  " to "  << config.yaw_d);
          yaw_weight_.z() = config.yaw_d;
          break;
        case Levels::RECONFIGURE_LQI_Z_P:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of z from " << z_weight_.x() <<  " to "  << config.z_p);
          z_weight_.x() = config.z_p;
          break;
        case Levels::RECONFIGURE_LQI_Z_I:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of z from " << z_weight_.y() <<  " to "  << config.z_i);
          z_weight_.y() = config.z_i;
          break;
        case Levels::RECONFIGURE_LQI_Z_D:
          //ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of z from " << z_weight_.z() <<  " to "  << config.z_d);
          z_weight_.z() = config.z_d;
          break;
        default :
          break;
        }

      if (!realtime_update_) {
        // instantly modify gain if no joint angles

        if(optimalGain()) {
          clampGain();
          publishGain();
        }
        else {
          //ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
        }
      }

    }
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
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;

    }

  /* the articulated inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  //p_matrix_pseudo_inverse_inertia_pub_.publish(p_pseudo_inverse_with_inertia_msg);
}

void UnderActuatedImpedanceController::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  if (joint_pos_.size() > 0)
  {
    for (int i = 0; i < state->name.size(); i++)
    {
      // std::cout<<"state size"<<state->name.size()<<std::endl;
      // std::cout<<"pos size"<<joint_pos_.size()<<std::endl;
      // std::cout<<"vel size"<<joint_vel_.size()<<std::endl;
      // std::cout<<"pos size"<<state->position.size()<<std::endl;
      // std::cout<<"vel size"<<state->velocity.size()<<std::endl;
      //joint_name_[i] = state->name[i];
      joint_pos_[i] = state->position[i];
      // joint_vel_[i] = state->velocity[i];
    }
  }
}

void UnderActuatedImpedanceController::jointCmdCallback(const sensor_msgs::JointStateConstPtr& cmd)
{
  for (int i = 0; i < cmd->position.size(); i++)
  {
    target_joint_pos_[i] = cmd->position[i];
    //target_joint_vel_[i] = cmd->velocity[i];
    //target_joint_acc_[i] = cmd->effort[i];
  }
  // std::cout << "tar"<<target_joint_pos_<<std::endl;
}
void UnderActuatedImpedanceController::posCmdCallback(const geometry_msgs::PointConstPtr& cmd)
{
  pos_cmd_ = *cmd;
}

void UnderActuatedImpedanceController::modeCallback(const std_msgs::UInt8ConstPtr& mode)
{
  if (mode_.data != mode->data)
  {
    if (mode->data == 1)
      ROS_INFO_STREAM("Position Control Mode");
    else if (mode->data == 0)
      ROS_INFO_STREAM("Joint Angle Mode");
  }
  mode_ = *mode;
}

void UnderActuatedImpedanceController::addExternalWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
{
  external_wrench_ = *msg;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedImpedanceController, aerial_robot_control::ControlBase);
