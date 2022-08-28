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

#include <tiger/control/walk_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control::Tiger;

WalkController::WalkController():
  PoseLinearController(),
  walk_pid_controllers_(0),
  joint_index_map_(0),
  force_joint_control_(false),
  joint_no_load_end_t_(0),
  prev_navi_target_joint_angles_(0)
{
}

void WalkController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();

  tiger_robot_model_ = boost::dynamic_pointer_cast<::Tiger::FullVectoringRobotModel>(robot_model);
  tiger_walk_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::Tiger::WalkNavigator>(navigator);

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);
  target_joint_state_.position.resize(0);
  target_joint_state_.name.resize(0);
  target_vectoring_f_ = Eigen::VectorXd::Zero(3 * motor_num_);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  joint_control_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);

  joint_torque_pub_ = nh_.advertise<spinal::ServoTorqueCmd>("servo/torque_enable", 1);
  joint_yaw_torque_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("joint_yaw/torque_enable", boost::bind(&WalkController::servoTorqueCtrlCallback, this, _1, _2, "yaw"));
  joint_pitch_torque_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("joint_pitch/torque_enable", boost::bind(&WalkController::servoTorqueCtrlCallback, this, _1, _2, "pitch"));

  joint_no_load_sub_ = nh_.subscribe<std_msgs::Empty>("joint_no_load", 1, &WalkController::jointNoLoadCallback, this);
  joint_force_compliance_sub_ = nh_.subscribe<std_msgs::Empty>("joint_force_comliance", 1, &WalkController::jointForceComplianceCallback, this);

  target_joint_state_.name = tiger_robot_model_->getLinkJointNames();
  int joint_num = tiger_robot_model_->getLinkJointNames().size();
  target_joint_state_.position.assign(joint_num, 0);


  // std::stringstream ss;
  // for(auto n: tiger_robot_model_->getLinkJointNames()) ss << n << ", ";
  // ROS_WARN_STREAM("joint names: " << ss.str());
}

void WalkController::rosParamInit()
{
  ros::NodeHandle walk_control_nh(nh_, "controller/walk");
  getParam<double>(walk_control_nh, "joint_ctrl_rate", joint_ctrl_rate_, 1.0); // 1 Hz
  getParam<double>(walk_control_nh, "joint_torque_control_thresh", joint_torque_control_thresh_, 2.0); // 2 Nm
  getParam<double>(walk_control_nh, "servo_angle_bias", servo_angle_bias_, 0.02); // 0.02 rad
  getParam<double>(walk_control_nh, "servo_max_torque", servo_max_torque_, 6.0); // 6.0 Nm
  getParam<double>(walk_control_nh, "servo_torque_change_rate", servo_torque_change_rate_, 1.5); // rate
  ros::NodeHandle xy_nh(walk_control_nh, "xy");
  ros::NodeHandle z_nh(walk_control_nh, "z");

  double limit_sum, limit_p, limit_i, limit_d;
  double limit_err_p, limit_err_i, limit_err_d;
  double p_gain, i_gain, d_gain;

  auto loadParam = [&, this](ros::NodeHandle nh)
    {
      getParam<double>(nh, "limit_sum", limit_sum, 1.0e6);
      getParam<double>(nh, "limit_p", limit_p, 1.0e6);
      getParam<double>(nh, "limit_i", limit_i, 1.0e6);
      getParam<double>(nh, "limit_d", limit_d, 1.0e6);
      getParam<double>(nh, "limit_err_p", limit_err_p, 1.0e6);
      getParam<double>(nh, "limit_err_i", limit_err_i, 1.0e6);
      getParam<double>(nh, "limit_err_d", limit_err_d, 1.0e6);

      getParam<double>(nh, "p_gain", p_gain, 0.0);
      getParam<double>(nh, "i_gain", i_gain, 0.0);
      getParam<double>(nh, "d_gain", d_gain, 0.0);
    };

  loadParam(xy_nh);
  walk_pid_controllers_.push_back(PID("x", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  walk_pid_controllers_.push_back(PID("y", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));

  std::vector<int> indices = {X, Y};
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(xy_nh));
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, indices));

  loadParam(z_nh);
  walk_pid_controllers_.push_back(PID("z", p_gain, i_gain, d_gain, limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d));
  walk_pid_reconf_servers_.push_back(boost::make_shared<PidControlDynamicConfig>(z_nh));
  walk_pid_reconf_servers_.back()->setCallback(boost::bind(&WalkController::cfgPidCallback, this, _1, _2, std::vector<int>(1, Z)));

  // calculate the torque_position Kp
  double angle_scale;
  getParam<double>(nh_, "servo_controller/joints/angle_scale", angle_scale, 1.0);
  double torque_load_scale;
  getParam<double>(nh_, "servo_controller/joints/torque_scale", torque_load_scale, 1.0);
  double load_kp;
  getParam<double>(nh_, "servo_controller/joints/load_kp", load_kp, 1.0); // KP / 128
  tor_kp_ = torque_load_scale * load_kp / angle_scale;
}

bool WalkController::update()
{
  ControlBase::update();

  // skip before the model initialization
  if (tiger_robot_model_->getStaticVectoringF().size() == 0 ||
      tiger_robot_model_->getStaticJointT().size() == 0) {
    return false;
  }

  // set (initialize) joint index map
  if (joint_index_map_.size() == 0) {
    setJointIndexMap();
  }

  if (navigator_->getNaviState() == aerial_robot_navigation::START_STATE) {
    control_timestamp_ = ros::Time::now().toSec();
  }

  // thrust control
  thrustControl();

  // joint control
  jointControl();

  // send control command to robot
  sendCmd();

  return true;
}

void WalkController::thrustControl()
{
  tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_vel = estimator_->getVel(Frame::BASELINK, estimate_mode_);
  tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

  tf::Vector3 baselink_target_pos = tiger_walk_navigator_->getTargetBaselinkPos();
  tf::Vector3 baselink_target_vel = tiger_walk_navigator_->getTargetBaselinkVel();
  tf::Vector3 baselink_target_rpy = tiger_walk_navigator_->getTargetBaselinkRpy();

  // feed-forwared control: compensate the static balance
  Eigen::VectorXd static_thrust_force = tiger_robot_model_->getStaticVectoringF();
  target_vectoring_f_ = static_thrust_force;


  // feed-back control:  baselink position control
  Eigen::VectorXd target_wrench = Eigen::VectorXd::Zero(6);

  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {

    // PoseLinearController::controlCore(); // TODO: no need?

    tf::Vector3 pos_err = baselink_target_pos - baselink_pos;
    tf::Vector3 vel_err = baselink_target_vel - baselink_vel;
    tf::Vector3 rpy_err = baselink_target_rpy - baselink_rpy;

    // time diff
    double du = ros::Time::now().toSec() - control_timestamp_;

    // x
    walk_pid_controllers_.at(X).update(pos_err.x(), du, vel_err.x());

    // y
    walk_pid_controllers_.at(Y).update(pos_err.y(), du, vel_err.y());

    // z
    walk_pid_controllers_.at(Z).update(pos_err.z(), du, vel_err.z());

    // w.r.t. world frame
    tf::Vector3 target_acc(walk_pid_controllers_.at(X).result(),
                           walk_pid_controllers_.at(Y).result(),
                           walk_pid_controllers_.at(Z).result());

    // assign to target wrench
    target_wrench.head(3) = Eigen::Vector3d(target_acc.x(), target_acc.y(), target_acc.z());


    // ros pub
    pid_msg_.x.total.at(0) =  walk_pid_controllers_.at(X).result();
    pid_msg_.x.p_term.at(0) = walk_pid_controllers_.at(X).getPTerm();
    pid_msg_.x.i_term.at(0) = walk_pid_controllers_.at(X).getITerm();
    pid_msg_.x.d_term.at(0) = walk_pid_controllers_.at(X).getDTerm();
    pid_msg_.x.target_p = baselink_target_pos.x();
    pid_msg_.x.err_p = pos_err.x();
    pid_msg_.x.target_d = baselink_target_vel.x();
    pid_msg_.x.err_d = vel_err.x();

    pid_msg_.y.total.at(0) =  walk_pid_controllers_.at(Y).result();
    pid_msg_.y.p_term.at(0) = walk_pid_controllers_.at(Y).getPTerm();
    pid_msg_.y.i_term.at(0) = walk_pid_controllers_.at(Y).getITerm();
    pid_msg_.y.d_term.at(0) = walk_pid_controllers_.at(Y).getDTerm();
    pid_msg_.y.target_p = baselink_target_pos.y();
    pid_msg_.y.err_p = pos_err.y();
    pid_msg_.y.target_d = baselink_target_vel.y();
    pid_msg_.y.err_d = vel_err.y();

    pid_msg_.z.total.at(0) =  walk_pid_controllers_.at(Z).result();
    pid_msg_.z.p_term.at(0) = walk_pid_controllers_.at(Z).getPTerm();
    pid_msg_.z.i_term.at(0) = walk_pid_controllers_.at(Z).getITerm();
    pid_msg_.z.d_term.at(0) = walk_pid_controllers_.at(Z).getDTerm();
    pid_msg_.z.target_p = baselink_target_pos.z();
    pid_msg_.z.err_p = pos_err.z();
    pid_msg_.z.target_d = baselink_target_vel.z();
    pid_msg_.z.err_d = vel_err.z();

    // update
    control_timestamp_ = ros::Time::now().toSec();
  }

  // allocation
  // use all vecotoring angles
  // consider cog and baselink are all level
  const auto rotors_origin = tiger_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto links_rot = tiger_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();

  // Note: only consider the inside links
  Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_ / 2);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  for(int i = 0; i < motor_num_ / 2; i++) {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin.at(2 * i));
    q_mat.middleCols(3 * i, 3) = wrench_map * links_rot.at(2 * i);
  }
  auto q_mat_inv = aerial_robot_model::pseudoinverse(q_mat);
  Eigen::VectorXd fb_vectoring_f = q_mat_inv * target_wrench; // feed-back control
  for(int i = 0; i < motor_num_ / 2; i++) {
    target_vectoring_f_.segment(3 * 2 * i, 3) += fb_vectoring_f.segment(3 * i, 3);
  }
  // ROS_INFO_STREAM_THROTTLE(1.0, "[fb control] fb vectoring f: " << fb_vectoring_f.transpose());


  // target lambda and gimbal angles
  for(int i = 0; i < motor_num_; i++) {
    Eigen::Vector3d f = target_vectoring_f_.segment(3 * i, 3);

    double lambda = f.norm();
    double roll = atan2(-f.y(), f.z());
    double pitch = atan2(f.x(), -f.y() * sin(roll) + f.z() * cos(roll));

    target_base_thrust_.at(i)= lambda;
    target_gimbal_angles_.at(2 * i) = roll;
    target_gimbal_angles_.at(2 * i + 1) = pitch;
  }
}

void WalkController::jointControl()
{
  // basically, use position control for all joints
  auto navi_target_joint_angles = tiger_walk_navigator_->getTargetJointState().position;
  target_joint_state_.position = navi_target_joint_angles;
  if (prev_navi_target_joint_angles_.size() == 0) {
    prev_navi_target_joint_angles_ = navi_target_joint_angles;
  }
  if (navigator_->getNaviState() == aerial_robot_navigation::START_STATE) {
    prev_navi_target_joint_angles_ = navi_target_joint_angles;
  }

  // std::stringstream ss1, ss2;
  // for (int i = 0; i < prev_navi_target_joint_angles_.size(); i++) {
  //   ss1 << prev_navi_target_joint_angles_.at(i) << ", ";
  //   ss2 << navi_target_joint_angles.at(i) << ", ";
  // }
  // ROS_INFO_STREAM("prev_navi_target_joint_angles: " << ss1.str());
  // ROS_INFO_STREAM("navi_target_joint_angles: " << ss2.str());

  // use torque control for joints that needs large torque load
  auto current_angles = getCurrentJointAngles();
  Eigen::VectorXd static_joint_torque = tiger_robot_model_->getStaticJointT();

  const auto& names = target_joint_state_.name;
  auto& target_angles = target_joint_state_.position;
  for(int i = 0; i < names.size(); i++) {

    bool need_torque_control = true;

    double current_angle = current_angles.at(i);
    double navi_target_angle  = navi_target_joint_angles.at(i);
    double prev_navi_target_angle  = prev_navi_target_joint_angles_.at(i);
    double tor = static_joint_torque(i);

    // no torque control if torque is small
    // or has force-position control flag (free leg)
    if (fabs(tor) < joint_torque_control_thresh_) {
      need_torque_control = false;
    }

    // modify the target torque according to the target joint state
    // special algorithm for the additional pulley mechanism with non-final axis encoder
    //if (prev_navi_target_angle != navi_target_angle) { // Bug: too strict for acos func
    if (fabs(prev_navi_target_angle - navi_target_angle) > 1e-4
        && need_torque_control) {

      // the case of inside pitch joint (e.g., joint1_pitch)
      if (tor > 0) {

        double modified_navi_target_angle = navi_target_angle + servo_angle_bias_;

        if (navi_target_angle > prev_navi_target_angle) {
          if (current_angle >= modified_navi_target_angle) {
            // the joint converges
            prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " reaches the new target angle " << navi_target_angle);
          }
          else {
            // increase servo torque to reach the target angle, feedforwardly
            tor = clamp(tor * servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " increase torque.");
          }
        }
        if (navi_target_angle < prev_navi_target_angle) {
          if (current_angle <= modified_navi_target_angle) {
            // the joint converges
            prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " reaches the new target angle " << navi_target_angle);
          }
          else {
            // decrease servo torque to reach the target angle, feedforwardly
            tor = clamp(tor / servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " decrease torque.");
          }
        }

        ROS_WARN("[Tiger][Control] %s not converge. prev target: %f, curr target: %f, modified target: %f, curr pos: %f",
                 names.at(i).c_str(), prev_navi_target_angle, navi_target_angle,
                 modified_navi_target_angle, current_angle);
      }

      // the case of outside pitch joint (e.g., joint2_pitch)
      if (tor < 0) {

        double modified_navi_target_angle = navi_target_angle - servo_angle_bias_;

        if (navi_target_angle < prev_navi_target_angle) {
          if (current_angle <= modified_navi_target_angle) {
            // the joint converges
            prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " reaches the new target angle " << navi_target_angle);
          }
          else {
            // increase servo torque to reach the target angle, feedforwardly
            tor = clamp(tor * servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " increase torque.");
          }
        }

        if (navi_target_angle > prev_navi_target_angle) {

          if (current_angle >= modified_navi_target_angle) {
            // the joint converges
            prev_navi_target_joint_angles_.at(i) = navi_target_joint_angles.at(i);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " reaches the new target angle " << navi_target_angle);
          }
          else {
            // increase servo torque to reach the target angle, feedforwardly
            tor = clamp(tor / servo_torque_change_rate_, servo_max_torque_);
            ROS_INFO_STREAM("[Tiger][Control]" << names.at(i) << " decrease torque.");
          }
        }

        ROS_WARN("[Tiger][Control] %s not converge. prev target: %f, curr target: %f, modified target: %f, curr pos: %f",
                 names.at(i).c_str(), prev_navi_target_angle, navi_target_angle,
                 modified_navi_target_angle, current_angle);
      }
    }

    // compliance to fit the current joint angles
    if (joint_no_load_end_t_ > ros::Time::now().toSec()) {

      if (joint_no_load_end_t_ - ros::Time::now().toSec() < 0.1) {
        force_joint_control_ = false;
      }
      else {
        force_joint_control_ = true;
      }

      need_torque_control = true;
      tor = 0;
    }

    if (!need_torque_control) continue;

    double delta_angle = tor / tor_kp_;

    target_angles.at(i) = current_angle + delta_angle;
  }

}

void WalkController::sendCmd()
{
  PoseLinearController::sendCmd();

  // send ros message for monitoring
  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++) {
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  // send joint compliance command
  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE ||
      force_joint_control_) {

    double st = target_joint_state_.header.stamp.toSec();

    if (ros::Time::now().toSec() - st > 1 / joint_ctrl_rate_) {
      target_joint_state_.header.stamp = ros::Time::now();
      joint_control_pub_.publish(target_joint_state_);
    }
  }

  if (navigator_->getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {

    /* send base throttle command */
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.base_thrust = target_base_thrust_;
    flight_cmd_pub_.publish(flight_command_data);

    /* send gimbal control command */
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();

    for(int i = 0; i < motor_num_ * 2; i++)
      gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));

    gimbal_control_pub_.publish(gimbal_control_msg);
  }
}

bool WalkController::servoTorqueCtrlCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, const std::string& name)
{
  int offset = 0;
  if (name == std::string("yaw")) offset = 0;
  if (name == std::string("pitch")) offset = 1;

  int rotor_num = tiger_robot_model_->getRotorNum();
  spinal::ServoTorqueCmd torque_msg;
  for (int i = 0; i < rotor_num; i++) {
    torque_msg.index.push_back(4 * i + offset);
    torque_msg.torque_enable.push_back(req.data);
  }
  joint_torque_pub_.publish(torque_msg);

  if (!req.data) force_joint_control_ = false;

  return true;
}

void WalkController::jointForceComplianceCallback(const std_msgs::EmptyConstPtr& msg)
{
  force_joint_control_ = true;
}

void WalkController::jointNoLoadCallback(const std_msgs::EmptyConstPtr& msg)
{
  joint_no_load_end_t_ = ros::Time::now().toSec() + 5.0; // 10.0 is a paramter
}

void WalkController::cfgPidCallback(aerial_robot_control::PidControlConfig &config, uint32_t level, std::vector<int> controller_indices)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if(config.pid_control_flag)
    {
      switch(level)
        {
        case Levels::RECONFIGURE_P_GAIN:
          for(const auto& index: controller_indices)
            {
              walk_pid_controllers_.at(index).setPGain(config.p_gain);
              ROS_INFO_STREAM("change p gain for walk PID controller '" << walk_pid_controllers_.at(index).getName() << "'");
            }
          break;
        case Levels::RECONFIGURE_I_GAIN:
          for(const auto& index: controller_indices)
            {
              walk_pid_controllers_.at(index).setIGain(config.i_gain);
              ROS_INFO_STREAM("change i gain for walk PID controller '" << walk_pid_controllers_.at(index).getName() << "'");
            }
          break;
        case Levels::RECONFIGURE_D_GAIN:
          for(const auto& index: controller_indices)
            {
              walk_pid_controllers_.at(index).setDGain(config.d_gain);
              ROS_INFO_STREAM("change d gain for walk PID controller '" << walk_pid_controllers_.at(index).getName() << "'");
            }
          break;
        default :
          break;
        }
    }
}


// utils
void WalkController::setJointIndexMap()
{
  const auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  const auto& search_v = current_joint_state.name;

  joint_index_map_.resize(0); // resize
  const auto& joint_names = target_joint_state_.name;
  for(const auto& n: joint_names) {

    auto res = std::find(search_v.begin(), search_v.end(), n);

    if (res == search_v.end()) {
      ROS_ERROR_STREAM("[Tiger] joint index mapping, cannot find " << n);
      continue;
    }

    auto id = std::distance(search_v.begin(), res);

    joint_index_map_.push_back(id);
  }
}

std::vector<double> WalkController::getCurrentJointAngles()
{
  const auto current_joint_state = tiger_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  std::vector<double> angles(0);
  for(const auto id: joint_index_map_) {
    angles.push_back(current_joint_state.position.at(id));
  }

  return angles;
}

bool WalkController::samejointAngles(std::vector<double> group_a, std::vector<double> group_b)
{
  bool res = true;

  if (group_a.size() != group_b.size()) {
    ROS_ERROR_STREAM("[Tiger][Control] compare joint angles between two groups, but the sizes are different. " <<  group_a.size() << "VS " << group_b.size());
    return false;
  }

  for (int i = 0; i < group_a.size(); i++) {
    if (group_a.at(i) != group_b.at(i)) {
        res = false;
        break;
      }
  }

  return res;
}

void WalkController::reset()
{
  PoseLinearController::reset();
  prev_navi_target_joint_angles_.resize(0);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::Tiger::WalkController, aerial_robot_control::ControlBase);
