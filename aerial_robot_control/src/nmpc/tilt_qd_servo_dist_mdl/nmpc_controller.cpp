//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_controller.h"

using namespace aerial_robot_control;

void nmpc::TiltQdServoNMPCwITerm::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                             boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                             boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                             boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                             double ctrl_loop_du)
{
  TiltQdServoNMPC::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_du);

  pub_disturb_wrench_ = nh_.advertise<geometry_msgs::WrenchStamped>("disturbance_wrench", 1);
}

void nmpc::TiltQdServoNMPCwITerm::initParams()
{
  TiltQdServoNMPC::initParams();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle nmpc_nh(control_nh, "nmpc");

  /* disturbance rejection using I term */
  double fx_limit, fy_limit, fz_limit, mx_limit, my_limit, mz_limit;
  getParam<double>(nmpc_nh, "limit_fx", fx_limit, 5.0);
  getParam<double>(nmpc_nh, "limit_fy", fy_limit, 5.0);
  getParam<double>(nmpc_nh, "limit_fz", fz_limit, 5.0);
  getParam<double>(nmpc_nh, "limit_mx", mx_limit, 1.0);
  getParam<double>(nmpc_nh, "limit_my", my_limit, 1.0);
  getParam<double>(nmpc_nh, "limit_mz", mz_limit, 1.0);

  double i_gain_x, i_gain_y, i_gain_z, i_gain_roll, i_gain_pitch, i_gain_yaw;
  getParam<double>(nmpc_nh, "i_gain_x", i_gain_x, 1.0);
  getParam<double>(nmpc_nh, "i_gain_y", i_gain_y, 1.0);
  getParam<double>(nmpc_nh, "i_gain_z", i_gain_z, 1.0);
  getParam<double>(nmpc_nh, "i_gain_roll", i_gain_roll, 0.5);
  getParam<double>(nmpc_nh, "i_gain_pitch", i_gain_pitch, 0.5);
  getParam<double>(nmpc_nh, "i_gain_yaw", i_gain_yaw, 0.5);

  double freq = 1.0 / ctrl_loop_du_;
  pos_i_term_[0].initialize(i_gain_x, fx_limit, freq);  // x
  pos_i_term_[1].initialize(i_gain_y, fy_limit, freq);  // y
  pos_i_term_[2].initialize(i_gain_z, fz_limit, freq);  // z

  pos_i_term_[3].initialize(i_gain_roll, mx_limit, freq);   // roll
  pos_i_term_[4].initialize(i_gain_pitch, my_limit, freq);  // pitch
  pos_i_term_[5].initialize(i_gain_yaw, mz_limit, freq);    // yaw
}

void nmpc::TiltQdServoNMPCwITerm::prepareNMPCParams()
{
  /* disturbance rejection */
  calcDisturbWrench();

  std::vector<double> params(mpc_solver_ptr_->NP_ - 4);
  params[0] = dist_force_w_.x;
  params[1] = dist_force_w_.y;
  params[2] = dist_force_w_.z;
  params[3] = dist_torque_cog_.x;
  params[4] = dist_torque_cog_.y;
  params[5] = dist_torque_cog_.z;

  mpc_solver_ptr_->setParameters(params, false);
}

void nmpc::TiltQdServoNMPCwITerm::calcDisturbWrench()
{
  /* update I term */
  tf::Vector3 pos = estimator_->getPos(Frame::COG, estimate_mode_);
  double qw = odom_.pose.pose.orientation.w;
  double qx = odom_.pose.pose.orientation.x;
  double qy = odom_.pose.pose.orientation.y;
  double qz = odom_.pose.pose.orientation.z;

  tf::Vector3 target_pos;
  double qwr, qxr, qyr, qzr;
  if (!is_traj_tracking_)
  {
    target_pos = navigator_->getTargetPos();

    tf::Vector3 target_rpy = navigator_->getTargetRPY();
    tf::Quaternion q_ref = tf::Quaternion();
    q_ref.setRPY(target_rpy.x(), target_rpy.y(), target_rpy.z());
    qwr = q_ref.w();
    qxr = q_ref.x();
    qyr = q_ref.y();
    qzr = q_ref.z();
  }
  else
  {
    target_pos.setX(x_u_ref_.x.data.at(0));
    target_pos.setY(x_u_ref_.x.data.at(1));
    target_pos.setZ(x_u_ref_.x.data.at(2));

    qwr = x_u_ref_.x.data.at(6);
    qxr = x_u_ref_.x.data.at(7);
    qyr = x_u_ref_.x.data.at(8);
    qzr = x_u_ref_.x.data.at(9);
  }

  double qe_w = qw * qwr + qx * qxr + qy * qyr + qz * qzr;
  double qe_x = qwr * qx - qw * qxr + qyr * qz - qy * qzr;
  double qe_y = qwr * qy - qw * qyr - qxr * qz + qx * qzr;
  double qe_z = qxr * qy - qx * qyr + qwr * qz - qw * qzr;

  double sign_qe_w = qe_w > 0 ? 1 : -1;

  double fx_w_i_term = pos_i_term_[0].update(pos.x() - target_pos.x());
  double fy_w_i_term = pos_i_term_[1].update(pos.y() - target_pos.y());
  double fz_w_i_term = pos_i_term_[2].update(pos.z() - target_pos.z());
  double mx_cog_i_term = pos_i_term_[3].update(sign_qe_w * qe_x);
  double my_cog_i_term = pos_i_term_[4].update(sign_qe_w * qe_y);
  double mz_cog_i_term = pos_i_term_[5].update(sign_qe_w * qe_z);

  dist_force_w_ = geometry_msgs::Vector3();
  dist_force_w_.x = fx_w_i_term;
  dist_force_w_.y = fy_w_i_term;
  dist_force_w_.z = fz_w_i_term;
  dist_torque_cog_ = geometry_msgs::Vector3();
  dist_torque_cog_.x = mx_cog_i_term;
  dist_torque_cog_.y = my_cog_i_term;
  dist_torque_cog_.z = mz_cog_i_term;
}

/**
 * @brief callbackViz: publish the predicted trajectory and reference trajectory
 * @param [ros::TimerEvent&] event
 */
void nmpc::TiltQdServoNMPCwITerm::callbackViz(const ros::TimerEvent& event)
{
  TiltQdServoNMPC::callbackViz(event);

  /* disturbance wrench */
  geometry_msgs::WrenchStamped dist_wrench_;
  dist_wrench_.header.frame_id = "beetle1/cog";

  dist_wrench_.wrench.torque = dist_torque_cog_;

  tf::Matrix3x3 rot_mtx_cog2w = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 dist_force_w = tf::Vector3(dist_force_w_.x, dist_force_w_.y, dist_force_w_.z);
  tf::Vector3 dist_force_cog = rot_mtx_cog2w.inverse() * dist_force_w;
  dist_wrench_.wrench.force.x = dist_force_cog.x();
  dist_wrench_.wrench.force.y = dist_force_cog.y();
  dist_wrench_.wrench.force.z = dist_force_cog.z();

  dist_wrench_.header.stamp = ros::Time::now();

  pub_disturb_wrench_.publish(dist_wrench_);
}

void nmpc::TiltQdServoNMPCwITerm::cfgNMPCCallback(NMPCConfig& config, uint32_t level)
{
  TiltQdServoNMPC::cfgNMPCCallback(config, level);

  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;

  if (config.i_gain_flag)
  {
    try
    {
      switch (level)
      {
        case Levels::RECONFIGURE_NMPC_I_GAIN_X: {
          pos_i_term_[0].setIGain(config.i_gain_x);
          ROS_INFO_STREAM("change i_gain_x for NMPC '" << config.i_gain_x << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_Y: {
          pos_i_term_[1].setIGain(config.i_gain_y);
          ROS_INFO_STREAM("change i_gain_y for NMPC '" << config.i_gain_y << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_Z: {
          pos_i_term_[2].setIGain(config.i_gain_z);
          ROS_INFO_STREAM("change i_gain_z for NMPC '" << config.i_gain_z << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_ROLL: {
          pos_i_term_[3].setIGain(config.i_gain_roll);
          ROS_INFO_STREAM("change i_gain_roll for NMPC '" << config.i_gain_roll << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_PITCH: {
          pos_i_term_[4].setIGain(config.i_gain_pitch);
          ROS_INFO_STREAM("change i_gain_pitch for NMPC '" << config.i_gain_pitch << "'");
          break;
        }
        case Levels::RECONFIGURE_NMPC_I_GAIN_YAW: {
          pos_i_term_[5].setIGain(config.i_gain_yaw);
          ROS_INFO_STREAM("change i_gain_yaw for NMPC '" << config.i_gain_yaw << "'");
          break;
        }
        default:
          break;
      }
    }
    catch (std::invalid_argument& e)
    {
      ROS_ERROR_STREAM("NMPC config failed: " << e.what());
    }
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::nmpc::TiltQdServoNMPCwITerm, aerial_robot_control::ControlBase);
