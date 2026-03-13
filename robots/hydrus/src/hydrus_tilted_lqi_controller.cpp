#include <hydrus/hydrus_tilted_lqi_controller.h>

using namespace aerial_robot_control;

HydrusTiltedLQIController::HydrusTiltedLQIController():
  UnderActuatedTiltedLQIController()
{
}

void HydrusTiltedLQIController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  wrench_estimate_thread_ = boost::thread([this]()
                                          {
                                            ros::Rate loop_rate(100.0);
                                            while(ros::ok())
                                              {
                                                externalWrenchEstimate();
                                                loop_rate.sleep();
                                              }
                                          });
}

bool HydrusTiltedLQIController::checkRobotModel()
{
  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");

      return false;
    }
  return true;
}

void HydrusTiltedLQIController::externalWrenchEstimate()
{
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
     navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

  Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
  auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::HydrusImu>(estimator_->getImuHandler(0));
  if (!imu_handler)
  {
    ROS_ERROR("HydrusImu is null!");
    return;
  }

  tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
  tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);

  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  double mass = robot_model_->getMass();

  Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
  sum_momentum.head(3) = mass * vel_w;
  sum_momentum.tail(3) = inertia * omega_cog;
  Eigen::VectorXd sum_force = Eigen::VectorXd::Zero(6);
  //std::cout<<"acc:"<<acc_w<<std::endl;
  // sum_force.head(3) = mass * acc_w;

  Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
  J_t.topLeftCorner(3,3) = cog_rot;
  Eigen::VectorXd N = mass * robot_model_->getGravity();

  N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);
  Eigen::VectorXd target_wrench_cog = getTargetWrenchCog();

  if(prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum; // not good
    }
  double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;
  integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_ - sum_force) * dt;
  est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);
  Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
  est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.wrench.force.x = est_external_wrench_(0);
  wrench_msg.wrench.force.y = est_external_wrench_(1);
  wrench_msg.wrench.force.z = est_external_wrench_(2);
  wrench_msg.wrench.torque.x = est_external_wrench_(3);
  wrench_msg.wrench.torque.y = est_external_wrench_(4);
  wrench_msg.wrench.torque.z = est_external_wrench_(5);
  estimate_external_wrench_pub_.publish(wrench_msg);
  prev_est_wrench_timestamp_ = ros::Time::now().toSec();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIController, aerial_robot_control::ControlBase);
