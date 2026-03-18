#include <hydrus/hydrus_tilted_impedance_controller.h>
#include <kdl/chainiksolverpos_lma.hpp>

using namespace aerial_robot_control;

HydrusTiltedImpedanceController::HydrusTiltedImpedanceController():
  UnderActuatedTiltedImpedanceController()
{
}
using namespace differential_kinematics;

void HydrusTiltedImpedanceController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  UnderActuatedTiltedImpedanceController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller1/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller2/simulation/command", 1));
  joint_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("servo_controller/joints/controller3/simulation/command", 1));
 
  pos_cmd_.x = -0.3;
  pos_cmd_.y = 0.3;
  pos_cmd_.z = 0.0;

  xd_ddot_ = Eigen::VectorXd::Zero(3);
  xd_dot_ = Eigen::VectorXd::Zero(3);
  xd_ = Eigen::VectorXd::Zero(3);
  xref_ = Eigen::VectorXd::Zero(3);
  Fext_ = Eigen::Vector3d::Zero();
  
  joint_cmd_.name.resize(3);
  joint_cmd_.position.resize(3);
  joint_cmd_.name[0] = "joint1";
  joint_cmd_.name[1] = "joint2";
  joint_cmd_.name[2] = "joint3";
  
  xref_(0) = 1.2; //1.039 // 1.0357
  xref_(1) = 0.0;

  xd_(0) = 1.2; //1.039
  xd_(1) = 0.0;


  init_sum_momentum_ = Eigen::VectorXd::Zero(6);
  integrate_term_ = Eigen::VectorXd::Zero(6);
  est_external_wrench_ = Eigen::VectorXd::Zero(6);

  prev_est_wrench_timestamp_ = 0;
  estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
  ext_force_sub_ = nh_.subscribe("ext_force", 1, &HydrusTiltedImpedanceController::extForceCallback, this);
  ee_pos_pub_ = nh_.advertise<geometry_msgs::Pose>("end_effector_pose", 1);
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  plan_flag_sub_ = nh_.subscribe("plan_start", 1, &HydrusTiltedImpedanceController::planStartCallback, this);
  end_wrench_sub_ = nh_.subscribe("end_wrench", 1, &HydrusTiltedImpedanceController::endWrenchCallback, this);
  wrench_estimate_thread_ = boost::thread([this]()
                                          {
                                            ros::Rate loop_rate(100.0);
                                            while(ros::ok())
                                              {
                                                externalWrenchEstimate();
                                                loop_rate.sleep();
                                              }
                                          });
  time_ = ros::Time::now();
}

bool HydrusTiltedImpedanceController::checkRobotModel()
{
  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("Impedance gain generator", "Impedance gain generator: robot model is not initiliazed");
      return false;
    }

  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("Impedance gain generator", "Impedance gain generator: invalid pose, stability is invalid");

      return false;
    }
  return true;
  
}
void HydrusTiltedImpedanceController::controlCore()
{
  UnderActuatedTiltedImpedanceController::controlCore();

  // Admittance control
  double Ma = ma_;
  double Ca = ca_;
  double Ka = ka_;
  double M1 = robot_model_->getInertiaMap().at("link1").getMass();
  double M2 = robot_model_->getInertiaMap().at("link2").getMass();
  double M3 = robot_model_->getInertiaMap().at("link3").getMass();
  double M4 = robot_model_->getInertiaMap().at("link4").getMass();
  double m = M1+M2+M3+M4;

  double dt = (ros::Time::now() - time_).toSec();
  if (plan_flag_)
  {
    Eigen::Vector3d Fref = Eigen::Vector3d::Zero();
    // filter external force
    double alpha = 0.8;
    Fext_(0) = alpha * est_external_wrench_(0) + (1 - alpha) * Fext_(0);


    if (Fext_(0) > 0.35)
      Fext_(0) = 0.35;
    else if (Fext_(0) < -2.2)
      Fext_(0) = -2.2;
    Fref(0) = fref_;
    //xd_ddot_ = (R.inverse() * Fext - Ka * (xd_ - xref_) - Ca * xd_dot_) / Ma;
    // defferential method to calculate distance 
    xd_ddot_ = ((Fext_ - Fref) - Ka * (xd_ - xref_) - Ca * xd_dot_) / Ma;
    xd_ += xd_dot_ * dt;
    xd_dot_ += xd_ddot_ * dt;
    // if (xd_(0) > 0.95)
    //   xd_(0) = 0.95;
    if (xd_(0) > 1.90)
      xd_(0) = 1.90;
    else if (xd_(0) < 0.80)
      xd_(0) = 0.80;
    geometry_msgs::Pose ee_pose;
    std::cout<<"fext_"<<Fext_(0)<<std::endl;
    std::cout<<"xd_"<<xd_<<std::endl;

    // ---------------CoG--------------------
    // Calculate joint angle from theta

    //double ctheta = (xd_(0)*m-0.6*m+0.3*M4)/(0.3*M3+0.9*M2+1.2*M1);
    double ctheta = (xd_(0)-0.6)/1.2;
    
    // joint_cmd_.position[0] = 1.5708 - std::acos(ctheta);
    // joint_cmd_.position[1] = 2 * std::acos(ctheta);
    // joint_cmd_.position[2] = -std::acos(ctheta);
    joint_cmd_.position[0] = std::acos(ctheta);
    joint_cmd_.position[1] = std::acos(ctheta);
    joint_cmd_.position[2] = -std::acos(ctheta);
    std::cout<<"theta_"<<ctheta<<std::endl;
    // ---------------CoG--------------------
    //sstd::cout<<"theta"<<std::acos(ctheta)<<" "<<std::acos(ctheta)/3.14159*180<<std::endl;
    // publish joint angle command
    joints_ctrl_pub_.publish(joint_cmd_);
    // Distance message
    ee_pose.position.x = xd_(0);
    ee_pose.position.y = Fext_(0);
    ee_pos_pub_.publish(ee_pose);
    time_ = ros::Time::now();
  }
  else
  {
    //std::cout<<"reset admittance"<<std::endl;

    xd_(0) = 1.2; //1.039
    xd_(1) = 0.0;

    xd_dot_ = Eigen::VectorXd::Zero(3);

    time_ = ros::Time::now();
  }
}


void HydrusTiltedImpedanceController::rosParamInit()
{
  UnderActuatedImpedanceController::rosParamInit();
  
  ros::NodeHandle param_nh(nh_, "controller/impedance");
  getParam<double>(param_nh, "mdx", mdx_, 1.0);
  getParam<double>(param_nh, "mdy", mdy_, 1.0);
  getParam<double>(param_nh, "mdz", mdz_, 1.0);
  getParam<double>(param_nh, "Idx", Idx_, 1.0);
  getParam<double>(param_nh, "Idy", Idy_, 1.0);
  getParam<double>(param_nh, "Idz", Idz_, 1.0);
  getParam<double>(param_nh, "x_y_p", x_y_p_, 30.0);
  getParam<double>(param_nh, "x_y_p", x_y_p_, 30.0);
  getParam<double>(param_nh, "z_p", z_p_, 30.0);
  getParam<double>(param_nh, "roll_pitch_p", roll_pitch_p_, 30.0);
  getParam<double>(param_nh, "yaw_p", yaw_p_, 30.0);
  getParam<double>(param_nh, "joints_p", joints_p_, 15.0);
  getParam<double>(param_nh, "pos_p", pos_p_, 10.0);
  getParam<double>(param_nh, "x_y_zeta", x_y_zeta_, 1.2);
  getParam<double>(param_nh, "z_zeta", z_zeta_, 1.0);
  getParam<double>(param_nh, "roll_pitch_zeta", roll_pitch_zeta_, 0.4);
  getParam<double>(param_nh, "yaw_zeta", yaw_zeta_, 0.5);
  getParam<double>(param_nh, "joints_d", joints_d_, 5.0);
  getParam<double>(param_nh, "pos_d", pos_d_, 4.0);
  getParam<double>(param_nh, "ma", ma_, 5.0);
  getParam<double>(param_nh, "ca", ca_, 20.0);
  getParam<double>(param_nh, "ka", ka_, 20.0);
  getParam<double>(param_nh, "fref", fref_, 0.0);

  getParam<double>(param_nh, "kot", kot_, 2.0);
  getParam<double>(param_nh, "kor", kor_, 2.0);

  momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6,6);

  momentum_observer_matrix_(0, 0) *= kot_;
  momentum_observer_matrix_(1, 1) *= kot_;
  momentum_observer_matrix_(2, 2) *= kot_;
  momentum_observer_matrix_.bottomRows(3) *= kor_;

}

void HydrusTiltedImpedanceController::externalWrenchEstimate()
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



void HydrusTiltedImpedanceController::extForceCallback(const geometry_msgs::WrenchConstPtr& cmd)
{
  std::cout<<"*cmd"<<*cmd<<std::endl;
  ext_force_ = *cmd;
}

void HydrusTiltedImpedanceController::planStartCallback(const std_msgs::BoolConstPtr& msg)
{
  plan_flag_ = msg->data;

}

void HydrusTiltedImpedanceController::endWrenchCallback(const geometry_msgs::WrenchStampedConstPtr& cmd)
{
  end_external_wrench_ = *cmd;
}



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedImpedanceController, aerial_robot_control::ControlBase);
