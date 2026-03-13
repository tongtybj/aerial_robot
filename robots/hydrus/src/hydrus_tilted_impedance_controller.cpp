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
  pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("pos/roll_error", 1));
  pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("pos/pitch_error", 1));
  pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("pos/z", 1));
  target_joint_pos_[0] = 1.57;
  target_joint_pos_[1] = 1.57;
  target_joint_pos_[2] = 1.57;
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
  
  // xref_(0) = 0.9047;
  // xref_(1) = 0.5223;

  // xd_(0) = 0.9047;
  // xd_(1) = 0.5223;
  xref_(0) = 1.039; //1.039 // 1.0357
  xref_(1) = 0.0;

  xd_(0) = 1.039; //1.039
  xd_(1) = 0.0;

  q_result_ = KDL::JntArray(3);
  p_ = 0.0;

  init_sum_momentum_ = Eigen::VectorXd::Zero(6);
  integrate_term_ = Eigen::VectorXd::Zero(6);
  est_external_wrench_ = Eigen::VectorXd::Zero(6);

  prev_est_wrench_timestamp_ = 0;
  estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
  ext_force_sub_ = nh_.subscribe("ext_force", 1, &HydrusTiltedImpedanceController::extForceCallback, this);
  ee_pos_pub_ = nh_.advertise<geometry_msgs::Pose>("end_effector_pose", 1);
  joints_ctrl_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);
  flight_nav_pub_ = nh_.advertise<aerial_robot_msgs::FlightNav>("uav/nav", 1);
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

  double q = 0.01; // Process noise covariance
  double r = 0.01;

  double dt = (ros::Time::now() - time_).toSec();
  if (plan_flag_)
  {
 
      


    // else
    //   contact_flag_ = false;

    if (contact_flag_)
      Ka *= 0.3;
      Ca *= 0.3;
    
    std::cout<<"contact_flag_ "<<contact_flag_<<std::endl;
    std::cout<<"Ka"<<Ka<<std::endl;

    Eigen::AngleAxisd rotation_vector(rpy_.z(), Eigen::Vector3d::UnitZ());
    Eigen::Matrix3d R = rotation_vector.toRotationMatrix(); 

    Eigen::Vector3d Fref = Eigen::Vector3d::Zero();
    // ----------first order--------------------
    double alpha = 0.2;
    Fext_(0) = alpha * est_external_wrench_(0) + (1 - alpha) * Fext_(0);
    //Fext_(0) = 20 * (pos_.x() - target_pos_.x()) + 5 * vel_.x();
    // ------------------------------

    // ----------Kalman--------------------
    // double p_pred = p_ + q;
    // double x_pred = end_external_wrench_.wrench.force.x;
    // double e = est_external_wrench_(0) - x_pred;
    // double k = p_pred/(p_pred + r);
    // Fext_(0) = x_pred + k * e;
    // p_ = (1 - k) * p_pred;
    // ------------------------------

    if (Fext_(0) > 0.35)
      Fext_(0) = 0.35;
    else if (Fext_(0) < -2.2)
      Fext_(0) = -2.2;
    Fref(0) = fref_;
    //xd_ddot_ = (R.inverse() * Fext - Ka * (xd_ - xref_) - Ca * xd_dot_) / Ma;
    xd_ddot_ = ((Fext_ - Fref) - Ka * (xd_ - xref_) - Ca * xd_dot_) / Ma;
    xd_ += xd_dot_ * dt;
    xd_dot_ += xd_ddot_ * dt;
    // if (xd_(0) > 0.95)
    //   xd_(0) = 0.95;
    if (xd_(0) > 1.08)
      xd_(0) = 1.08;
    else if (xd_(0) < 0.94)
      xd_(0) = 0.94;
    geometry_msgs::Pose ee_pose;


    // joint_cmd_pubs_[0].publish(j1_term);
    // joint_cmd_pubs_[1].publish(j2_term);
    // joint_cmd_pubs_[2].publish(j3_term);

    std::cout<<"fext_"<<Fext_(0)<<std::endl;
    std::cout<<"xd_"<<xd_<<std::endl;
    std::cout<<"ma_"<<ma_<<std::endl;
    std::cout<<"fref_"<<fref_<<std::endl;

    // double theta = std::acos(xd_(0)/1.2);
    //  std::cout<<"theta"<<theta;
    // joint_cmd_.position[0] = 2 * theta;
    // joint_cmd_.position[1] = -2 * theta;
    // joint_cmd_.position[2] = 2 * theta;
    // ---------------CoG--------------------
    double ctheta = (xd_(0)*m-0.6*m+0.3*M4)/(0.3*M3+0.9*M2+1.2*M1);
    
    joint_cmd_.position[0] = 1.5708 - std::acos(ctheta);
    joint_cmd_.position[1] = 2 * std::acos(ctheta);
    joint_cmd_.position[2] = -std::acos(ctheta);
    // ---------------CoG--------------------
    std::cout<<"theta"<<std::acos(ctheta)<<" "<<std::acos(ctheta)/3.14159*180<<std::endl;
    joints_ctrl_pub_.publish(joint_cmd_);
    // aerial_robot_msgs::FlightNav nav_msg;
    // nav_msg.header.frame_id = std::string("/world");
    // nav_msg.header.stamp = ros::Time::now();

    // nav_msg.yaw_nav_mode = nav_msg.POS_VEL_MODE;
    // nav_msg.target_yaw = -std::acos(xd_(0)/1.2);
    // flight_nav_pub_.publish(nav_msg);

    ee_pose.position.x = xd_(0);
    ee_pose.position.y = Fext_(0);
    ee_pos_pub_.publish(ee_pose);


    // joint_cmd_.position[1] = 1.57;

    // KDL::Chain kdl_chain;
    // robot_model_->getTree().getChain(std::string("root"), std::string("end_effector"), kdl_chain);
    // KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
    // KDL::Frame target_pose_cog;
    // target_pose_cog.p = KDL::Vector(xd_(0), xd_(1), 0.00);
    // target_pose_cog.M = KDL::Rotation::Identity();

    // KDL::Frame target_pose;
    // target_pose = cog * target_pose_cog;
    // target_pose.M = KDL::Rotation::Identity();

    // ee_pose.position.x = target_pose.p(0);
    // ee_pose.position.y = target_pose.p(1);  
    // ee_pose.position.x = xd_(0);
    // ee_pose.position.y = xd_(1);  
     //std::cout << "q" << aerial_robot_model::kdlToEigen(cog.M) << aerial_robot_model::kdlToEigen(cog.p)<<std::endl;
    // if ( ee_pose.position.x < 0.6)
    // {
    //   std::cout<<"ee_"<<ee_pose<<std::endl;
    //   std::cout<<"ee_"<<xd_(0)<<xd_(1)<<std::endl;
    //   std::cout << "q" << aerial_robot_model::kdlToEigen(cog.M) << aerial_robot_model::kdlToEigen(cog.p)<<std::endl;
    // }
   
    //ee_pos_pub_.publish(ee_pose);
    time_ = ros::Time::now();
  }
  else
  {
    std::cout<<"reset admittance"<<std::endl;
    // joint_cmd_.position[0] = 1.047;
    // joint_cmd_.position[1] = 1.047;
    // joint_cmd_.position[2] = -0.52;
    // joints_ctrl_pub_.publish(joint_cmd_);

    xref_(0) = 1.039; //1.039 // 1.0357
    xref_(1) = 0.0;

    xd_(0) = 1.039; //1.039
    xd_(1) = 0.0;

    xd_dot_ = Eigen::VectorXd::Zero(3);

    time_ = ros::Time::now();
  }


  //std::cout << "q" << aerial_robot_model::kdlToEigen(target_pose.M) << aerial_robot_model::kdlToEigen(target_pose.p)<<std::endl;

  // KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);
  // KDL::JntArray q_init(3); 
  // q_init(0) = 1.047;        
  // q_init(1) = 1.047; 
  // q_init(2) = -0.524; 
  // KDL::Frame fk_ee;
  // if (!inited)
  // {
  //   int a = fk_solver.JntToCart(q_init, fk_ee);
  //   inited = true;
  // }
  // else
  //   int a = fk_solver.JntToCart(q_result_, fk_ee);
 
  // KDL::Frame target_pose_cog;
  // target_pose_cog.p = KDL::Vector(0.6, 1.63, 0.00);           // 位移部分
  // target_pose_cog.M = KDL::Rotation::Identity();
  
  // // KDL::Frame target_pose;
  // // target_pose = cog * target_pose_cog;
  // // target_pose.M = fk_ee.M;
  // KDL::ChainIkSolverPos_LMA ik_solver(kdl_chain);


  // // std::cout << "q" << aerial_robot_model::kdlToEigen(target_pose.M) << aerial_robot_model::kdlToEigen(target_pose.p)<<std::endl;
  //   int status = ik_solver.CartToJnt(q_init, target_pose_cog, q_result_);
  //   if (status >= 0) {
  //       std::cout << "IK 成功，角度解为：" << std::endl;
  //       for (unsigned int i = 0; i < 3; ++i) {
  //           std::cout << "q" << i << " = " << q_result_(i) << " rad" << std::endl;
  //       }
  //   } else {
  //       std::cerr << "IK 求解失败，错误代码：" << status << std::endl;
  //   }

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
  getParam<double>(param_nh, "fref", fref_, -0.3);

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
