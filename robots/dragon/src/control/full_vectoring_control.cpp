#include <dragon/control/full_vectoring_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

DragonFullVectoringController::DragonFullVectoringController():
  PoseLinearController()
{
}

void DragonFullVectoringController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();

  dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::FullVectoringRobotModel>(robot_model);
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::transformable::RobotModel>();

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
  rotor_interfere_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("rotor_interfere_wrench", 1);
  interfrence_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("interference_markers", 1);


  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  torque_allocation_matrix_inv_pub_stamp_ = 0;

  rotor_interfere_comp_wrench_ = Eigen::VectorXd::Zero(6); // reset
  est_external_wrench_ = Eigen::VectorXd::Zero(6);
  init_sum_momentum_ = Eigen::VectorXd::Zero(6);
  integrate_term_ = Eigen::VectorXd::Zero(6);
  prev_est_wrench_timestamp_ = 0;
  fz_bias_ = 0;
  tx_bias_ = 0;
  ty_bias_ = 0;

  force_lock_roll_sub_ = nh_.subscribe("force_lock_roll", 1, &DragonFullVectoringController::forceLockRollCallback, this);

  wrench_estimate_thread_ = boost::thread([this]()
                                          {
                                            ros::NodeHandle control_nh(nh_, "controller");
                                            double update_rate;
                                            control_nh.param ("wrench_estimate_update_rate", update_rate, 100.0);

                                            ros::Rate loop_rate(update_rate);
                                            while(ros::ok())
                                              {
                                                externalWrenchEstimate();
                                                loop_rate.sleep();
                                              }
                                          });
}

void DragonFullVectoringController::rotorInterfereEstimate()
{
  //rotor interference compensation based on previous robot model
  overlap_positions_.clear();
  overlap_weights_.clear();
  overlap_segments_.clear();
  overlap_rotors_.clear();

  if(navigator_->getForceLandingFlag())
    {
      rotor_interfere_comp_wrench_ = Eigen::VectorXd::Zero(6);
      return;
    }

  const auto& seg_tf_map = robot_model_for_control_->getSegmentsTf();

  if(seg_tf_map.size() == 0) return;

  const auto u = robot_model_for_control_->getRotorsNormalFromCog<Eigen::Vector3d>();
  double link_length = (seg_tf_map.at(std::string("inter_joint1")).p - seg_tf_map.at(std::string("link1")).p).Norm();
  KDL::Frame cog_inv = robot_model_for_control_->getCog<KDL::Frame>().Inverse();

  auto rotorInterfere = [this, &seg_tf_map, &cog_inv](int i, Eigen::Vector3d p_rotor, Eigen::Vector3d u_rotor, double link_length, std::string rotor_name)
    {
      for(int j = 0; j < motor_num_; ++j)
        {
          bool overlap_inter = false;
          std::string s = std::to_string(j + 1);

          KDL::Frame f_link  = cog_inv * seg_tf_map.at(std::string("link") + s);
          Eigen::Vector3d u_link = aerial_robot_model::kdlToEigen(f_link.M * KDL::Vector(1, 0, 0));
          Eigen::Vector3d p_link = aerial_robot_model::kdlToEigen(f_link.p);
          Eigen::Vector3d p_inter;
          double dist_inter = overlap_dist_inter_joint_thresh_;
          if(j == motor_num_ - 1)
            p_inter = p_link + u_link * link_length;
          else
            p_inter = aerial_robot_model::kdlToEigen(((cog_inv * seg_tf_map.at(std::string("inter_joint") + s)).p + (cog_inv * seg_tf_map.at(std::string("link") + std::to_string(j + 2))).p)/2);

          if(p_inter.z() > p_rotor.z() && p_link.z() > p_rotor.z()) continue; // never overlap

          // case1: inter joint
          if(p_inter.z() <= p_rotor.z())
            {
              dist_inter = (p_rotor + (p_inter - p_rotor).dot(u_rotor) * u_rotor - p_inter).norm();
              if(dist_inter < overlap_dist_inter_joint_thresh_)
                {
                  overlap_inter = true;
                  /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() <<  ", interfere with inter_joint" << j+1 << ": " << p_inter.transpose());
                }
            }

          if(j == i) // self overlap never
            {
              if(overlap_inter)
                {
                  overlap_positions_.push_back(p_inter);
                  overlap_weights_.push_back(1);
                  overlap_segments_.push_back(std::string("inter_joint") + s);
                  overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);
                }
              continue;
            }

          // case2: rotor
          Eigen::Vector3d p_rotor_l = aerial_robot_model::kdlToEigen((cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_left"))).p);
          Eigen::Vector3d p_rotor_r = aerial_robot_model::kdlToEigen((cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_right"))).p);
          bool overlap_rotor = false;
          double linear_rotor_weight = 0;

          if(p_rotor.z() > p_rotor_l.z() && p_rotor.z() > p_rotor_r.z())
            {
              Eigen::Vector3d p_projected_rotor_l = p_rotor + (p_rotor_l.z() - p_rotor.z()) / u_rotor.z() * u_rotor;
              Eigen::Vector3d p_projected_rotor_r = p_rotor + (p_rotor_r.z() - p_rotor.z()) / u_rotor.z() * u_rotor;
              double dist_rotor_l = (p_projected_rotor_l - p_rotor_l).norm();
              double dist_rotor_r = (p_projected_rotor_r - p_rotor_r).norm();

              if(dist_rotor_l < overlap_dist_rotor_relax_thresh_ || dist_rotor_r < overlap_dist_rotor_relax_thresh_)
                {
                  // ROS_INFO_STREAM("p_projected_rotor_l: " << p_projected_rotor_l.transpose() << "; p_projected_rotor_r: " << p_projected_rotor_r.transpose());
                  // ROS_INFO_STREAM("p_rotor_l: " << p_rotor_l.transpose() << "; p_rotor_r: " << p_rotor_r.transpose());
                  // ROS_INFO_STREAM("dist_rotor_l: " << dist_rotor_l << "; dist_rotor_r: " << dist_rotor_r);
                  double rotor_l_weight = 1;
                  double rotor_r_weight = 1;


                  double relax_range = overlap_dist_rotor_relax_thresh_ - overlap_dist_rotor_thresh_;
                  if(dist_rotor_l > overlap_dist_rotor_thresh_)
                    {
                      if(dist_rotor_l > overlap_dist_rotor_relax_thresh_) rotor_l_weight = 0;
                      else
                        {
                          if(overlap_dist_rotor_relax_thresh_ > overlap_dist_rotor_thresh_)
                            {
                              double diff = (overlap_dist_rotor_relax_thresh_ - dist_rotor_l) / relax_range;
                              rotor_l_weight = diff * diff ;
                            }
                        }
                    }
                  if(dist_rotor_r > overlap_dist_rotor_thresh_)
                    {
                      if(dist_rotor_r > overlap_dist_rotor_relax_thresh_) rotor_r_weight = 0;
                      else
                        {
                          if(overlap_dist_rotor_relax_thresh_ > overlap_dist_rotor_thresh_)
                            {
                              double diff = (overlap_dist_rotor_relax_thresh_ - dist_rotor_r) / relax_range;
                              rotor_r_weight = diff * diff ;
                            }
                        }
                    }

                  if(dist_rotor_l > overlap_dist_rotor_relax_thresh_)
                    {
                      linear_rotor_weight = (overlap_dist_rotor_relax_thresh_ - dist_rotor_r) / overlap_dist_rotor_relax_thresh_;
                      overlap_positions_.push_back(p_rotor_r);
                      overlap_weights_.push_back(rotor_r_weight);
                      overlap_segments_.push_back(std::string("rotor") + s + std::string("_right"));
                      overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);


                      /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with rotor" << j+1 << "right: " << overlap_positions_.back().transpose());
                    }
                  else if(dist_rotor_r > overlap_dist_rotor_relax_thresh_)
                    {
                      linear_rotor_weight = (overlap_dist_rotor_relax_thresh_ - dist_rotor_l) / overlap_dist_rotor_relax_thresh_;
                      overlap_positions_.push_back(p_rotor_l);
                      overlap_weights_.push_back(rotor_l_weight);

                      overlap_segments_.push_back(std::string("rotor") + s + std::string("_left"));
                      overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                      /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with rotor" << j+1 << "left: " << overlap_positions_.back().transpose());
                    }
                  else
                    {
                      linear_rotor_weight = (overlap_dist_rotor_relax_thresh_ - (dist_rotor_l + dist_rotor_r) / 2) / overlap_dist_rotor_relax_thresh_;
                      overlap_positions_.push_back((rotor_l_weight * p_rotor_l + rotor_r_weight * p_rotor_r) / (rotor_l_weight + rotor_r_weight));
                      overlap_weights_.push_back((rotor_l_weight + rotor_r_weight) / 2);

                      overlap_segments_.push_back(std::string("rotor") + s + std::string("_left&right"));
                      overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                      /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with rotor" << j+1 << "left&right: " << overlap_positions_.back().transpose());
                    }

                  overlap_rotor = true;
                }
            }

          // case3: link
          Eigen::Matrix2d A; A << u_link.dot(u_link), -u_link.dot(u_rotor), u_link.dot(u_rotor), - u_rotor.dot(u_rotor);
          Eigen::Vector2d diff(-u_link.dot(p_link - p_rotor), -u_rotor.dot(p_link - p_rotor));
          Eigen::Vector2d t = A.inverse() * diff;
          double dist_link = 1e6;
          bool overlap_link = false;

          Eigen::Vector3d p_link_overlap = p_link + u_link * t(0);
          dist_link = (p_link_overlap - (p_rotor + u_rotor * t(1))).norm();

          if(t(1) < 0)
            {

              if(dist_link < overlap_dist_link_relax_thresh_)
                {
                  overlap_link = true;

                  if(t(0) < 0)
                    {
                      if(j == 0 && t(0) > overlap_dist_link_relax_thresh_) overlap_link = true; // relax for the end of the link
                      else overlap_link = false;
                    }
                  if(t(0) > link_length)
                    {
                      if(j == motor_num_ - 1 && t(0) < link_length + overlap_dist_link_relax_thresh_) overlap_link = true; // relax for the end of the link
                      else overlap_link = false;
                    }
                }

              if(overlap_link)
                {
                  double linear_link_weight = (overlap_dist_link_relax_thresh_ - dist_link) / overlap_dist_link_relax_thresh_;

                  double weight = 1;
                  if(dist_link > overlap_dist_link_thresh_)
                    {
                      double diff = (overlap_dist_link_relax_thresh_ - dist_link) / (overlap_dist_link_relax_thresh_ - overlap_dist_link_thresh_);
                      weight = diff * diff ;
                    }

                  if(overlap_rotor)
                    {
                      Eigen::Vector3d p_rotor_overlap = overlap_positions_.back();
                      double weight_rotor_overlap = overlap_weights_.back();
                      overlap_positions_.back() = (linear_rotor_weight * p_rotor_overlap + linear_link_weight * p_link_overlap) / (linear_rotor_weight + linear_link_weight);
                      overlap_weights_.back() = (weight_rotor_overlap + weight) / 2;

                      overlap_segments_.back() = overlap_segments_.back() + std::string("&link");

                      /// ROS_INFO_STREAM("  rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with link" << j+1 << "&rotor : " << overlap_positions_.back().transpose());
                    }
                  else
                    {

                      if(overlap_inter)
                        {
                          double linear_inter_weight = (overlap_dist_inter_joint_thresh_ - dist_inter) / overlap_dist_inter_joint_thresh_;
                          overlap_positions_.push_back((linear_inter_weight * p_inter + linear_link_weight * p_link_overlap) / (linear_inter_weight + linear_link_weight));
                          overlap_weights_.push_back(weight);

                          overlap_segments_.push_back(std::string("link&inter_joint") + s);
                          overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                          /// ROS_INFO_STREAM(" rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with link & inter_joint" << j+1 << ": " << overlap_positions_.back().transpose());
                        }
                      else
                        {
                          overlap_positions_.push_back(p_link_overlap);
                          overlap_weights_.push_back(weight);

                          overlap_segments_.push_back(std::string("link") + s);
                          overlap_rotors_.push_back(std::string("rotor") + std::to_string(i + 1) + std::string("_") + rotor_name);

                          /// ROS_INFO_STREAM("rotor" << i + 1 << "_" << rotor_name << " " << p_rotor.transpose() << ", interfere with link" << j+1 << ": " << overlap_positions_.back().transpose());
                        }
                    }
                }

            }
        }
    };

  for(int i = 0; i < motor_num_; ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f_rotor  = cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_left"));
      rotorInterfere(i, aerial_robot_model::kdlToEigen(f_rotor.p), u.at(i), link_length, std::string("left"));
      f_rotor  = cog_inv * seg_tf_map.at(std::string("edf") + s + std::string("_right"));
      rotorInterfere(i, aerial_robot_model::kdlToEigen(f_rotor.p), u.at(i), link_length, std::string("right"));
    }


  if(overlap_positions_.size() == 0)
    {
      fz_bias_ = (1 - wrench_lpf_rate_) * fz_bias_ + wrench_lpf_rate_ * est_external_wrench_(2);
      tx_bias_ = (1 - wrench_lpf_rate_) * tx_bias_ + wrench_lpf_rate_ * est_external_wrench_(3);
      ty_bias_ = (1 - wrench_lpf_rate_) * ty_bias_ + wrench_lpf_rate_ * est_external_wrench_(4);

      rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * Eigen::VectorXd::Zero(3);
    }
  else
    {
      Eigen::Vector3d external_wrench(est_external_wrench_(2), est_external_wrench_(3) - tx_bias_, est_external_wrench_(4) - ty_bias_); //fz, mx,my
      if(fz_bias_thresh_ < fabs(fz_bias_)) external_wrench(0) -= fz_bias_;

      /// ROS_WARN_STREAM("compensate rotor overlap interfere: fz_bias: " << fz_bias_ << " external wrench: " << external_wrench.transpose());

      if(external_wrench(0) >= 0)
        {
          rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * Eigen::VectorXd::Zero(3);
          rotor_interfere_force_.setZero(0);
          return;
        }

      // for(int i = 0; i < overlap_rotors_.size(); i++) std::cout << overlap_rotors_.at(i) << " -> " << overlap_segments_.at(i) << "; ";
      // std::cout << std::endl;
      // for(int i = 0; i < overlap_positions_.size(); i++) std::cout << overlap_positions_.at(i).transpose() << "; ";
      // std::cout << std::endl;

      Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, overlap_positions_.size());
      for(int j = 0; j < overlap_positions_.size(); j++)
        A.col(j) = Eigen::Vector3d(1, overlap_positions_.at(j).y(), -overlap_positions_.at(j).x());

      Eigen::MatrixXd dev_weight_mat = Eigen::MatrixXd::Identity(A.cols(), A.cols()) - Eigen::MatrixXd::Ones(A.cols(), A.cols())/A.cols();
      Eigen::MatrixXd W_dev = Eigen::MatrixXd::Identity(A.cols(), A.cols());
      for(int j = 0; j < overlap_weights_.size(); j++)
        W_dev(j,j) = overlap_weights_.at(j);

      Eigen::MatrixXd W_diff = rotor_interfere_torque_xy_weight_ * Eigen::MatrixXd::Identity(A.rows(), A.rows());
      W_diff(0,0) = 1;
      rotor_interfere_force_ = (A.transpose() * W_diff * A  + rotor_interfere_force_dev_weight_ * dev_weight_mat * W_dev * dev_weight_mat).inverse() * A.transpose() * W_diff * external_wrench;

      if(rotor_interfere_force_.maxCoeff() > 0)
        {
          while(1)
            {
              std::vector<Eigen::VectorXd> overlap_positions_tmp = overlap_positions_;
              std::vector<double> overlap_weights_tmp = overlap_weights_;
              std::vector<std::string> overlap_segments_tmp = overlap_segments_;
              std::vector<std::string> overlap_rotors_tmp = overlap_rotors_;
              overlap_positions_.clear();
              overlap_weights_.clear();
              overlap_segments_.clear();
              overlap_rotors_.clear();

              for(int i = 0; i < rotor_interfere_force_.size(); i++)
                {
                  if(rotor_interfere_force_(i) < 0)
                    {
                      overlap_positions_.push_back(overlap_positions_tmp.at(i));
                      overlap_weights_.push_back(overlap_weights_tmp.at(i));
                      overlap_segments_.push_back(overlap_segments_tmp.at(i));
                      overlap_rotors_.push_back(overlap_rotors_tmp.at(i));
                    }
                }
              if(overlap_positions_.size() == 0)
                {
                  rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * Eigen::VectorXd::Zero(3);
                  rotor_interfere_force_.setZero();
                  ROS_DEBUG_STREAM("no rotor_interfere from recalculate");
                  break;
                }
              else
                {
                  /// ROS_WARN_STREAM("rotor_interfere force before recalculate: " << rotor_interfere_force_.transpose());
                  A = Eigen::MatrixXd::Zero(3, overlap_positions_.size());
                  for(int j = 0; j < overlap_positions_.size(); j++)
                    A.col(j) = Eigen::Vector3d(1, overlap_positions_.at(j).y(), -overlap_positions_.at(j).x());

                  Eigen::MatrixXd dev_weight_mat = Eigen::MatrixXd::Identity(A.cols(), A.cols()) - Eigen::MatrixXd::Ones(A.cols(), A.cols())/A.cols();
                  Eigen::MatrixXd W_dev = Eigen::MatrixXd::Identity(A.cols(), A.cols());
                  for(int j = 0; j < overlap_weights_.size(); j++)
                    W_dev(j,j) = overlap_weights_.at(j);

                  Eigen::MatrixXd W_diff = rotor_interfere_torque_xy_weight_ * Eigen::MatrixXd::Identity(A.rows(), A.rows());
                  W_diff(0,0) = 1;
                  rotor_interfere_force_ = (A.transpose() * W_diff * A  + rotor_interfere_force_dev_weight_ * dev_weight_mat * W_dev * dev_weight_mat).inverse() * A.transpose() * W_diff * external_wrench;


                  if(rotor_interfere_force_.maxCoeff() > 0)
                    {
                      ROS_DEBUG_STREAM("invalid rotor_interfere force: " << rotor_interfere_force_.transpose()); // loop
                    }
                  else
                    {
                      ROS_DEBUG_STREAM("rotor_interfere force recalculate: " << rotor_interfere_force_.transpose());
                      break;
                    }
                }
            }
        }
      else
        ROS_DEBUG_STREAM("rotor_interfere force: " << rotor_interfere_force_.transpose());

      //rotor_interfere_comp_wrench_.segment(2, 3) = (1 - comp_wrench_lpf_rate_) * rotor_interfere_comp_wrench_.segment(2, 3) +  comp_wrench_lpf_rate_ * (- A * rotor_interfere_force_);
      rotor_interfere_comp_wrench_.segment(2, 3) =  - A * rotor_interfere_force_;

      ROS_DEBUG_STREAM("rotor_interfere_wrench: " << -rotor_interfere_comp_wrench_.segment(2, 3).transpose());
    }

  // visualize the interference
  visualization_msgs::MarkerArray interference_marker_msg;
  if(overlap_positions_.size() > 0)
    {
      int id = 0;
      for(int i = 0; i < overlap_positions_.size(); i++)
        {
          visualization_msgs::Marker segment_sphere;
          segment_sphere.header.stamp = ros::Time::now();
          segment_sphere.header.frame_id = nh_.getNamespace() + std::string("/cog"); //overlap_segments_.at(i);
          segment_sphere.id = id++;
          segment_sphere.action = visualization_msgs::Marker::ADD;
          segment_sphere.type = visualization_msgs::Marker::SPHERE;
          segment_sphere.pose.position.x = overlap_positions_.at(i).x();
          segment_sphere.pose.position.y = overlap_positions_.at(i).y();
          segment_sphere.pose.position.z = overlap_positions_.at(i).z();
          segment_sphere.pose.orientation.w = 1;
          segment_sphere.scale.x = 0.15;
          segment_sphere.scale.y = 0.15;
          segment_sphere.scale.z = 0.15;
          segment_sphere.color.g = 1.0;
          segment_sphere.color.a = 0.5;

          interference_marker_msg.markers.push_back(segment_sphere);

          visualization_msgs::Marker force_arrow;
          force_arrow.header.stamp = ros::Time::now();
          force_arrow.header.frame_id = nh_.getNamespace() + std::string("/cog"); //overlap_segments_.at(i);
          force_arrow.id = id++;
          force_arrow.action = visualization_msgs::Marker::ADD;
          force_arrow.type = visualization_msgs::Marker::ARROW;
          force_arrow.pose.position.x = overlap_positions_.at(i).x();
          force_arrow.pose.position.y = overlap_positions_.at(i).y();
          force_arrow.pose.position.z = overlap_positions_.at(i).z() - 0.02;
          force_arrow.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, -M_PI/2, 0);
          force_arrow.scale.x = rotor_interfere_force_(i) / 10.0;
          force_arrow.scale.y = 0.02;
          force_arrow.scale.z = 0.02;
          force_arrow.color.g = 1.0;
          force_arrow.color.a = 0.5;

          interference_marker_msg.markers.push_back(force_arrow);
        }

      // remove the old marker
      visualization_msgs::Marker delete_operation;
      delete_operation.id = id++;
      delete_operation.action = visualization_msgs::Marker::DELETE;
      interference_marker_msg.markers.push_back(delete_operation);
      delete_operation.id = id++;
      interference_marker_msg.markers.push_back(delete_operation);
    }
  else
    {
      visualization_msgs::Marker delete_operation;
      delete_operation.action = visualization_msgs::Marker::DELETEALL;
      interference_marker_msg.markers.push_back(delete_operation);
    }

  interfrence_marker_pub_.publish(interference_marker_msg);

  if(overlap_rotors_.size() > 0)
    {
      std::stringstream ss;
      for(int i = 0; i < overlap_rotors_.size(); i++)
        ss << overlap_rotors_.at(i) << " -> " << overlap_segments_.at(i) << "; ";
      ROS_DEBUG_STREAM("rotor interference: " << ss.str());
    }
}

void DragonFullVectoringController::rotorInterfereCompensate(Eigen::VectorXd& target_wrench_acc)
{
  double mass_inv =  1 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();

  Eigen::VectorXd rotor_interfere_comp_acc = Eigen::VectorXd::Zero(6);
  rotor_interfere_comp_acc(2) = mass_inv * rotor_interfere_comp_wrench_(2);

  bool torque_comp = false;
  if(overlap_positions_.size() == 1)
    {
      ROS_INFO_STREAM("compsensate the torque resulted from rotor interference: " << overlap_rotors_.at(0) << " to " << overlap_segments_.at(0));
      torque_comp = true;
    }

  if(overlap_positions_.size() == 2)
    {
      if(overlap_rotors_.at(0).substr(0, 6) == overlap_rotors_.at(1).substr(0, 6))
        {
          ROS_INFO_STREAM("do rotor interference torque compensation: " << overlap_rotors_.at(0) << " and " << overlap_rotors_.at(1));
          torque_comp = true;
        }
    }

  if(torque_comp)
    rotor_interfere_comp_acc.tail(3) = inertia_inv * rotor_interfere_comp_wrench_.tail(3);

  target_wrench_acc += rotor_interfere_comp_acc;
}

void DragonFullVectoringController::rotorInterfereAvoid(const DragonModelPtr robot_model, RotorInterfereMap& interfere_map, std::vector<int>& roll_locked_gimbal, std::vector<double>& gimbal_nominal_angles)
{
  int rotor_num = robot_model->getRotorNum();
  std::string thrust_link = robot_model->getThrustLinkName();
  const auto& seg_tf_map = robot_model->getSegmentsTf();
  const auto rotors_pos = robot_model->getRotorsOriginFromCog<KDL::Vector>();

  double angle_max_thresh = 1.2; // TODO: parameter
  double angle_min_thresh = 0.4; // TODO: parameter
  double collision_padding_rate = 1.2; // TODO: parameter
  double dist_thresh = robot_model->getEdfRadius() * 4 + 0.05; // > 2 x diameter

  interfere_map = RotorInterfereMap{};

  std::stringstream ss_map;
  ss_map << "\n";

  for(int i = 0; i < rotor_num; i++)
    {
      auto pos_i = rotors_pos.at(i); // position w.r.t. CoG

      std::string rotor_i = thrust_link + std::to_string(i + 1);
      auto pose_i = seg_tf_map.at(rotor_i); // pose w.r.t. Baselink (Root)

      std::vector<std::pair<int, Eigen::Vector3d>> bound_list;

      for (int j = 0; j < rotor_num; j++)
        {
          if (i == j) continue;


          std::string rotor_j = thrust_link + std::to_string(j + 1);
          auto pose_j = seg_tf_map.at(rotor_j); // pose w.r.t. Baselink (Root)

          // relativel position from the i-th rotor
          auto rel_pose = pose_i.Inverse() * pose_j;
          auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

          // skip if the j-th rotor is not align in the same plane;
          if (fabs(rel_pos.y()) > dist_thresh) continue;

          double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle
          // skip if the j-th rotor is far from i-th
          if (fabs(angle) > angle_max_thresh) continue;

          // calculate the bound angle

          double r = robot_model->getEdfRadius() * collision_padding_rate;

          double x1 = 0;
          double y1 = 0;
          double x2 = rel_pos.x();
          double y2 = rel_pos.z();

          double b = y1 - y2;

          double a1 = x2 + r - x1;
          double l1 = sqrt(pow(a1, 2) + pow(b, 2));
          double phi1 = atan2(r, l1);
          double theta1 = atan2(a1, b) + phi1;
          theta1 = -theta1;

          double a2 = x2 - r - x1;
          double l2 = sqrt(pow(a2, 2) + pow(b, 2));
          double phi2 = atan2(r, l2);
          double theta2 = atan2(a2, b) - phi2;
          theta2 = -theta2;

          //ss_map << "(" << i + 1 << ", " << j + 1 << "): " << angle << ", " << rel_pos.transpose() << ", theta1: " << theta1 << ", theta2: " << theta2 << "\n";

          Eigen::Vector3d angle_v;
          if (fabs(theta1) < fabs(theta2)) angle_v = Eigen::Vector3d(angle, theta1, theta2);
          else angle_v = Eigen::Vector3d(angle, theta2, theta1);

          bound_list.push_back(std::make_pair(j, angle_v));
        }

      if (bound_list.size() == 0) continue;

      // sort
      std::sort(bound_list.begin(), bound_list.end(),  \
                [](auto const& lhs, auto const& rhs) { \
                  return fabs(lhs.second(0)) < fabs(rhs.second(0));
                });
      // only consider when the prime interference is critical
      double prime_tilt_angle = bound_list.at(0).second(0);
      if (fabs(prime_tilt_angle) > angle_min_thresh) continue;

      // TODO: consider the sub-prime interference!

      interfere_map.insert(std::make_pair(i, bound_list));

      roll_locked_gimbal.at(i) = 1;
      gimbal_nominal_angles.at(2 * i) = 0;
    }

  if (interfere_map.size() > 0)
    {
      for (const auto& it: interfere_map)
        {
          ss_map << "rotor" << it.first+1 << ": \n";
          for(const auto& bound: it.second)
            {
              ss_map << "\t rotor" << bound.first + 1 << ": " << bound.second.transpose() << " \n";
            }
        }

      ROS_INFO_STREAM_THROTTLE(1.0, "\033[32m" << ss_map.str() << "\033[0m");
    }
}


void DragonFullVectoringController::controlCore()
{
  /* TODO: saturation of z control */
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();


  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv =  1 / robot_model_->getMass();

  // rotor interference
  if(rotor_interfere_estimate_) rotorInterfereEstimate();
  if(rotor_interfere_compensate_) rotorInterfereCompensate(target_wrench_acc_cog);

  setTargetWrenchAccCog(target_wrench_acc_cog);

  // iteratively find the target force and target gimbal angles
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation); // update the cog orientation
  KDL::JntArray gimbal_processed_joint = dragon_robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  auto roll_locked_gimbal = dragon_robot_model_->getRollLockedGimbal();
  const auto links_rotation_from_cog = dragon_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  auto gimbal_nominal_angles = dragon_robot_model_->getGimbalNominalAngles();
  const auto& joint_index_map = dragon_robot_model_->getJointIndexMap();


  // workround: rotor interfere avoid
  RotorInterfereMap interfere_map;
  rotorInterfereAvoid(dragon_robot_model_, interfere_map, roll_locked_gimbal, gimbal_nominal_angles);

  /* WIP: force lock all gimbal roll angles to zero */
  if (force_lock_all_angle_)
    {
      for (int i = 0; i < motor_num_; i ++)
        {
          if (i % 2 == 1) continue;

          roll_locked_gimbal.at(i) = 1;
          gimbal_nominal_angles.at(2 * i) = 0;
        }
    }


  int gimbal_lock_num = std::accumulate(roll_locked_gimbal.begin(), roll_locked_gimbal.end(), 0);
  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_ - gimbal_lock_num);

  Eigen::MatrixXd A1;
  Eigen::VectorXd b1;
  Eigen::MatrixXd Psi;
  Dragon::FullVectoringRobotModel::updateJointTorqueMatrices(robot_model_for_control_,                 \
                                                             gimbal_processed_joint, links_rotation_from_cog, \
                                                             roll_locked_gimbal, gimbal_nominal_angles, \
                                                             thrust_force_weight_, joint_torque_weight_, \
                                                             A1, b1, Psi);

  double t = ros::Time::now().toSec();
  for(int j = 0; j < allocation_refine_max_iteration_; j++)
    {
      /* 5.2.1. update the wrench allocation matrix  */
      std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd mask(3,2);
      mask << 1, 0, 0, 0, 0, 1;
      int last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));

          if(roll_locked_gimbal.at(i) == 0)
            {
              /* 3DoF */
              full_q_mat.middleCols(last_col, 3) = wrench_map * links_rotation_from_cog.at(i);
              last_col += 3;
            }
          else
            {
              /* gimbal lock: 2Dof */

              Eigen::MatrixXd pitch_rot = Eigen::MatrixXd::Identity(3, 3);

              if (force_lock_all_angle_)
                {
                  double pitch_angle = gimbal_nominal_angles.at(i * 2 + 1) + force_tilt_limit_angle_;
                  pitch_rot = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(0, pitch_angle, 0));
                }

              auto map_it = interfere_map.find(i);
              if (map_it != interfere_map.end())
                {
                  double pitch_angle = gimbal_nominal_angles.at(i * 2 + 1) + map_it->second.at(0).second(1);
                  pitch_rot = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(0, pitch_angle, 0));
                }

              full_q_mat.middleCols(last_col, 2) = wrench_map * links_rotation_from_cog.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles.at(i * 2), 0, 0)) * pitch_rot * mask;
              last_col += 2;
            }
        }

      inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse(); // update
      full_q_mat.topRows(3) =  mass_inv * full_q_mat.topRows(3);
      full_q_mat.bottomRows(3) =  inertia_inv * full_q_mat.bottomRows(3);

      // allocation
      // 1. solve by psuedo-inverse
      double s_t1 = ros::WallTime::now().toSec();
      Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
      target_vectoring_f_ = full_q_mat_inv * target_wrench_acc_cog;


      // compesante joint torque, only if the robot is flying or almost flying.
      if (start_rp_integration_)
        {
          tf::Vector3 acc_bias_w(0, 0, pid_controllers_.at(Z).result()); // TODO: maybe only I term is better
          //tf::Vector3 acc_bias_w(0, 0, pid_controllers_.at(Z).getPTerm() + pid_controllers_.at(Z).getITerm());
          tf::Vector3 acc_bias_cog = uav_rot.inverse() * acc_bias_w;
          Eigen::VectorXd wrench_bias = Eigen::VectorXd::Zero(6);
          wrench_bias.head(3) = Eigen::Vector3d(acc_bias_cog.x(), acc_bias_cog.y(), acc_bias_cog.z());

          Dragon::FullVectoringRobotModel::compensateJointTorque(A1, Psi, b1, -wrench_bias, \
                                                                 full_q_mat_inv, full_q_mat, joint_torque_weight_, \
                                                                 target_vectoring_f_);
        }

      double t_diff_inv = ros::WallTime::now().toSec() - s_t1;
      static double ave_t_diff_inv = t_diff_inv;
      ave_t_diff_inv = 0.8 * ave_t_diff_inv + 0.2 * t_diff_inv;

      // 2. solve by QP
      OsqpEigen::Solver qp_solver;
      qp_solver.settings()->setVerbosity(false);
      qp_solver.settings()->setWarmStart(true);
      int f_ndof = 3 * motor_num_ - gimbal_lock_num;
      qp_solver.data()->setNumberOfVariables(f_ndof);
      qp_solver.data()->setNumberOfConstraints(6 + f_ndof);


      // 2.1 cost function
      Eigen::MatrixXd hessian = Eigen::MatrixXd::Identity(f_ndof, f_ndof);
      Eigen::SparseMatrix<double> hessian_sparse = hessian.sparseView();
      Eigen::VectorXd gradient = Eigen::VectorXd::Zero(f_ndof);
      qp_solver.data()->setHessianMatrix(hessian_sparse);
      qp_solver.data()->setGradient(gradient);

      // 2.2 constraint (except of range)
      Eigen::MatrixXd constraints = Eigen::MatrixXd::Zero(6 + f_ndof, f_ndof);
      constraints.topRows(6) = full_q_mat;
      constraints.bottomRows(f_ndof) = Eigen::MatrixXd::Identity(f_ndof, f_ndof);
      Eigen::SparseMatrix<double> constraint_sparse = constraints.sparseView();
      qp_solver.data()->setLinearConstraintsMatrix(constraint_sparse);
      double thrust_range = 100;
      Eigen::VectorXd lower_bound = Eigen::VectorXd::Ones(6 + f_ndof) * -thrust_range;
      Eigen::VectorXd upper_bound = Eigen::VectorXd::Ones(6 + f_ndof) * thrust_range;

      if (force_lock_all_angle_)
        {
          // WIP: the inside gimbal pitch should not have larger angle than the tilt of link
          // x is along the link direction, and z is orthonogal to that
          int col = 0;
          for (int i = 0; i < motor_num_; i ++)
            {
              if (i % 2 == 0)
                {
                  if (force_tilt_limit_angle_ < 0) upper_bound(6 + col) = 0;
                  if (force_tilt_limit_angle_ > 0) lower_bound(6 + col) = 0;
                }

              if (roll_locked_gimbal.at(i)) col +=2;
              else col +=3;
            }
        }

      if (interfere_map.size() > 0)
        {
          int col = 0;
          for (int i = 0; i < motor_num_; i ++)
            {
              auto map_it = interfere_map.find(i);
              if (map_it != interfere_map.end())
                {
                  double tilt_angle = map_it->second.at(0).second(1);
                  double sub_tilt_angle = map_it->second.at(0).second(2);

                  if (tilt_angle < 0 && sub_tilt_angle > 0) upper_bound(6 + col) = 0;
                  if (tilt_angle > 0 && sub_tilt_angle < 0) lower_bound(6 + col) = 0;
                  if (tilt_angle > 0 && sub_tilt_angle > 0) upper_bound(6 + col) = 0;
                  if (tilt_angle < 0 && sub_tilt_angle < 0) lower_bound(6 + col) = 0;
                }

              if (roll_locked_gimbal.at(i)) col +=2;
              else col +=3;
            }
        }


      lower_bound.head(6) = target_wrench_acc_cog;
      upper_bound.head(6) = target_wrench_acc_cog;

      qp_solver.data()->setLowerBound(lower_bound);
      qp_solver.data()->setUpperBound(upper_bound);

      if(!qp_solver.initSolver()) {
        ROS_ERROR_STREAM("can not initialize qp solver");
        return;
      }

      // TODO1: set the initial value
      // TODO2: the error of not resolvable
      double s_t = ros::WallTime::now().toSec();
      bool res = qp_solver.solve(); // with large range: x 1.5


      if(!res) {
        ROS_ERROR_STREAM("can not solve QP");
      } else {
        Eigen::VectorXd target_vectoring_f_qp = qp_solver.getSolution();

        double t_diff_qp = ros::WallTime::now().toSec() - s_t;
        static double ave_t_diff_qp = t_diff_qp;
        ave_t_diff_qp = 0.8 * ave_t_diff_qp + 0.2 * t_diff_qp;

        ROS_INFO_STREAM_THROTTLE(1.0, "\n target_vectoring_f_: " << target_vectoring_f_.transpose() << "\n "
                                 "target_vectoring_f_qp: " << target_vectoring_f_qp.transpose());
        ROS_INFO_THROTTLE(1.0, "Inv solve time: %f, QP solve time: %f", ave_t_diff_inv, ave_t_diff_qp);
        target_vectoring_f_ = target_vectoring_f_qp;
      }


      if(control_verbose_) ROS_DEBUG_STREAM("vectoring force for control in iteration "<< j+1 << ": " << target_vectoring_f_.transpose());
      last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          double prev_roll_angle = target_gimbal_angles_.at(2 * i);
          double prev_pitch_angle = target_gimbal_angles_.at(2 * i + 1);

          if(roll_locked_gimbal.at(i) == 0)
            {
              Eigen::Vector3d f = target_vectoring_f_.segment(last_col, 3);
              target_base_thrust_.at(i) = f.norm();

              double roll_angle = atan2(-f.y(), f.z());
              double pitch_angle = atan2(f.x(), -f.y() * sin(roll_angle) + f.z() * cos(roll_angle));

              target_gimbal_angles_.at(2 * i) = roll_angle;
              target_gimbal_angles_.at(2 * i + 1) = pitch_angle;

              last_col += 3;
            }
          else
            {
              Eigen::VectorXd f_2d = target_vectoring_f_.segment(last_col, 2);
              target_base_thrust_.at(i) = f_2d.norm();


              Eigen::MatrixXd pitch_rot = Eigen::MatrixXd::Identity(3, 3);
              Eigen::MatrixXd mask(3,2);
              mask << 1, 0, 0, 0, 0, 1;
              if (force_lock_all_angle_)
                {
                  double pitch_angle = gimbal_nominal_angles.at(i * 2 + 1) + force_tilt_limit_angle_;
                  pitch_rot = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(0, pitch_angle, 0));
                }
              auto map_it = interfere_map.find(i);
              if (map_it != interfere_map.end())
                {
                  double pitch_angle = gimbal_nominal_angles.at(i * 2 + 1) + map_it->second.at(0).second(1);
                  pitch_rot = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(0, pitch_angle, 0));
                }

              Eigen::VectorXd f = pitch_rot * mask * f_2d;

              target_gimbal_angles_.at(2 * i) = gimbal_nominal_angles.at(2 * i); // lock the gimbal roll
              target_gimbal_angles_.at(2 * i + 1) = atan2(f.x(), f.z());

              last_col += 2;
            }

          Dragon::FullVectoringRobotModel::getShortestPath(target_gimbal_angles_.at(2 * i), prev_roll_angle, \
                                                           target_gimbal_angles_.at(2 * i + 1), prev_pitch_angle);

        }

      /* before leave ground in takeoff phase, no active gimbal control, so use nominal values */
      if(!start_rp_integration_ && target_wrench_acc_cog(Z) < takeoff_acc_z_thresh_)
        {
          target_gimbal_angles_ = gimbal_nominal_angles;
        }

      std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog = rotors_origin_from_cog;
      for(int i = 0; i < motor_num_; ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = target_gimbal_angles_.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = target_gimbal_angles_.at(i * 2 + 1);
        }
      robot_model_for_control_->updateRobotModel(gimbal_processed_joint);
      rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      double max_diff = 1e-6;
      for(int i = 0; i < motor_num_; i++)
        {
          double diff = (rotors_origin_from_cog.at(i) - prev_rotors_origin_from_cog.at(i)).norm();
          if(diff > max_diff) max_diff = diff;
        }

      if(control_verbose_) ROS_DEBUG_STREAM("refine rotor origin in control: iteration "<< j+1 << ", max_diff: " << max_diff);

      if(max_diff < allocation_refine_threshold_)
        {
          if(control_verbose_) ROS_INFO_STREAM("refine rotor origin in control: converge in iteration " << j+1 << " max_diff " << max_diff << ", use " << ros::Time::now().toSec() - t << "sec");
          break;
        }

      if(j == allocation_refine_max_iteration_ - 1)
        {
          ROS_WARN_STREAM("refine rotor origin in control: can not converge in iteration " << j+1 << " max_diff " << max_diff);
        }
    }
}

void DragonFullVectoringController::externalWrenchEstimate()
{
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
     navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
    {
      prev_est_wrench_timestamp_ = 0;
      integrate_term_ = Eigen::VectorXd::Zero(6);
      return;
    }

  Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
  auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::DragonImu>(estimator_->getImuHandler(0));
  tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
  tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
  Eigen::Matrix3d cog_rot;
  tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  double mass = robot_model_->getMass();

  Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
  sum_momentum.head(3) = mass * vel_w;
  sum_momentum.tail(3) = inertia * omega_cog;

  Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
  J_t.topLeftCorner(3,3) = cog_rot;

  Eigen::VectorXd N = mass * robot_model_->getGravity();
  N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

  const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();
  Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
  target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

  if(prev_est_wrench_timestamp_ == 0)
    {
      prev_est_wrench_timestamp_ = ros::Time::now().toSec();
      init_sum_momentum_ = sum_momentum; // not good
    }

  double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;

  integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;

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

void DragonFullVectoringController::sendCmd()
{
  PoseLinearController::sendCmd();

  /* send base throttle command */
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  /* send gimbal control command */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  if (gimbal_vectoring_check_flag_)
    {
      gimbal_control_msg.position = dragon_robot_model_->getGimbalNominalAngles();
    }
  else
    {
      for(int i = 0; i < motor_num_ * 2; i++)
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    }
  gimbal_control_pub_.publish(gimbal_control_msg);


  sendTorqueAllocationMatrixInv();

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++)
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  /* rotor interfere wrench */
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  wrench_msg.wrench.force.z = -rotor_interfere_comp_wrench_(2);
  wrench_msg.wrench.torque.x = -rotor_interfere_comp_wrench_(3);
  wrench_msg.wrench.torque.y = -rotor_interfere_comp_wrench_(4);
  rotor_interfere_wrench_pub_.publish(wrench_msg);


  sensor_msgs::Joy force_msg; // can only publish 3 elements
  force_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
  for(int i = 0; i < rotor_interfere_force_.size(); i++)
    force_msg.axes.push_back(rotor_interfere_force_(i));
}

void DragonFullVectoringController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();


      //wrench allocation matrix
      double mass_inv =  1 / robot_model_for_control_->getMass();
      Eigen::Matrix3d inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse();
      Eigen::MatrixXd q_mat = robot_model_for_control_->calcWrenchMatrixOnCoG();
      q_mat.topRows(3) =  mass_inv * q_mat.topRows(3);
      q_mat.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
      Eigen::MatrixXd q_mat_inv = aerial_robot_model::pseudoinverse(q_mat.middleRows(2,3));

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv.rightCols(2);

      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = 0;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void DragonFullVectoringController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = 0;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

void DragonFullVectoringController::reset() {
  PoseLinearController::reset();

  setAttitudeGains();
}


void DragonFullVectoringController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "decoupling", decoupling_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false);
  getParam<double>(control_nh, "allocation_refine_threshold", allocation_refine_threshold_, 0.01);
  getParam<int>(control_nh, "allocation_refine_max_iteration", allocation_refine_max_iteration_, 1);

  momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6,6);
  double force_weight, torque_weight;
  getParam<double>(control_nh, "momentum_observer_force_weight", force_weight, 10.0);
  getParam<double>(control_nh, "momentum_observer_torque_weight", torque_weight, 10.0);
  momentum_observer_matrix_.topRows(3) *= force_weight;
  momentum_observer_matrix_.bottomRows(3) *= torque_weight;

  getParam<bool>(control_nh, "rotor_interfere_estimate", rotor_interfere_estimate_, true);
  getParam<bool>(control_nh, "rotor_interfere_compensate", rotor_interfere_compensate_, true);
  if(rotor_interfere_compensate_) rotor_interfere_estimate_ = true;

  getParam<double>(control_nh, "external_wrench_lpf_rate", wrench_lpf_rate_, 0.5);
  getParam<double>(control_nh, "external_fz_bias_thresh", fz_bias_thresh_, 1.0);
  getParam<double>(control_nh, "rotor_interfere_comp_wrench_lpf_rate", comp_wrench_lpf_rate_, 0.5);
  getParam<double>(control_nh, "rotor_interfere_force_dev_weight", rotor_interfere_force_dev_weight_, 1.0);
  getParam<double>(control_nh, "rotor_interfere_torque_xy_weight", rotor_interfere_torque_xy_weight_, 1.0);

  getParam<double>(control_nh, "overlap_dist_link_thresh", overlap_dist_link_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_rotor_thresh", overlap_dist_rotor_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_link_relax_thresh", overlap_dist_link_relax_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_rotor_relax_thresh", overlap_dist_rotor_relax_thresh_, 0.08);
  getParam<double>(control_nh, "overlap_dist_inter_joint_thresh", overlap_dist_inter_joint_thresh_, 0.08);

  getParam<double>(control_nh, "takeoff_acc_z_thresh", takeoff_acc_z_thresh_, 8.0);

  getParam<double>(control_nh, "thrust_force_weight", thrust_force_weight_, 1.0);
  getParam<double>(control_nh, "joint_torque_weight", joint_torque_weight_, 1.0);

  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.1);
}

void DragonFullVectoringController::forceLockRollCallback(const std_msgs::Float32& msg)
{
  force_tilt_limit_angle_ = msg.data;

  if (fabs(force_tilt_limit_angle_) <= M_PI)
    force_lock_all_angle_ = true;
  else
    force_lock_all_angle_ = false;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonFullVectoringController, aerial_robot_control::ControlBase);
