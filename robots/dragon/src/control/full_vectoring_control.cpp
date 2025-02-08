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

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  torque_allocation_matrix_inv_pub_stamp_ = 0;
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

  // rotor interfere avoid
  PrimeBoundMap rotor_bound_map;
  rotorInterfereAvoid(dragon_robot_model_, rotor_bound_map, roll_locked_gimbal, gimbal_nominal_angles);

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
  allocation::updateJointTorqueMatrices(robot_model_for_control_,                 \
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

      // 2. solve by QP
      allocation::constraint::vectoring(robot_model_for_control_, full_q_mat,    \
                                        target_wrench_acc_cog,          \
                                        gimbal_nominal_angles, roll_locked_gimbal, \
                                        rotor_bound_map, target_vectoring_f_);


      double t_diff_inv = ros::WallTime::now().toSec() - s_t1;
      static double ave_t_diff_inv = t_diff_inv;
      ave_t_diff_inv = 0.8 * ave_t_diff_inv + 0.2 * t_diff_inv;


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

              Eigen::VectorXd f = pitch_rot * mask * f_2d;

              target_gimbal_angles_.at(2 * i) = gimbal_nominal_angles.at(2 * i); // lock the gimbal roll
              target_gimbal_angles_.at(2 * i + 1) = atan2(f.x(), f.z());

              last_col += 2;
            }

          vectoring::getShortestPath(target_gimbal_angles_.at(2 * i), prev_roll_angle, \
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

void DragonFullVectoringController::rotorInterfereAvoid(const DragonModelPtr robot_model, PrimeBoundMap& prime_bound_map, std::vector<int>& roll_locked_gimbal, std::vector<double>& gimbal_nominal_angles)
{
  int rotor_num = robot_model->getRotorNum();
  std::string thrust_link = robot_model->getThrustLinkName();
  const auto& seg_tf_map = robot_model->getSegmentsTf();
  const auto rotors_pos = robot_model->getRotorsOriginFromCog<KDL::Vector>();

  double angle_max_thresh = 1.2; // TODO: parameter
  double collision_padding_rate = 1.2; // TODO: parameter
  double roll_angle_thresh = 0.4;  // TODO: parameter

  prime_bound_map = PrimeBoundMap{};
  std::map<int, std::vector<std::pair<int, Eigen::Vector3d>>> interfere_raw_map{};
  std::map<int, std::vector<std::pair<double, double>>> bounds_map{};

  auto tangent_calc = [] (auto r1, auto r2, auto rel_pos)
                      {
                        double x1 = 0;
                        double y1 = 0;
                        double x2 = rel_pos.x();
                        double y2 = rel_pos.z();

                        double b = y1 - y2;

                        double a1 = x2 + r2 - x1;
                        double l1 = sqrt(pow(a1, 2) + pow(b, 2));
                        double phi1 = atan2(r1, l1);
                        double theta1 = atan2(a1, b) + phi1;
                        theta1 = -theta1;

                        double a2 = x2 - r2 - x1;
                        double l2 = sqrt(pow(a2, 2) + pow(b, 2));
                        double phi2 = atan2(r1, l2);
                        double theta2 = atan2(a2, b) - phi2;
                        theta2 = -theta2;

                        return std::make_pair(theta1, theta2);
                      };

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
          double roll_angle = atan2(fabs(rel_pos.y()), fabs(rel_pos.z()));
          if (roll_angle > roll_angle_thresh) continue;

          double pitch_angle = atan2(-rel_pos.x(), -rel_pos.z());
          // skip if the j-th rotor is far from i-th
          if (fabs(pitch_angle) > angle_max_thresh) continue;

          // calculate the bound angle
          double r = robot_model->getEdfRadius() * collision_padding_rate;
          auto res = tangent_calc(r, r, rel_pos);
          double theta1 = res.first;
          double theta2 = res.second;

          Eigen::Vector3d angle_v(pitch_angle, theta1, theta2);
          bound_list.push_back(std::make_pair(j, angle_v));
        }

      // check the overlap with center link if it exists
      std::string name("center_link");
      auto it = seg_tf_map.find(name);
      if (it != seg_tf_map.end())
        {
          auto pose_c = it->second; // pose w.r.t. Baselink (Root)
          auto rel_pose = pose_i.Inverse() * pose_c; // relativel position from the i-th rotor
          auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

          double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle

          if (fabs(angle) < angle_max_thresh)
            {
              // calculate the bound angle
              auto geo = robot_model->getUrdfModel().getLink(name)->collision->geometry;
              auto type = geo->type;

              double r_r = robot_model->getEdfRadius() * collision_padding_rate;
              double r_c = 0;

              if (type == urdf::Geometry::CYLINDER)
                {
                  r_c = std::dynamic_pointer_cast<urdf::Cylinder>(geo)->radius;
                }
              else if (type == urdf::Geometry::SPHERE)
                {
                  r_c = std::dynamic_pointer_cast<urdf::Sphere>(geo)->radius;
                }
              else if (type == urdf::Geometry::BOX)
                {
                  auto dim = std::dynamic_pointer_cast<urdf::Box>(geo)->dim;
                  r_c = std::hypot(dim.x, dim.y);
                }
              else
                {
                  ROS_WARN_THROTTLE(1.0, "Currently, urdf geometry type %d is not supported for rotor interference avoidance", type);
                }

              if (r_c > 0)
                {
                  auto res = tangent_calc(r_r, r_c, rel_pos);
                  double theta1 = res.first;
                  double theta2 = res.second;
                  Eigen::Vector3d angle_v(angle, theta1, theta2);
                  bound_list.push_back(std::make_pair(-1, angle_v));
                }
            }
        }

      // check the overlap with extra module if it exists
      int extra_cnt = 0;
      for (const auto& it: robot_model->getExtraModuleMap())
        {
          auto pose_parent = seg_tf_map.find(it.second.first.getName())->second;
          auto pose_e = pose_parent * it.second.first.getFrameToTip(); // pose w.r.t. Baselink (Root)
          auto rel_pose = pose_i.Inverse() * pose_e; // relativel position from the i-th rotor
          auto rel_pos = aerial_robot_model::kdlToEigen(rel_pose.p);

          double angle = atan2(-rel_pos.x(), -rel_pos.z()); // only consider the pitch angle
          if (fabs(angle) > angle_max_thresh) continue;

          // calculate the bound angle
          double r_e = 0;
          auto dim = it.second.second;
          if (dim(1) == 0 && dim(2) == 0) r_e = dim(0); // sphere
          if (dim(1) > 0 && dim(2) == 0) r_e = dim(1); // clyinder
          if (dim(1) > 0 && dim(2) > 0) r_e = std::hypot(dim(0), dim(1)); // box

          double r_r = robot_model->getEdfRadius() * collision_padding_rate;

          auto res = tangent_calc(r_r, r_e, rel_pos);
          double theta1 = res.first;
          double theta2 = res.second;

          Eigen::Vector3d angle_v(angle, theta1, theta2);
          // ROS_INFO_STREAM("rotor " << i+1 << ": " << it.first << ": rel_pose" << rel_pos.transpose() << ": angle_v: " << angle_v.transpose());
          extra_cnt ++;
          bound_list.push_back(std::make_pair(-1 - extra_cnt, angle_v));
        }

      if (bound_list.size() == 0) continue;
      interfere_raw_map.insert(std::make_pair(i, bound_list));

      // update bounds
      std::vector<std::pair<double, double>> bounds {std::make_pair(-M_PI/2, M_PI/2)};
      double area_thresh = 0.4; // TODO: rosparam
      for (auto& new_bound : bound_list)
        {
          double theta1, theta2;
          if (new_bound.second(1) < new_bound.second(2))
            {
              theta1 = new_bound.second(1);
              theta2 = new_bound.second(2);
            }
          else
            {
              theta1 = new_bound.second(2);
              theta2 = new_bound.second(1);
            }

          std::vector<std::pair<double, double>> bounds_temp {};
          for (auto& curr_bound: bounds)
            {
              double curr_theta1 = curr_bound.first;
              double curr_theta2 = curr_bound.second;

              // 6 cases:
              if (theta2 < curr_theta1)
                {
                  bounds_temp.push_back(curr_bound);
                }

              if (theta1 < curr_theta1 && theta2 > curr_theta1 && theta2 < curr_theta2)
                {
                  if (curr_theta2 - theta2 < area_thresh) continue;

                  bounds_temp.push_back(std::make_pair(theta2, curr_theta2));
                }

              if (theta1 < curr_theta1 && theta2 > curr_theta2)
                {
                  // skip
                  continue;
                }

              if (theta1 > curr_theta1 && theta2 < curr_theta2)
                {
                  if (theta1 - curr_theta1 > area_thresh)
                    {
                      bounds_temp.push_back(std::make_pair(curr_theta1, theta1));
                    }
                  if (curr_theta2 - theta2 > area_thresh)
                    {
                      bounds_temp.push_back(std::make_pair(theta2, curr_theta2));
                    }
                }

              if (theta1 > curr_theta1 && theta1 < curr_theta2 && theta2 > curr_theta2)
                {
                  if (theta1 - curr_theta1 < area_thresh) continue;

                  bounds_temp.push_back(std::make_pair(curr_theta1, theta1));
                }

              if (theta1 > curr_theta2)
                {
                  bounds_temp.push_back(curr_bound);
                }
            }

          bounds = bounds_temp;
        }
      bounds_map.insert(std::make_pair(i, bounds));

      double lower = 1e6;
      double upper = 0;
      for (const auto& bound: bounds)
        {
          double theta1 = bound.first;
          double theta2 = bound.second;

          if (fabs(theta1) < fabs(lower))
            {
              lower = theta1;
              upper = theta2;
            }

          if (fabs(theta2) < fabs(lower))
            {
              lower = theta2;
              upper = theta1;
            }
        }

      // skip if the range is enough for normal tilting
      if (lower * upper < 0 && fabs(lower) > area_thresh / 2) continue;

      prime_bound_map.insert(std::make_pair(i, std::make_pair(lower, upper)));


      if (roll_locked_gimbal.at(i) == 0)
        {
          roll_locked_gimbal.at(i) = 1;
          gimbal_nominal_angles.at(2 * i) = 0;
        }
    }

  if (interfere_raw_map.size() > 0)
    {
      for (const auto& it: interfere_raw_map)
        {
          int rotor_id = it.first;
          ss_map << "rotor" << rotor_id+1 << ": \n";
          for(const auto& bound: it.second)
            {
              int index = bound.first;
              if (index >= 0)
                {
                  ss_map << "\t rotor" << bound.first + 1 << ": " << bound.second.transpose() << " \n";
                }
              else if (index == -1)
                {
                  ss_map << "\t center link: " << bound.second.transpose() << " \n";
                }
              else
                {
                  ss_map << "\t extra module: " << bound.second.transpose() << " \n";
                }
            }

          for(const auto& bound: bounds_map.at(rotor_id))
            {
              ss_map << "\t candidate bound: (" << bound.first << ", " << bound.second << ") \n";
            }

          auto bound = prime_bound_map.find(rotor_id);
          if (bound != prime_bound_map.end())
            {
              ss_map << "\t prime bound: (" << bound->second.first << ", " << bound->second.second << ") \n";
            }
        }

      // ROS_INFO_STREAM_THROTTLE(1.0, "\033[32m" << ss_map.str() << "\033[0m");
    }
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
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false);
  getParam<double>(control_nh, "allocation_refine_threshold", allocation_refine_threshold_, 0.01);
  getParam<int>(control_nh, "allocation_refine_max_iteration", allocation_refine_max_iteration_, 1);

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
