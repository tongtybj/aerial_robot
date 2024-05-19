#include <spidar/navigation/terrestrial/base.h>
#include <spidar/control/walk_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Spider::Terrestrial;

Base::Base():
  BaseNavigator(),
  joint_index_map_(0),
  target_baselink_pos_(0,0,0),
  target_baselink_vel_(0,0,0),
  target_baselink_rpy_(0,0,0),
  target_leg_ends_(0),
  target_link_rots_(0),
  reset_baselink_flag_(false),
  reset_leg_ends_flag_(false),
  free_leg_id_(-1),
  raise_leg_flag_(false),
  lower_leg_flag_(false),
  raise_converge_(false)
{
}

void Base::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                      double loop_du)
{
  /* initialize the flight control */
  BaseNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  spidar_robot_model_ = boost::dynamic_pointer_cast<::Spider::GroundRobotModel>(robot_model);
  robot_model_for_nav_ = boost::make_shared<aerial_robot_model::RobotModel>();


  target_baselink_pos_sub_ = nh_.subscribe("walk/baselink/traget/pos", 1, &Base::targetBaselinkPosCallback, this);
  target_baselink_delta_pos_sub_ = nh_.subscribe("walk/baselink/traget/delta_pos", 1, &Base::targetBaselinkDeltaPosCallback, this);

  raise_leg_sub_ = nh_.subscribe("walk/raise_leg", 1, &Base::raiseLegCallback, this);
  lower_leg_sub_ = nh_.subscribe("walk/lower_leg", 1, &Base::lowerLegCallback, this);

  target_leg_ends_pub_ = nh_.advertise<geometry_msgs::PoseArray>("debug/nav/target_leg_ends", 1); // for debug

  if(kinematic_simulation_) {
    sim_baselink_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ground_truth", 1);
    sim_joint_angles_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    sim_flight_config_sub_ = nh_.subscribe("flight_config_cmd", 1, &Base::simulateFlightConfigCallback, this);
    sim_flight_config_pub_ = nh_.advertise<std_msgs::UInt8>("flight_config_ack", 1);

    sim_joint_state_.position.resize(0);
    sim_joint_state_.name.resize(0);

    XmlRpc::XmlRpcValue params;
    nh_.getParam("zeros", params);
    for(auto param: params) {
      // ROS_INFO_STREAM(param.first << ": " << param.second);
      sim_joint_state_.name.push_back(param.first);
      sim_joint_state_.position.push_back(param.second);
    }
    sensor_msgs::JointState joint_msg = sim_joint_state_;
    joint_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < spidar_robot_model_->getRotorNum(); i++) {
      joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_roll"));
      joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_pitch"));
      joint_msg.position.push_back(0);
      joint_msg.position.push_back(0);
    }
    sim_joint_angles_pub_.publish(joint_msg);
  }
}

void Base::update()
{
  if (getNaviState() == aerial_robot_navigation::START_STATE) {
    if(estimator_->getUnhealthLevel() == Sensor::UNHEALTH_LEVEL3){
      ROS_WARN("Sensor Unhealth, cannot arming");
      setNaviState(ARM_OFF_STATE);
      return;
    }
  }

  BaseNavigator::update();

  // skip before the model initialization
  if (spidar_robot_model_->getMass() == 0) {
    return;
  }

  if(!checkKinematics()) {
    return;
  }

  if(!inverseKinematics()) {
    return;
  }

  // update robot model for navigation
  updateRobotModelForNav();

  // additional action for free leg
  freeLegAction();

  // action for failsafe
  failSafeAction();

  // only simulate joint motions
  kinematicSimulate();
}

// calculate target joint angle from target baselink based on IK
bool Base::inverseKinematics()
{
  const int leg_num = spidar_robot_model_->getRotorNum() / 2;
  const auto& seg_tf_map = spidar_robot_model_->getSegmentsTf();

  // calculate the target joint angles from baselink and end position
  KDL::Frame fw_target_baselink;
  fw_target_baselink.M
    = KDL::Rotation::EulerZYX(target_baselink_rpy_.z(),
                              target_baselink_rpy_.y(),
                              target_baselink_rpy_.x());
  tf::vectorTFToKDL(target_baselink_pos_, fw_target_baselink.p);

  double l0 = -1; // distance from "joint_junction_linkx" to "linkx"
  double l1 = -1; // distance from "linkx" to "linkx+1"
  double l2 = -1; // distance from "linkx+1" to "linkx+1_foot"

  const auto& names = target_joint_state_.name;
  auto& target_angles = target_joint_state_.position;
  KDL::Frame fr_baselink = seg_tf_map.at(spidar_robot_model_->getBaselinkName());

  std::stringstream ss;

  for (int i = 0; i < leg_num; i++) {
    std::string yaw_frame_name = std::string("joint_junction_link") + std::to_string(2 * i + 1);
    KDL::Frame fr_joint_yaw = seg_tf_map.at(yaw_frame_name); // w.r.t. root link
    fr_joint_yaw.M = KDL::Rotation::EulerZYX(M_PI/ 4 + M_PI/2 * i, 0, 0);  // workaround: get the orientation of the parent frame
    KDL::Frame fb_joint_yaw = fr_baselink.Inverse() * fr_joint_yaw; // w.r.t. baselink
    KDL::Frame fw_joint_yaw = fw_target_baselink * fb_joint_yaw; // w.r.t. world frame

    KDL::Frame fw_end = target_leg_ends_.at(i);
    KDL::Frame fy_end = fw_joint_yaw.Inverse() * fw_end; // w.r.t. yaw (yaw1) frame.
    KDL::Vector p = fy_end.p;
    // if (i == 0)
    //   ROS_ERROR_STREAM_THROTTLE(1.0, "fw_end pos: " << aerial_robot_model::kdlToEigen(p).transpose());

    double angle = atan2(p.y(), p.x());

    std::string pitch1_frame_name = std::string("link") + std::to_string(2 * i + 1);
    std::string pitch2_frame_name = std::string("link") + std::to_string(2 * (i + 1));
    std::string end_frame_name = std::string("link") + std::to_string(2 * (i + 1)) + std::string("_foot");

    if(l0 < 0) {
      KDL::Frame fr_joint_pitch1 = seg_tf_map.at(pitch1_frame_name);
      KDL::Frame fr_joint_pitch2 = seg_tf_map.at(pitch2_frame_name);
      KDL::Frame fr_end = seg_tf_map.at(end_frame_name);

      l0 = (fr_joint_pitch1.p - fr_joint_yaw.p).Norm();
      l1 = (fr_joint_pitch2.p - fr_joint_pitch1.p).Norm();
      l2 = (fr_end.p - fr_joint_pitch2.p).Norm();
    }

    // TODO: hard-coding for the kinemtaics from "joint_junction_linkx" to "linkx"
    KDL::Frame frame_delta1(KDL::Rotation::EulerZYX(angle, 0, 0), KDL::Vector::Zero());
    KDL::Frame frame_delta2(KDL::Rotation::Identity(), KDL::Vector(l0, 0, 0));
    KDL::Frame fr_joint_pitch1 = fr_joint_yaw * frame_delta1 * frame_delta2;
    // if (i == 0)
    //   ROS_ERROR_STREAM_THROTTLE(1.0, "l0: " << l0 << "; fr_joint_pitch1: " << aerial_robot_model::kdlToEigen(fr_joint_pitch1.p).transpose() << "; fr_joint_yaw: " << aerial_robot_model::kdlToEigen(fr_joint_yaw.p).transpose());
    KDL::Frame fb_joint_pitch1 = fr_baselink.Inverse() * fr_joint_pitch1; // w.r.t. baselink
    KDL::Frame fw_joint_pitch1 = fw_target_baselink * fb_joint_pitch1; // w.r.t. world frame

    KDL::Frame fp1_end = fw_joint_pitch1.Inverse() * fw_end;  // w.r.t. pitch1 frame
    double x_e = fp1_end.p.x();
    double y_e = - fp1_end.p.z(); // w.r.t pitch1 frame
    double b = l1 * l1 - x_e * x_e - y_e * y_e - l2 * l2;
    double b_d = b / (- 2 * l2);
    double d = sqrt(x_e * x_e + y_e * y_e);
    double b_dd = b_d / d;
    if (fabs(b_dd) > 1) {
      ROS_ERROR_STREAM("[Spider][Navigator] leg " << i + 1 << ": b_dd exceeds the valid scope: " << b_dd
                       << "; end pos: " << aerial_robot_model::kdlToEigen(fw_end.p).transpose()
                       << "; baselink pos: " << aerial_robot_model::kdlToEigen(fw_target_baselink.p).transpose());
      return false;
    }

    double phi = atan2(y_e, x_e);
    double theta2_d = acos(b_dd) + phi;
    if (theta2_d < 0) {
      theta2_d = - acos(b_dd) + phi;
    }
    double theta1 = atan2(y_e - l2 * sin(theta2_d), x_e - l2 * cos(theta2_d));
    double theta2 = theta2_d - theta1;

    // check validity
    double angle_limit = 1.74;
    if(fabs(angle) > angle_limit ) {
      ROS_ERROR_STREAM("[Spider][Navigator] joint" << i * 2 + 1 << "_yaw exceeds the valid range, angle is " << angle);
      return false;
    }

    angle_limit = 1.7;
    if(fabs(theta1) > angle_limit ) {
      ROS_ERROR_STREAM("[Spider][Navigator] joint" << i * 2 + 1 << "_pitch exceeds the valid range, angle is " << theta1);
      return false;
    }

    if(fabs(theta2) > angle_limit) {
      ROS_ERROR_STREAM("[Spider][Navigator] joint" << i * 2 + 2 << "_pitch exceeds the valid range, angle is " << theta2);
      return false;
    }

    // set the target joint angles
    if(names.at(4 * i) != std::string("joint") + std::to_string(2*i+1) + std::string("_yaw")) {
      ROS_ERROR_STREAM("[Spider][Navigator] name order is different. ID" << i << " name is " << names.at(4 * i));
      return false;
    }

    // special process for raising leg
    if (i == free_leg_id_ || leg_num == free_leg_id_) {

      // simple action
      if (raise_leg_flag_) theta1 -= raise_angle_;
    }

    target_angles.at(4 * i) = angle;
    target_angles.at(4 * i + 1) = theta1;
    target_angles.at(4 * i + 2) = 0;
    target_angles.at(4 * i + 3) = theta2;

    ss << "(" << target_angles.at(4 * i) << ", " << target_angles.at(4 * i + 1) << ", "
       << target_angles.at(4 * i + 2) << ", " << target_angles.at(4 * i + 3) << ") ";
  }
  ROS_DEBUG_STREAM_THROTTLE(1.0, "[Spider][Walk][Navigator] analytical joint angles from leg end and baselink: " << ss.str());

  return true;
}

void Base::updateRobotModelForNav()
{
  // update whole joint angles
  auto joint_state = spidar_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  for(int i = 0; i < joint_index_map_.size(); i++) {
    auto id = joint_index_map_.at(i);
    joint_state.position.at(id) = target_joint_state_.position.at(i);
  }

  // update robot model
  robot_model_for_nav_->updateRobotModel(joint_state);
  const auto& target_seg_tf_map = robot_model_for_nav_->getSegmentsTf();

  const auto& seg_tf_map = spidar_robot_model_->getSegmentsTf();
  KDL::Frame fr_baselink = seg_tf_map.at(spidar_robot_model_->getBaselinkName());

  KDL::Frame fw_target_baselink;
  fw_target_baselink.M
    = KDL::Rotation::EulerZYX(target_baselink_rpy_.z(),
                              target_baselink_rpy_.y(),
                              target_baselink_rpy_.x());


  // update the target rotation of all links
  for(int i = 0; i < spidar_robot_model_->getRotorNum(); i++) {
    std::string link_name = std::string("link") + std::to_string(i+1);
    KDL::Frame fr_target_link = target_seg_tf_map.at(link_name);
    KDL::Frame fb_target_link = fr_baselink.Inverse() * fr_target_link;
    KDL::Frame fw_target_link = fw_target_baselink * fb_target_link;
    target_link_rots_.at(i) = fw_target_link.M;
  }


  geometry_msgs::PoseArray poses_msg;
  poses_msg.header.stamp = ros::Time::now();
  for(const auto& f: target_leg_ends_) {
    geometry_msgs::Pose pose;
    tf::poseKDLToMsg(f, pose);
    poses_msg.poses.push_back(pose);
  }
  target_leg_ends_pub_.publish(poses_msg);
}

void Base::kinematicSimulate()
{
  if(!kinematic_simulation_) {
    return;
  }

  // init baselink
  if (target_leg_ends_.at(0).p.z() < 0) {
    // set the link end conntacting the ground
    double z_offset = - target_leg_ends_.at(0).p.z();

    for (auto& f: target_leg_ends_) {
      f.p += KDL::Vector(0, 0, z_offset);
    }
    target_baselink_pos_ += tf::Vector3(0, 0, z_offset);
  }

  // baselink odometry
  nav_msgs::Odometry odom_msg;
  tf::pointTFToMsg(target_baselink_pos_, odom_msg.pose.pose.position);
  odom_msg.pose.pose.orientation
    = tf::createQuaternionMsgFromRollPitchYaw(target_baselink_rpy_.x(),
                                              target_baselink_rpy_.y(),
                                              target_baselink_rpy_.z());
  sim_baselink_pose_pub_.publish(odom_msg);

  // joint states
  double r = sim_joint_lpf_rate_;
  for(int i = 0; i < sim_joint_state_.position.size(); i++) {
    std::string name = sim_joint_state_.name.at(i);
    auto res = std::find(target_joint_state_.name.begin(), target_joint_state_.name.end(), name);
    int j = std::distance(target_joint_state_.name.begin(), res);
    double angle = target_joint_state_.position.at(j);
    sim_joint_state_.position.at(i) =
      (1 - r) * sim_joint_state_.position.at(i) + r * angle;
  }
  sensor_msgs::JointState joint_msg = sim_joint_state_;
  joint_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < spidar_robot_model_->getRotorNum(); i++) {
    joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_roll"));
    joint_msg.name.push_back(std::string("gimbal") + std::to_string(i+1) + std::string("_pitch"));
    joint_msg.position.push_back(0);
    joint_msg.position.push_back(0);
  }

  sim_joint_angles_pub_.publish(joint_msg);
}

void Base::freeLegAction()
{
  if (free_leg_id_ == -1) return;

  const int leg_num = spidar_robot_model_->getRotorNum() / 2;
  const auto current_joint_angles = getCurrentJointAngles();


  // raise leg
  if (raise_leg_flag_) {

    bool all_raise_converge = false;

    for (int i = 0; i < leg_num; i++) {

      if (free_leg_id_ != i && free_leg_id_ != leg_num) continue;

      double curr_angle = current_joint_angles.at(4 * i + 1);
      double target_angle = target_joint_state_.position.at(4 * i + 1);

      if (curr_angle - target_angle > raise_converge_thresh_) {
        all_raise_converge = false;
        continue;
      }

      // converge to the raise leg
      all_raise_converge = true;


      if (!raise_converge_) {
        ROS_INFO_STREAM("[Spider][Walk][Navigator] leg" << i + 1 << " reach the raise goal, joint" << i * 2 +1 << "_pitch" << ", curr angle: " << curr_angle << "; target angle: " << target_angle << ", raise_converge_threshold: " << raise_converge_thresh_);
      }
    }

    if (all_raise_converge && !raise_converge_) {
      ROS_INFO_STREAM("[Spider][Walk][Navigator] all legs raise to the desicred joint angles");
      raise_converge_ = true;
    }
  }

  // lower leg
  if (lower_leg_flag_) {

    bool all_contact = false;
    double t = ros::Time::now().toSec();

    if (t - contact_check_prev_t_ > check_interval_) {

      for (int i = 0; i < leg_num; i++) {

        double curr_angle = current_joint_angles.at(4 * i + 1);
        double prev_angle = contact_check_prev_joint_angles_.at(4 * i + 1);
        double target_angle = target_joint_state_.position.at(4 * i + 1);

        if (free_leg_id_ != i && free_leg_id_ != leg_num) continue;

        // touch detection rule:
        // - small delta angle around the touch-down target angle
        // - constant angle for a while
        // - certain duration from start time
        if (target_angle - curr_angle < lower_touchdown_thresh_ &&
            fabs(curr_angle - prev_angle) < constant_angle_thresh_) {

          if (t - contact_check_start_t_ > converge_time_thresh_) {
            all_contact = true;

            ROS_INFO_STREAM("[Spider][Walk][Navigator] leg" << i + 1 << " touches the ground, joint" << i * 2 +1 << "_pitch" << ", curr angle: " << curr_angle << "; target angle: " << target_angle);
            continue;
          }
        } else {
          contact_check_start_t_ = t;
        }

        all_contact = false;

      }

      contact_check_prev_joint_angles_ = current_joint_angles;
      contact_check_prev_t_ = t;

    }

    if (all_contact) {
      ROS_INFO_STREAM("[Spider][Walk][Navigator] all legs contact to the ground");
      contactLeg();
    }

  }
}

void Base::failSafeAction()
{
  // baselink rotation
  tf::Matrix3x3 rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_);
  tf::Vector3 zb = rot * tf::Vector3(0,0,1);
  double theta = atan2(sqrt(zb.x() * zb.x() + zb.y() * zb.y()), zb.z());

  if (theta > baselink_rot_thresh_){
    ROS_WARN_STREAM("[Spider][Walk][Navigation] baselink rotation is abnormal, tool tiled: " << theta);

    if (raise_leg_flag_) {
      lowerLeg();
      ROS_WARN_STREAM("[Spider][Walk][Navigation] instantly lower the raised leg" << free_leg_id_ + 1);
    }
  }

  // pitch joint of opposite of raise leg
  if (raise_leg_flag_) {
    int leg_num = spidar_robot_model_->getRotorNum() / 2;
    int leg_id = (free_leg_id_ + leg_num / 2) % leg_num;
    int j = 4 * leg_id + 1;
    double target_angle = target_joint_state_.position.at(j);
    double current_angle = getCurrentJointAngles().at(j);
    std::string name = target_joint_state_.name.at(j);

    //ROS_INFO_STREAM("[Spider][Walk][Navigation] " << name << ", target angle  " << target_angle << ", current angle: " << current_angle);
    if (target_angle - current_angle > opposite_raise_leg_thresh_) {
      ROS_WARN_STREAM("[Spider][Walk][Navigation] " << name << " is overload because of raising leg, target angle  " << target_angle << ", current angle: " << current_angle);
      ROS_WARN_STREAM("[Spider][Walk][Navigation] instantly lower the raising leg" << free_leg_id_ + 1);
      lowerLeg();
    }
  }
}

bool Base::checkKinematics()
{
  // get current leg position w.r.t. world frame
  std::vector<KDL::Frame> curr_leg_ends;
  if(!getCurrentLegEndsPos(curr_leg_ends)) {
    return false;
  }

  // target joint angles
  if (target_joint_state_.name.size() == 0) {

    // initialize the target joint
    target_joint_state_.name = spidar_robot_model_->getLinkJointNames();
    target_joint_state_.position.resize(target_joint_state_.name.size());

    // set (initialize) joint index map
    setJointIndexMap();

    // set target joint angles
    target_joint_state_.position = getCurrentJointAngles();
  }

  auto current_joint_angles = getCurrentJointAngles();

  // target link rotation
  if (target_link_rots_.size() == 0) {
    target_link_rots_.resize(spidar_robot_model_->getRotorNum());
  }

  // initialize targets
  if (target_leg_ends_.size() == 0) {
    reset_baselink_flag_ = true;
    reset_leg_ends_flag_ = true;
  }

  // reset targets
  if (getNaviState() == aerial_robot_navigation::START_STATE) {
    ROS_INFO("[Walk][Navigator] set initial position and joint angles as target ones");

    reset_baselink_flag_ = true;
    reset_leg_ends_flag_ = true;
  }

  // update target baselink pose
  if (reset_baselink_flag_) {
    tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
    tf::Vector3 baselink_rpy = estimator_->getEuler(Frame::BASELINK, estimate_mode_);
    resetTargetBaselink(baselink_pos, baselink_rpy);

    reset_baselink_flag_ = false;
  }

  // update target leg end frame (position)
  if (reset_leg_ends_flag_) {
    resetTargetLegEnds(curr_leg_ends);
    reset_leg_ends_flag_ = false;
  }

  return true;
}

void Base::resetTargetBaselink(tf::Vector3 pos, tf::Vector3 rpy)
{
  target_baselink_pos_ = pos;
  target_baselink_rpy_ = rpy;
}

void Base::resetTargetLegEnds(std::vector<KDL::Frame> frames)
{
  target_leg_ends_ = frames;
}


void Base::setJointIndexMap()
{
  const auto joint_state = spidar_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  const auto& search_v = joint_state.name;

  joint_index_map_.resize(0); // resize
  const auto& joint_names = target_joint_state_.name;
  for(const auto& n: joint_names) {

    auto res = std::find(search_v.begin(), search_v.end(), n);

    if (res == search_v.end()) {
      ROS_ERROR_STREAM("[Spider][Navigator] joint index mapping, cannot find " << n);
      continue;
    }

    auto id = std::distance(search_v.begin(), res);

    joint_index_map_.push_back(id);
  }
}


tf::Vector3 Base::getCurrentBaselinkPos()
{
  return estimator_->getPos(Frame::BASELINK, estimate_mode_);
}

std::vector<double> Base::getCurrentJointAngles()
{
  const auto joint_state = spidar_robot_model_->getGimbalProcessedJoint<sensor_msgs::JointState>();
  std::vector<double> angles(0);
  for(const auto id: joint_index_map_) {
    angles.push_back(joint_state.position.at(id));
  }

  return angles;
}

bool Base::getCurrentLegEndsPos(std::vector<KDL::Frame>& leg_ends_pos)
{
  const auto& seg_tf_map = spidar_robot_model_->getSegmentsTf();
  if(seg_tf_map.size() == 0) return false;


  tf::Transform tf_w_baselink(estimator_->getOrientation(Frame::BASELINK, estimate_mode_),
                              estimator_->getPos(Frame::BASELINK, estimate_mode_));
  KDL::Frame fw_baselink;
  tf::transformTFToKDL(tf_w_baselink, fw_baselink);

  std::vector<KDL::Frame> curr_leg_ends;
  const int leg_num = spidar_robot_model_->getRotorNum() / 2;
  KDL::Frame fr_baselink = seg_tf_map.at(spidar_robot_model_->getBaselinkName());

  // get current leg position w.r.t. world frame
  for (int i = 0; i < leg_num; i++) {
    std::string frame_name = std::string("link") + std::to_string(2 * (i + 1)) + std::string("_foot");
    KDL::Frame fr_end = seg_tf_map.at(frame_name); // w.r.t. root link
    KDL::Frame fb_end = fr_baselink.Inverse() * fr_end; // w.r.t. baselink
    KDL::Frame fw_end = fw_baselink * fb_end; // w.r.t. world frame
    leg_ends_pos.push_back(fw_end);
  }

  return true;
}


void Base::halt()
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
  std_srvs::SetBool srv;
  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the joint torque");
  else
    ROS_ERROR("Failed to call service joints/torque_enable");

  client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

  srv.request.data = false;
  if (client.call(srv))
    ROS_INFO("dragon control halt process: disable the gimbal torque");
  else
    ROS_ERROR("Failed to call service gimbals/torque_enable");
}


void Base::raiseLeg(int leg_id)
{
  free_leg_id_ = leg_id;

  raise_leg_flag_ = true;
  raise_converge_ = false;
  lower_leg_flag_ = false;

  spidar_robot_model_->setFreeleg(free_leg_id_);
  walk_controller_->startRaiseLeg();
}

void Base::lowerLeg()
{
  lower_leg_flag_ = true;
  raise_leg_flag_ = false;
  raise_converge_ = false;
  walk_controller_->startLowerLeg();

  resetContactStatus(); // reset contact variables
}

void Base::contactLeg()
{
  lower_leg_flag_ = false;
  raise_leg_flag_ = false;
  raise_converge_ = false;
  spidar_robot_model_->resetFreeleg();
  walk_controller_->startContactTransition(free_leg_id_);
  free_leg_id_ = -1;
}

void Base::resetContactStatus()
{
  contact_check_prev_joint_angles_ = getCurrentJointAngles();
  contact_check_start_t_ = ros::Time::now().toSec();
  contact_check_prev_t_ = ros::Time::now().toSec();
}


void Base::rosParamInit()
{
  BaseNavigator::rosParamInit();

  ros::NodeHandle nh_walk(nh_, "navigation/walk");
  getParam<double>(nh_walk, "raise_angle", raise_angle_, 0.2);
  getParam<double>(nh_walk, "lower_touchdown_thresh", lower_touchdown_thresh_, 0.05);
  getParam<double>(nh_walk, "raise_converge_thresh", raise_converge_thresh_, 0.05);
  getParam<double>(nh_walk, "constant_angle_thresh", constant_angle_thresh_, 0.02);
  getParam<double>(nh_walk, "check_interval", check_interval_, 0.1);
  getParam<double>(nh_walk, "converge_time_thresh", converge_time_thresh_, 0.5);
  getParam<bool>(nh_walk, "kinematic_simulation", kinematic_simulation_, false);
  getParam<double>(nh_walk, "sim_joint_lpf_rate", sim_joint_lpf_rate_, 0.5);

  ros::NodeHandle nh_failsafe(nh_walk, "failsafe");
  getParam<double>(nh_failsafe, "baselink_rot_thresh", baselink_rot_thresh_, 0.2);
  getParam<double>(nh_failsafe, "opposite_raise_leg_thresh", opposite_raise_leg_thresh_, 0.1);


  if (kinematic_simulation_) {
    // WIP: reset the servo angle bias for simulation
    ros::NodeHandle nh_control(nh_, "controller/walk");
    nh_control.setParam("servo_angle_bias", 0);
  }

}

void Base::targetBaselinkPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::vector3MsgToTF(msg->vector, target_baselink_pos_);
}

void Base::targetBaselinkDeltaPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::Vector3 delta_pos;
  tf::vector3MsgToTF(msg->vector, delta_pos);
  target_baselink_pos_ += delta_pos;

  ROS_ERROR("get new target baselink");
}

void Base::raiseLegCallback(const std_msgs::UInt8ConstPtr& msg)
{
  raiseLeg(msg->data);
}

void Base::lowerLegCallback(const std_msgs::EmptyConstPtr& msg)
{
  lowerLeg();
}

void Base::simulateFlightConfigCallback(const spinal::FlightConfigCmdConstPtr& msg)
{
  std_msgs::UInt8 ack_msg;
  switch(msg->cmd)
    {
    case spinal::FlightConfigCmd::ARM_ON_CMD:
      ack_msg.data = spinal::FlightConfigCmd::ARM_ON_CMD;
      sim_flight_config_pub_.publish(ack_msg);
      break;
    case spinal::FlightConfigCmd::ARM_OFF_CMD:
      ack_msg.data = spinal::FlightConfigCmd::ARM_OFF_CMD;
      sim_flight_config_pub_.publish(ack_msg);
      break;
    case spinal::FlightConfigCmd::FORCE_LANDING_CMD:
      ack_msg.data = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
      sim_flight_config_pub_.publish(ack_msg);
      break;
    default:
      break;
    }
}

void Base::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
{
  sensor_msgs::Joy joy_cmd;
  if(joy_msg->axes.size() == PS3_AXES && joy_msg->buttons.size() == PS3_BUTTONS) {
      joy_cmd = (*joy_msg);
  }
  else if(joy_msg->axes.size() == PS4_AXES && joy_msg->buttons.size() == PS4_BUTTONS) {
      joy_cmd = ps4joyToPs3joyConvert(*joy_msg);
  }
  else {
      ROS_WARN("the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(), (int)joy_msg->axes.size());
      return;
  }

  static sensor_msgs::Joy prev_joy_cmd = joy_cmd;

  if(joy_cmd.buttons[PS3_BUTTON_REAR_RIGHT_2] == 1) {
    /* all joint servo */
    if (joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo on */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = true;

      if (client.call(srv))
        ROS_INFO("[Spider][Joy] enable alll joint torque");
      else
        ROS_ERROR("Failed to call service joints/torque_enable");

      client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

      if (client.call(srv))
        ROS_INFO("[Spider][Joy] enable alll gimbal torque");
      else
        ROS_ERROR("Failed to call service gimbals/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;

    }

    if (joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo off */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joints/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = false;

      if (client.call(srv))
        ROS_INFO("[Spider][Joy] disable the all joint torque");
      else
        ROS_ERROR("Failed to call service joints/torque_enable");

      client = nh_.serviceClient<std_srvs::SetBool>("gimbals/torque_enable");

      if (client.call(srv))
        ROS_INFO("[Spider][Joy] disable the all gimbal torque");
      else
        ROS_ERROR("Failed to call service gimbals/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;
    }
  } else {
    /* joint pitch servo */
    if (joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_TRIANGLE] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo on */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = true;

      if (client.call(srv))
        ROS_INFO("[Spider][Joy] enable the pitch joint torque");
      else
        ROS_ERROR("Failed to call service joint_pitch/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;

    }

    if (joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_ACTION_CROSS] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      /* servo off */
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = false;

      if (client.call(srv))
        ROS_INFO("[Spider][Joy] disable the pitch joint torque");
      else
        ROS_ERROR("Failed to call service joint_pitch/torque_enable");

      prev_joy_cmd = joy_cmd;
      return;
    }
  }


  if (joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_1] == 1) {

    if (joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      target_baselink_pos_ += tf::Vector3(0, 0, 0.2);
      ROS_INFO_STREAM("stand-up");

      prev_joy_cmd = joy_cmd;
      return;
    }

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {
      if (prev_joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {
        prev_joy_cmd = joy_cmd;
        return;
      }

      target_baselink_pos_ -= tf::Vector3(0, 0, 0.2);
      ROS_INFO_STREAM("sit-down");

      prev_joy_cmd = joy_cmd;
      return;
    }
  } else if (joy_cmd.buttons[PS3_BUTTON_REAR_RIGHT_1] == 1) {

    /* raise/lower test */
    if(joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {

      /* raise test */
      if (raise_leg_flag_) {
        return;
      }

      ROS_INFO("[Joy, raise all legs test]");

      int leg_num = spidar_robot_model_->getRotorNum() / 2;
      raiseLeg(leg_num);

      return;
    }

    /* lower test */
    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {
      if (lower_leg_flag_) {
        return;
      }

      if (!raise_leg_flag_) {
        return;
      }

      ROS_INFO("[Joy, lower leg1 down test]");
      lowerLeg();

      return;
    }

  }else {

    /* raise/lower test */
    if(joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {

      /* raise test */
      if (raise_leg_flag_) {
        return;
      }

      ROS_INFO("[Joy, raise leg1 test]");

      raiseLeg(0);

      return;
    }

    /* lower test */
    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {
      if (lower_leg_flag_) {
        return;
      }

      if (!raise_leg_flag_) {
        return;
      }

      ROS_INFO("[Joy, lower leg1 down test]");
      lowerLeg();

      return;
    }

  }


  /* quick land command */
  if(joy_cmd.buttons[PS3_BUTTON_ACTION_SQUARE] == 1 || joy_cmd.buttons[PS3_BUTTON_CROSS_RIGHT] == 1){

    if(getNaviState() == LAND_STATE) return;

    if(getNaviState() == ARM_ON_STATE)
      {
        setNaviState(STOP_STATE);
        ROS_ERROR("Joy Conrol: not land, but disarm motors directly");
      }
    return;
  }

  BaseNavigator::joyStickControl(joy_msg);

  prev_joy_cmd = joy_cmd;
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Spider::Terrestrial::Base, aerial_robot_navigation::BaseNavigator);
