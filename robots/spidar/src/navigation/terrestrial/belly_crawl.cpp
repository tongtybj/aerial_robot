#include <spidar/navigation/terrestrial/belly_crawl.h>
#include <spidar/control/walk_control.h> // for complate the class definition of class aerial_robot_control::Spider::WalkController

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Spider::Terrestrial;

BellyCrawl::BellyCrawl()
{
  phase_ = PHASE0;
  move_flag_ = false;

  belly_.phase_ = belly_.PHASE0;
  limb_.phase_ = belly_.PHASE0;
}


void BellyCrawl::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                            double loop_du)
{
  /* initialize with the super class */
  Base::initialize(nh, nhp, robot_model, estimator, loop_du);

  move_sub_ = nh_.subscribe("walk/belly_crawl/traget/delta_pos", 1, &BellyCrawl::moveCallback, this);

}

void BellyCrawl::rosParamInit()
{
  Base::rosParamInit();

  ros::NodeHandle nh_walk(nh_, "navigation/walk");
  ros::NodeHandle nh_belly_crawl(nh_walk, "belly_crawl");
  nh_belly_crawl.param("stride", stride_, 0.2);
  nh_belly_crawl.param("reset_leg_end", cycle_reset_leg_end_, false);
  nh_belly_crawl.param("belly_debug", belly_debug_, false);
  nh_belly_crawl.param("limb_debug", limb_debug_, false);

  ros::NodeHandle nh_belly_crawl_limb(nh_belly_crawl, "limbw");
  nh_belly_crawl_limb.param("joint_err_thresh", limb_.joint_err_thresh_, 0.05);
  nh_belly_crawl_limb.param("loop_duration", limb_.loop_duration_, 0.1);
  nh_belly_crawl_limb.param("servo_switch_duration", limb_.servo_switch_duration_, 0.5);

  ros::NodeHandle nh_belly_crawl_baselink(nh_belly_crawl, "baselink");
  nh_belly_crawl_baselink.param("loop_duration", belly_.loop_duration_, 0.1);
  nh_belly_crawl_baselink.param("servo_switch_duration", belly_.servo_switch_duration_, 0.5);
  nh_belly_crawl_baselink.param("raise_height", belly_.raise_height_, 0.1);
  nh_belly_crawl_baselink.param("raise_thresh", belly_.raise_thresh_, 0.05);
  nh_belly_crawl_baselink.param("move_thresh", belly_.move_thresh_, 0.05);
  nh_belly_crawl_baselink.param("descend_thresh", belly_.descend_thresh_, 0.02);
}


void BellyCrawl::update()
{
  Base::update();

  stateMachine();
}

void BellyCrawl::stateMachine()
{
  if (!move_flag_) {
    return;
  }

  switch(phase_) {

  case PHASE0:
    {
      iterativeUpdateTargetPos();
      phase_ = PHASE1;
      break;
    }
  case PHASE1:
    {
      if (belly_debug_) {
        phase_ = PHASE2;
        break;
      }

      limbSubStateMachine();
      break;
    }
  case PHASE2:
    {
      if (limb_debug_) {
        reset();
        break;
      }


      bellySubStateMachine();

      break;
    }
  default:
    {
      break;
    }
  }
}

void BellyCrawl::limbSubStateMachine()
{
  double t = ros::Time::now().toSec();

  if (t - limb_.prev_t_ < limb_.loop_duration_) {
    return;
  }

  const int limb_num = spidar_robot_model_->getRotorNum() / 2;

  switch(limb_.phase_) {

  case limb_.PHASE0:
    {
      // all legs contact to the ground
      std::string prefix("[Spider][Belly Crawl][Limb][Phase0]");

      // set the static belly mode
      walk_controller_->setFloatingBellyMode(false);

      // start raise all limbs
      raiseLeg(limb_num);

      // update the foot postion, and thus the joint angles
      for (auto& leg_end:  target_leg_ends_) {
        tf::Vector3 delta = target_pos_ - getTargetBaselinkPos();
        leg_end.p += KDL::Vector(delta.x(), delta.y(), delta.z());
      }

      // shift to PHASE1
      ROS_INFO_STREAM(prefix << " shift to PHASE1 for raise all limbs");
      limb_.phase_ = limb_.PHASE1;

      break;
    }
  case limb_.PHASE1:
    {
      // free leg is raising
      std::string prefix("[Spider][Belly Crawl][Limb][Phase1]");

      // 1. check the inside pitch joint of all limbs
      if (!raise_converge_) {
        break;
      }

      bool converge = true;

      // 2. check the inside yaw joint of each limb to lower leg
      for (int j = 0; j < limb_num; j++) {

        double target_angle = target_joint_state_.position.at(j);
        double current_angle = getCurrentJointAngles().at(j);
        double err = target_angle - current_angle;

        if (fabs(err) > limb_.joint_err_thresh_) {
          // joint angle error too big

          ROS_INFO_STREAM_THROTTLE(0.5, prefix << " the angle error of joint" << j/2 + 1 << "_yaw does not converge, " << err << "(" << limb_.joint_err_thresh_ << ")");

          converge = false;

          break;
        }
      }

      // 3. check the outside pitch joint of free leg to lower leg
      for (int j = 0; j < limb_num; j++) {

        double target_angle = target_joint_state_.position.at(j + 3);
        double current_angle = getCurrentJointAngles().at(j + 3);
        double err = target_angle - current_angle;

        if (fabs(err) > limb_.joint_err_thresh_ ) {
          // joint angle error too big

          ROS_INFO_STREAM_THROTTLE(0.5, prefix << " the angle error of joint" << j/2 + 2 << "_pitch does not converge, " << err << "(" << limb_.joint_err_thresh_ << ")");

          converge = false;

          break;
        }
      }

      if (!converge) {
        break;
      }

      // reach the target joint yaw to lower leg
      lowerLeg();

      // shift to PHASE2
      ROS_INFO_STREAM(prefix << " shift to PHASE2 for lower all limbs");
      limb_.phase_ = limb_.PHASE2;

      break;
    }
  case limb_.PHASE2:
    {
      // free leg is lowering
      std::string prefix("[Spider][Belly Crawl][Limb][Phase2]");

      if (lower_leg_flag_) {
        // still in lowering phase, skip
        ROS_INFO_STREAM_THROTTLE(0.5, prefix << " still in lowering phase, skip");

        break;
      }

      if (walk_controller_->getContactTransition()) {
        // still in contact transition, skip
        ROS_INFO_STREAM_THROTTLE(0.5, prefix << " finish lowering, but still in ground contact transition, skip");

        break;
      }

      // servo off the all pitch joints
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = false;

      if (client.call(srv)) {
        ROS_INFO_STREAM(prefix << " disable the pitch joint torque");
      } else {
        ROS_ERROR_STREAM(prefix << "failed to call service joint_pitch/torque_enable");
      }

      limb_.servo_switch_t_ = t;
      limb_.phase_ = limb_.PHASE3;
      ROS_INFO_STREAM(prefix << " shift to PHASE3 for final fit");

      break;
    }
  case limb_.PHASE3:
    {
      std::string prefix("[Spider][Belly Crawl][Limb][Phase2]");

      if (t - limb_.servo_switch_t_ < limb_.servo_switch_duration_) {
        break;
      }


      ROS_INFO_STREAM(prefix << " baselink has been fitted");

      // servo off the joint pitch
      ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
      std_srvs::SetBool srv;
      srv.request.data = true;

      if (client.call(srv))
        ROS_INFO_STREAM(prefix << " enable the pitch joint torque");
      else
        ROS_ERROR_STREAM(prefix << "Failed to call service joint_pitch/torque_enable");

      limb_.phase_ = limb_.PHASE0;

      // reset the target leg ends to further update the target joint angles
      if (cycle_reset_leg_end_) {
        reset_leg_ends_flag_ = true;
      }

      // reset the target baselink to further update the target joint angles
      if (cycle_reset_baselink_) {
        reset_baselink_flag_ = true;
      }

      // workaround: condition for whole state machine
      phase_ = PHASE2; // move to belly move

      break;
    }
  default:
    {
      break;
    }
  }

  limb_.prev_t_ = t;
}

void BellyCrawl::bellySubStateMachine()
{
  tf::Vector3 curr_pos = getCurrentBaselinkPos();
  tf::Vector3 target_pos = getTargetBaselinkPos();
  double t = ros::Time::now().toSec();

  if (t - belly_.prev_t_ < belly_.loop_duration_) {
    return;
  }

  switch(belly_.phase_) {

  case belly_.PHASE0:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase0]");

      walk_controller_->setFloatingBellyMode(true);

      target_pos += tf::Vector3(0, 0, belly_.raise_height_); // TODO: only horizontal ground
      setTargetBaselinkPos(target_pos);
      init_pos_ = curr_pos;

      // shift to belly_.PHASE1
      belly_.phase_ ++;
      ROS_INFO_STREAM(prefix << " shift to PHASE1 for raise");

      break;
    }
  case belly_.PHASE1:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase1]");

      double diff = curr_pos.z() - init_pos_.z();

      ROS_INFO_STREAM_THROTTLE(0.1, prefix << " curr z: " << curr_pos.z() << "; target z: " << target_pos_.z() << "; init z: " << init_pos_.z());

      if (diff > belly_.raise_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has raised to a enough height, move horizontally");

        target_pos.setX(target_pos_.x());
        target_pos.setY(target_pos_.y());
        setTargetBaselinkPos(target_pos);

        // shift to belly_.PHASE2
        belly_.phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE2 for horizontal move");
      }

      break;

    }
  case belly_.PHASE2:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase2]");

      tf::Vector3 diff_vec = curr_pos - target_pos;
      diff_vec.setZ(0);
      double diff = diff_vec.length();

      ROS_INFO_STREAM_THROTTLE(0.1, prefix << " curr x: " << curr_pos.x() << ", y: " << curr_pos.y() \
                               << "; target x: " << target_pos.x() << ", y: " << target_pos.y());

      if (fabs(diff) < belly_.move_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has moved to the right place");

        setTargetBaselinkPos(target_pos_); // set real target pos

        belly_.phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE3 for descend");
      }

      break;
    }
  case belly_.PHASE3:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase3]");

      double diff = curr_pos.z() - prev_pos_.z();

      if (fabs(diff) < belly_.descend_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has descend to a static point");

        // servo off the joint pitch
        ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
        std_srvs::SetBool srv;
        srv.request.data = false;

        if (client.call(srv))
          ROS_INFO_STREAM(prefix << " disable the pitch joint torque");
        else
          ROS_ERROR_STREAM(prefix << "Failed to call service joint_pitch/torque_enable");

        belly_.servo_switch_t_ = t;
        belly_.phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE4 for final fit");
      }

      break;
    }
  case belly_.PHASE4:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase4]");

      if (t - belly_.servo_switch_t_ < belly_.servo_switch_duration_) {
        break;
      }

      double diff = curr_pos.z() - prev_pos_.z();

      if (fabs(diff) < belly_.descend_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has been fitted");

        // servo off the joint pitch
        ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
        std_srvs::SetBool srv;
        srv.request.data = true;

        if (client.call(srv))
          ROS_INFO_STREAM(prefix << " enable the pitch joint torque");
        else
          ROS_ERROR_STREAM(prefix << "Failed to call service joint_pitch/torque_enable");

        ROS_INFO_STREAM(prefix << " finish moving");
      } else {
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " baselink cannot be fitted. curr z: "
                                 << curr_pos.z() << ", z: " << target_pos.z());
        break;
      }

      belly_.phase_ = belly_.PHASE0;

      // set the static floating belly mode
      walk_controller_->setFloatingBellyMode(false);


      // reset the target leg ends to further update the target joint angles
      if (cycle_reset_leg_end_) {
        reset_leg_ends_flag_ = true;
      }

      // reset the target baselink to further update the target joint angles
      if (cycle_reset_baselink_) {
        reset_baselink_flag_ = true;
      }

      // workaround: condition for whole state machine
      phase_ = PHASE0; // move to initialize phase
      if (target_pos_ == final_target_pos_) {
        ROS_INFO_STREAM(prefix << " complete the iterative move");
        reset();
      }

      break;
    }
  default:
    break;
  }

  prev_pos_ = curr_pos;
  belly_.prev_t_ = t;
}

void BellyCrawl::setPose(tf::Transform pose)
{
  tf::Vector3 pos = pose.getOrigin();
  double yaw = tf::getYaw(pose.getRotation());

  setPose(pos, yaw);
}

void BellyCrawl::setPose(tf::Vector3 pos, double yaw)
{
  reset();

  final_target_pos_ = pos;
  final_target_yaw_ = yaw;

  move_flag_ = true;
}

void BellyCrawl::setDeltaPos(tf::Vector3 delta_pos)
{
  delta_pos.setZ(0);
  tf::Vector3 target_pos = getTargetBaselinkPos() + delta_pos;
  setPose(target_pos);
}

void BellyCrawl::reset()
{
  move_flag_ = false;
  phase_ = PHASE0;
  limb_.phase_ = limb_.PHASE0;
  belly_.phase_ = belly_.PHASE0;
}

void BellyCrawl::iterativeUpdateTargetPos()
{
  tf::Vector3 target_pos = getTargetBaselinkPos();

  double delta = (final_target_pos_ - target_pos).length();
  if (delta <= stride_) {
    target_pos_ = final_target_pos_;
  } else {
    target_pos_ = target_pos + (final_target_pos_ - target_pos).normalize() * stride_;
  }
}

void BellyCrawl::moveCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
{
  tf::Vector3 delta_pos;
  tf::vector3MsgToTF(msg->vector, delta_pos);
  setDeltaPos(delta_pos);
}

void BellyCrawl::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
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


  if(joy_cmd.buttons[PS3_BUTTON_REAR_LEFT_2] == 1) {
    /* belly crawl test */

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {
      if (!move_flag_) {
        tf::Vector3 delta_pos(stride_, 0, 0);
        setDeltaPos(delta_pos);
        ROS_INFO("[Spider][Walk][Navigation][Joy] start belly crawl");
      }
      return;
    }

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {

      // abort process here

      if (move_flag_) {

        // PHASE1: limb motion
        if (phase_ == PHASE1) {
          ROS_WARN_STREAM("[Spider][Walk][Navigation][Joy] instantly lower the raised leg");
          lowerLeg();
        }

        // PHASE2: belly motion
        if (phase_ == PHASE2) {
          ROS_WARN_STREAM("[Spider][Walk][Navigation][Joy] instantly lower the baselink");

          /* servo off */
          ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
          std_srvs::SetBool srv;
          srv.request.data = false;

          if (client.call(srv))
            ROS_INFO("[Spider][Joy] disable the pitch joint torque");
          else
            ROS_ERROR("Failed to call service joint_pitch/torque_enable");
        }

        reset();
        ROS_INFO("[Spider][Walk][Navigation][Joy] abort belly crawl");

      }
      return;
    }
  }

  Base::joyStickControl(joy_msg);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Spider::Terrestrial::BellyCrawl, aerial_robot_navigation::BaseNavigator);
