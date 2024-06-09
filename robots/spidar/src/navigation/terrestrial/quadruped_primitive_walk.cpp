#include <spidar/navigation/terrestrial/quadruped_primitive_walk.h>
#include <spidar/control/walk_control.h> // for complate the class definition of class aerial_robot_control::Spider::WalkController

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Spider::Terrestrial;



QuadrupedPrimitiveWalk::QuadrupedPrimitiveWalk():
  move_flag_(false),
  leg_id_(-1),
  phase_(PHASE0),
  move_baselink_(false)
{
}

void QuadrupedPrimitiveWalk::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                        double loop_du)
{
  /* initialize with the super class */
  Base::initialize(nh, nhp, robot_model, estimator, loop_du);

  move_sub_ = nh_.subscribe("walk/enable", 1, &QuadrupedPrimitiveWalk::moveCallback, this);

  raise_angle_orig_ = raise_angle_;
}

void QuadrupedPrimitiveWalk::rosParamInit()
{
  Base::rosParamInit();

  ros::NodeHandle nh_walk(nh_, "navigation/walk");
  ros::NodeHandle nh_quadruped_walk(nh_walk, "primitive_quadruped");

  getParam<int>(nh_quadruped_walk, "total_cycle", total_cycle_, 1);
  getParam<double>(nh_quadruped_walk, "stride", stride_, 0.2);
  getParam<double>(nh_quadruped_walk, "move_leg_joint_err_thresh", move_leg_joint_err_thresh_, 0.05);
  getParam<double>(nh_quadruped_walk, "baselink_converge_thresh", baselink_converge_thresh_, 0.05);
  getParam<double>(nh_quadruped_walk, "front_leg_raise_ratio", front_leg_raise_ratio_, 1.0);
  getParam<double>(nh_quadruped_walk, "converge_du", converge_du_, 0.5);
  getParam<bool>(nh_quadruped_walk, "reset_leg_end", cycle_reset_leg_end_, true);
  getParam<bool>(nh_quadruped_walk, "reset_baselink", cycle_reset_baselink_, false);
  getParam<bool>(nh_quadruped_walk, "debug", debug_, false);
  getParam<int>(nh_quadruped_walk, "debug_legs", debug_legs_, 0);
}

void QuadrupedPrimitiveWalk::update()
{
  Base::update();

  if (getNaviState() == aerial_robot_navigation::ARM_ON_STATE) {
    walk_controller_->setFloatingBellyMode(true);
  }


  stateMachine();
}

void QuadrupedPrimitiveWalk::stateMachine()
{
  /*
    simple walking pattern:
    - Straight line walk along x axis.
    - Every leg move along x axis with `walk_stride` at each step.
    - Leg will raise (raise inner joint pitch angle), and also move joint yaw angle at the same time.
    - Center link also move with `walk_stride`/`leg_num` at each step.
    - A circle means all leg finish move one time
   */

  if (!move_flag_) { return; }

  switch(phase_) {

  case PHASE0:
    {
      // all legs contact to the ground
      std::string prefix("[Spider][Primitive Quadruped Walk][Phase0]");

      // start raise leg mode for this leg
      raiseLeg(leg_id_);
      if (leg_id_ == 0 || leg_id_ == 3) {
        // workaround: set larget raise angle for front legs
        raise_angle_ = raise_angle_orig_ * front_leg_raise_ratio_;
      }

      // update the foot postion, and thus the joint angles
      target_leg_ends_.at(leg_id_).p += KDL::Vector(stride_, 0, 0); // only along x axis


      // reset the timestamp to check joint convergence
      converge_timestamp_ = ros::Time::now().toSec();

      // shift to PHASE1
      ROS_INFO_STREAM(prefix << " shift from PHASE0 to PHASE1 for leg" << leg_id_ + 1);
      phase_ = PHASE1;

      break;
    }
  case PHASE1:
    {
      // free leg is raising
      std::string prefix("[Spider][Primitive Quadruped Walk][Phase1]");

      // check the pitch joint of free leg to lower leg
      int j = 4 * leg_id_;
      if (!raise_converge_) {
        // NOTE: no need to consider the converge of this joint

        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " the angle error of joint" << j/2 + 1 << "_pitch does not converge");
        break;
      }

      // check the inside yaw joint of free leg to lower leg
      double target_angle = target_joint_state_.position.at(j);
      double current_angle = getCurrentJointAngles().at(j);
      double err = target_angle - current_angle;
      if (fabs(err) > move_leg_joint_err_thresh_ ) {
        // joint angle error too big

        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " the angle error of joint" << j/2 + 1 << "_yaw does not converge, " << err << "(" << move_leg_joint_err_thresh_ << ")");

        // reset the timestamp to check joint convergence
        converge_timestamp_ = ros::Time::now().toSec();

        break;
      }

      // check the outside pitch joint of free leg to lower leg
      target_angle = target_joint_state_.position.at(j + 3);
      current_angle = getCurrentJointAngles().at(j + 3);
      err = target_angle - current_angle;
      //ROS_INFO_STREAM(prefix << " joint" << j/2 + 2 << "_pitch, target angle: " << target_angle << "; current angle: " << current_angle);
      if (fabs(err) > move_leg_joint_err_thresh_ ) {
        // joint angle error too big

        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " the angle error of joint" << j/2 + 2 << "_pitch does not converge, " << err << "(" << move_leg_joint_err_thresh_ << ")");

        // reset the timestamp to check joint convergence
        converge_timestamp_ = ros::Time::now().toSec();

        break;
      }

      // no check of the converge duration
      // double t = ros::Time::now().toSec() - converge_timestamp_;
      // if (t < walk_pattern_converge_du_) {
      //   // no reach enough convergence time
      //   ROS_INFO_STREAM_THROTTLE(0.1, prefix << " not reach enough convergence time for joint, start from " << converge_timestamp_  << ", last " << t);
      //   break;
      // }

      // reach the target joint yaw to lower leg
      raise_angle_ = raise_angle_orig_;
      lowerLeg();

      // reset the timestamp to check joint convergence
      converge_timestamp_ = ros::Time::now().toSec();

      // shift to PHASE2
      ROS_INFO_STREAM(prefix << " shift from PHASE1 to PHASE2 for leg" << leg_id_ + 1);
      phase_ = PHASE2;

      break;
    }
  case PHASE2:
    {
      // free leg is lowering
      std::string prefix("[Spider][Primitive Quadruped Walk][Phase2]");

      if (lower_leg_flag_) {
        // still in lowering phase, skip
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " still in lowering phase, skip");

        break;
      }

      if (walk_controller_->getContactTransition()) {
        // still in contact transition, skip
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " finish lowering, but still in ground contact transition, skip");

        break;
      }

      if (debug_) {
        if (debug_legs_ == 0) {
          // only test single leg raise & lower

          // shift to PHASE0
          ROS_INFO_STREAM(prefix << " debug mode, finish walk");
          move_flag_ = false;
          break;
        }
      }

      // move center link
      move_baselink_ = false;
      if (leg_id_ == 3) {
        // WIP: the leg selection.
        //      move baselink only after leg4 (all front legs are updated)
        // WIP: forward move
        target_baselink_pos_ += tf::Vector3(stride_, 0, 0);
        move_baselink_ = true;
      }

      // shift to PHASE3
      ROS_INFO_STREAM(prefix << " shift from PHASE2 to PHASE3 for leg" << leg_id_ + 1);
      phase_ = PHASE3;

      break;
    }
  case PHASE3:
    {
      // baselink is move
      std::string prefix("[Spider][Primitive Quadruped Walk][Phase3]");

      if (move_baselink_) {

        // move center link
        tf::Vector3 baselink_pos = estimator_->getPos(Frame::BASELINK, estimate_mode_);
        double err = target_baselink_pos_.x() - baselink_pos.x();

        if (fabs(err) > baselink_converge_thresh_) {
          // no reach enough convergence position
          ROS_INFO_STREAM_THROTTLE(0.1, prefix << " no reach enough convergence position, target x: "
                                   << target_baselink_pos_.x() << ", current x: " << baselink_pos.x()
                                   << ", converge thresh: " << baselink_converge_thresh_);

          // reset the timestamp to check baselink convergence
          converge_timestamp_ = ros::Time::now().toSec();

          break;
        }

        double t = ros::Time::now().toSec() - converge_timestamp_;
        if (t < converge_du_) {
          // no reach enough convergence time
          ROS_INFO_STREAM_THROTTLE(0.1, prefix << " not reach enough convergence time for baselink move, start from" << converge_timestamp_  << ", last " << t);
          break;
        }
      }

      ROS_INFO_STREAM(prefix << " finish move of leg" << leg_id_ + 1 << " in walk cycle of " << cycle_cnt_);

      // reset the target leg ends to further update the target joint angles
      if (cycle_reset_leg_end_) {
        reset_leg_ends_flag_ = true;
      }

      // reset the target baselink to further update the target joint angles
      if (cycle_reset_baselink_) {
        reset_baselink_flag_ = true;
      }

      // instant update to the target joint angle
      if (kinematic_simulation_) {
        spidar_robot_model_->updateRobotModel(target_joint_state_);

        for(int i = 0; i < sim_joint_state_.position.size(); i++) {
          std::string name = sim_joint_state_.name.at(i);
          auto res = std::find(target_joint_state_.name.begin(), target_joint_state_.name.end(), name);
          int j = std::distance(target_joint_state_.name.begin(), res);
          sim_joint_state_.position.at(i) = target_joint_state_.position.at(j);
        }
      }

      // move next leg with following rule:
      // front-left (0) -> front-right (3) -> back-right (2) -> back-left (1)

      if (debug_) {
        if ((debug_legs_ == 1 && leg_id_ == 0) ||
            (debug_legs_ == 2 && leg_id_ == 3) ||
            (debug_legs_ == 3 && leg_id_ == 2) ||
            (debug_legs_ == 4 && leg_id_ == 1)) {
          ROS_INFO_STREAM(prefix << " debug mode, finish walk");
          move_flag_ = false;
          break;
        }
      }

      leg_id_ --;
      if (leg_id_ < 0) leg_id_ += 4;

      // check whether finish one cycle
      if (leg_id_ == 0) {
        cycle_cnt_ ++;
      }
      if (cycle_cnt_ == total_cycle_) {
        ROS_INFO_STREAM(prefix << " finish walk with totla cycle of " << total_cycle_);
        move_flag_ = false;
      }

      phase_ = PHASE0; // back to Phase0 to move next leg.

      break;
    }
  default:
    break;
  }
}

void QuadrupedPrimitiveWalk::resetStateMachine()
{
  leg_id_ = 0;
  cycle_cnt_ = 0;
  phase_ = PHASE0;
}

void QuadrupedPrimitiveWalk::moveCallback(const std_msgs::BoolConstPtr& msg)
{
  resetStateMachine();
  move_flag_ = msg->data;

  if (!move_flag_) {
    // abort process here

    if (raise_leg_flag_) {
      ROS_WARN_STREAM("[Spider][Walk][Navigation] instantly lower the raised leg" << free_leg_id_ + 1 << " during the walk pattern");
      lowerLeg();
    }
  }
}

void QuadrupedPrimitiveWalk::joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
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
    /* walk test */

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_UP] == 1) {
      if (!move_flag_) {
        move_flag_ = true;
        resetStateMachine();
        ROS_INFO("[Spider][Walk][Navigation][Joy] start quadruped primitive walk");
      }
      return;
    }

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {

      // abort process here

      if (move_flag_) {
        move_flag_ = false;
        resetStateMachine();
        ROS_INFO("[Spider][Walk][Navigation][Joy] abort quadruped primitive walk");

        if (raise_leg_flag_) {
          ROS_WARN_STREAM("[Spider][Walk][Navigation][Joy] instantly lower the raised leg"
                          << free_leg_id_ + 1 << " during walking");
          lowerLeg();
        }
      }
      return;
    }
  }

  Base::joyStickControl(joy_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Spider::Terrestrial::QuadrupedPrimitiveWalk, aerial_robot_navigation::BaseNavigator);
