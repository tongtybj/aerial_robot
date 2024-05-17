#include <spidar/navigation/terrestrial/belly_crawl.h>
#include <spidar/control/walk_control.h> // for complate the class definition of class aerial_robot_control::Spider::WalkController

using namespace aerial_robot_model;
using namespace aerial_robot_navigation::Spider::Terrestrial;

BellyCrawl::BellyCrawl():
  move_flag_(false),
  phase_(PHASE0)
{
}


void BellyCrawl::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize with the super class */
  Base::initialize(nh, nhp, robot_model, estimator);

  move_sub_ = nh_.subscribe("walk/belly_crawl/traget/delta_pos", 1, &BellyCrawl::moveCallback, this);

}

void BellyCrawl::rosParamInit()
{
  ros::NodeHandle nh_walk(nh_, "navigation/walk");
  ros::NodeHandle nh_belly_crawl(nh_walk, "belly_crawl");
  ros::NodeHandle nh_belly_crawl_baselink(nh_belly_crawl, "baselink");

  nh_belly_crawl_baselink.param("loop_duration", loop_duration_, 0.1);
  nh_belly_crawl_baselink.param("servo_switch_duration", servo_switch_duration_, 0.5);
  nh_belly_crawl_baselink.param("raise_height", raise_height_, 0.1);
  nh_belly_crawl_baselink.param("raise_thresh", raise_thresh_, 0.05);
  nh_belly_crawl_baselink.param("move_thresh", move_thresh_, 0.05);
  nh_belly_crawl_baselink.param("descend_thresh", descend_thresh_, 0.02);
}


void BellyCrawl::update()
{
  Base::update();

  stateMachine();
}

void BellyCrawl::stateMachine()
{
  tf::Vector3 curr_pos = getCurrentBaselinkPos();
  tf::Vector3 target_pos = getTargetBaselinkPos();
  double t = ros::Time::now().toSec();

  if (t - prev_t_ < loop_duration_) {
    return;
  }

  switch(phase_) {

  case PHASE0:
    {
      if (!move_flag_) {
        break;
      }

      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase0]");

      target_pos += tf::Vector3(0, 0, raise_height_); // TODO: only horizontal ground
      setTargetBaselinkPos(target_pos);
      init_pos_ = curr_pos;

      // shift to PHASE1
      phase_ ++;
      ROS_INFO_STREAM(prefix << " shift to PHASE1 for raise");

      break;
    }
  case PHASE1:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase1]");

      double diff = curr_pos.z() - init_pos_.z();

      ROS_INFO_STREAM_THROTTLE(0.1, prefix << " curr z: " << curr_pos.z() << "; target z: " << init_pos_.z());

      if (diff > raise_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has raised to a enough height, move horizontally");

        target_pos.setX(target_pos_.x());
        target_pos.setY(target_pos_.y());
        setTargetBaselinkPos(target_pos);

        // shift to PHASE2
        phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE2 for horizontal move");
      }

      break;

    }
  case PHASE2:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase2]");

      tf::Vector3 diff_vec = curr_pos - target_pos;
      diff_vec.setZ(0);
      double diff = diff_vec.length();

      ROS_INFO_STREAM_THROTTLE(0.1, prefix << " curr x: " << curr_pos.x() << ", y: " << curr_pos.y() \
                               << "; target x: " << target_pos.x() << ", y: " << target_pos.y());

      if (fabs(diff) < move_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has moved to the right place");

        setTargetBaselinkPos(target_pos_); // set real target pos

        phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE3 for descend");
      }

      break;
    }
  case PHASE3:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase3]");

      double diff = curr_pos.z() - prev_pos_.z();

      if (fabs(diff) < descend_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has descend to a static point");

        // servo off the joint pitch
        ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
        std_srvs::SetBool srv;
        srv.request.data = false;

        if (client.call(srv))
          ROS_INFO_STREAM(prefix << " disable the pitch joint torque");
        else
          ROS_ERROR_STREAM(prefix << "Failed to call service joint_pitch/torque_enable");

        servo_switch_t_ = t;
        phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE4 for final fit");
      }

      break;
    }
  case PHASE4:
    {
      std::string prefix("[Spidar][Belly Crawl][Baselink][Phase4]");

      if (t - servo_switch_t_ < servo_switch_duration_) {
        break;
      }

      double diff = curr_pos.z() - prev_pos_.z();

      if (fabs(diff) < descend_thresh_) {
        ROS_INFO_STREAM(prefix << " baselink has been fitted");

        // servo off the joint pitch
        ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("joint_pitch/torque_enable");
        std_srvs::SetBool srv;
        srv.request.data = true;

        if (client.call(srv))
          ROS_INFO_STREAM(prefix << " enable the pitch joint torque");
        else
          ROS_ERROR_STREAM(prefix << "Failed to call service joint_pitch/torque_enable");

        phase_ = PHASE0;
        move_flag_ = false;
        ROS_INFO_STREAM(prefix << " finish moving");
      } else {
        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " baselink cannot be fitted. curr z: "
                                 << curr_pos.z() << ", z: " << target_pos.z());
      }

      break;
    }
  default:
    break;
  }

  prev_pos_ = curr_pos;
  prev_t_ = t;
}

void BellyCrawl::setPose(tf::Transform pose)
{
  tf::Vector3 pos = pose.getOrigin();
  double yaw = tf::getYaw(pose.getRotation());

  setPose(pos, yaw);
}

void BellyCrawl::setPose(tf::Vector3 pos, double yaw)
{
  target_pos_ = pos;
  target_yaw_ = yaw;

  move_flag_ = true; // TODO: check the validity of the target pos & yaw
  reset();
}

void BellyCrawl::setDeltaPos(tf::Vector3 delta_pos)
{
  delta_pos.setZ(0);
  tf::Vector3 target_pos = target_baselink_pos_ + delta_pos;
  setPose(target_pos);
}

void BellyCrawl::reset()
{
  phase_ = PHASE0;
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
        tf::Vector3 delta_pos(0.2, 0, 0);
        setDeltaPos(delta_pos);
        ROS_INFO("[Spider][Walk][Navigation][Joy] start belly crawl");
      }
      return;
    }

    if(joy_cmd.buttons[PS3_BUTTON_CROSS_DOWN] == 1) {

      // abort process here

      if (move_flag_) {
        move_flag_ = false;
        reset();
        ROS_INFO("[Spider][Walk][Navigation][Joy] abort belly crawl");

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
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Spider::Terrestrial::BellyCrawl, aerial_robot_navigation::BaseNavigator);
