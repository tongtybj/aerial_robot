#include <spidar/navigation/baselink_motion.h>

namespace aerial_robot_navigation::Spider
{
  BaselinkMotion::BaselinkMotion(ros::NodeHandle nh, WalkNavigator* navigator):
    nh_(nh), navigator_(navigator),
    move_flag_(false), preempt_flag_(false),
    prev_t_(0)
  {
    ros::NodeHandle baselink_nh(nh_, "navigation/baselink");
    baselink_nh.param("duration", duration_, 0.1);
    baselink_nh.param("raise_height", raise_height_, 0.1);
    baselink_nh.param("raise_thresh", raise_thresh_, 0.05);
    baselink_nh.param("move_thresh", move_thresh_, 0.05);
    baselink_nh.param("descend_thresh", descend_thresh_, 0.02);

    phase_ = PHASE0;
  }

  bool BaselinkMotion::getMoveFlag() const
  {
    return move_flag_;
  }

  void BaselinkMotion::set(tf::Transform pose)
  {
    tf::Vector3 pos = pose.getOrigin();
    double yaw = tf::getYaw(pose.getRotation());

    set(pos, yaw);
  }

  void BaselinkMotion::set(tf::Vector3 pos, double yaw)
  {
    target_pos_ = pos;
    target_yaw_ = yaw;

    move_flag_ = true; // TODO: check the validity of the target pos & yaw
    reset();
  }

  void BaselinkMotion::stop()
  {
    preempt_flag_ = true;
  }

  void BaselinkMotion::reset()
  {
    phase_ = PHASE0;
    preempt_flag_ = false;
  }

  void BaselinkMotion::update()
  {
    tf::Vector3 curr_pos = navigator_->getCurrentBaselinkPos();
    tf::Vector3 target_pos = navigator_->getTargetBaselinkPos();
    double t = ros::Time::now().toSec();

    if (t - prev_t_ < duration_) {
      return;
    }

    switch(phase_) {

    case PHASE0:
      {
        if (!move_flag_) {
          break;
        }

        std::string prefix("[Spider][Baselink][Phase0]");

        target_pos += tf::Vector3(0, 0, raise_height_); // TODO: only horizontal ground
        navigator_->setTargetBaselinkPos(target_pos);
        init_pos_ = navigator_->getCurrentBaselinkPos();

        // shift to PHASE1
        phase_ ++;
        ROS_INFO_STREAM(prefix << " shift to PHASE1 for raise");

        break;
      }
    case PHASE1:
      {
        std::string prefix("[Spider][Baselink][Phase1]");

        double diff = curr_pos.z() - init_pos_.z();

        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " curr z: " << curr_pos.z() << "; target z: " << init_pos_.z());

        if (diff > raise_thresh_) {
          ROS_INFO_STREAM(prefix << " baselink has raised to a enough height, move horizontally");

          target_pos.setX(target_pos_.x());
          target_pos.setY(target_pos_.y());
          navigator_->setTargetBaselinkPos(target_pos);

          // shift to PHASE2
          phase_ ++;
          ROS_INFO_STREAM(prefix << " shift to PHASE2 for horizontal move");
        }

        break;

      }
    case PHASE2:
      {
        std::string prefix("[Spider][Baselink][Phase2]");

        tf::Vector3 diff_vec = curr_pos - target_pos;
        diff_vec.setZ(0);
        double diff = diff_vec.length();

        ROS_INFO_STREAM_THROTTLE(0.1, prefix << " curr x: " << curr_pos.x() << ", y: " << curr_pos.y() \
                                 << "; target x: " << target_pos.x() << ", y: " << target_pos.y());

        if (fabs(diff) < move_thresh_) {
          ROS_INFO_STREAM(prefix << " baselink has moved to the right place");

          navigator_->setTargetBaselinkPos(target_pos_); // set real target pos

          phase_ ++;
          ROS_INFO_STREAM(prefix << " shift to PHASE3 for descend");
        }

        break;
      }
    case PHASE3:
      {
        std::string prefix("[Spider][Baselink][Phase3]");

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

          phase_ ++;
          ROS_INFO_STREAM(prefix << " shift to PHASE4 for final fit");
        }

        break;
      }
    case PHASE4:
      {
        std::string prefix("[Spider][Baselink][Phase4]");

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
          ROS_INFO_STREAM(prefix << " shift to PHASE0");
        }

        break;
      }
    default:
      break;
    }

    prev_pos_ = curr_pos;
    prev_t_ = t;
  }


};
