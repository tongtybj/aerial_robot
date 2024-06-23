#include <spidar/navigation/flight_navigation.h>

using namespace aerial_robot_navigation;
using namespace aerial_robot_navigation::Spider;

FlightNavigator::FlightNavigator():
  DragonNavigator()
{
}


void FlightNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator)
{
  /* initialize the flight control */
  DragonNavigator::initialize(nh, nhp, robot_model, estimator);

  ros::NodeHandle land_nh(nh_, "navigation/land");
  getParam<double>(land_nh, "inside_land_pitch_angle", inside_land_pitch_angle_, -0.1);
  getParam<double>(land_nh, "outside_land_pitch_angle", outside_land_pitch_angle_, 0.2);
}

void FlightNavigator::landingProcess()
{

  if(getForceLandingFlag() || getNaviState() == LAND_STATE)
    {
      target_omega_.setValue(0,0,0); // for sure to  target angular velocity is zero

      if(!level_flag_)
        {
          const auto joint_state = robot_model_->kdlJointToMsg(robot_model_->getJointPositions());
          sensor_msgs::JointState joint_control_msg;
          int joint_num = joint_state.position.size();
          for(int i = 0; i < joint_num; i++)
            {
              std::string name = joint_state.name.at(i);
              if(name.find("joint") != std::string::npos)
                {
                  double target_cmd;
                  if(name.find("pitch") != std::string::npos)
                    {
                      int j = atoi(name.substr(5,1).c_str()) - 1; // start from 0

                      if (j % 2 == 0)
                        {
                          target_cmd = inside_land_pitch_angle_;
                        }
                      else
                        {
                          target_cmd = outside_land_pitch_angle_;
                        }
                    }
                  else
                    {
                      target_cmd = joint_state.position[i];
                    }

                  joint_control_msg.name.push_back(name);
                  joint_control_msg.position.push_back(target_cmd);

                }
            }


          joint_control_pub_.publish(joint_control_msg);

          final_target_baselink_rot_.setRPY(0, 0, 0);

          tf::Vector3 base_euler = estimator_->getEuler(Frame::BASELINK, estimate_mode_);

          if(fabs(fabs(base_euler.y()) - M_PI/2) > 0.2)
            {
              /* force set the current deisre tilt to current estimated tilt */
              curr_target_baselink_rot_.setRPY(base_euler.x(), base_euler.y(), 0); // case1
            }
          else
            { // singularity of XYZ Euler

              double r,p,y;
              tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r,p,y);
              if(fabs(fabs(p) - M_PI/2) > 0.2)
                curr_target_baselink_rot_.setRPY(0, base_euler.y(), 0); // case2
            }
        }

      level_flag_ = true;

      if(getNaviState() == LAND_STATE && !landing_flag_)
        {
          landing_flag_ = true;
          setTeleopFlag(false);
          setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
          setNaviState(HOVER_STATE);
        }
    }

  /* back to landing process */
  if(landing_flag_)
    {
      const auto joint_state = robot_model_->kdlJointToMsg(robot_model_->getJointPositions());
      bool already_level = true;
      int joint_num = joint_state.position.size();
      for(int i = 0; i < joint_num; i++)
        {
          std::string name = joint_state.name.at(i);
          if(joint_state.name[i].find("joint") != std::string::npos
             && joint_state.name[i].find("pitch") != std::string::npos)
            {
              double target_cmd = 0;
              int j = atoi(name.substr(5,1).c_str()) - 1; // start from 0

              if (j % 2 == 0)
                {
                  target_cmd = inside_land_pitch_angle_;
                }
              else
                {
                  target_cmd = outside_land_pitch_angle_;
                }


              if(fabs(joint_state.position[i] - target_cmd) > 0.085) already_level = false;
            }
        }

      if(curr_target_baselink_rot_.length()) already_level = false;

      if(already_level && getNaviState() == HOVER_STATE)
        {
          ROS_WARN("gimbal control: back to land state");
          setNaviState(LAND_STATE);
          setTeleopFlag(true);
        }
    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::Spider::FlightNavigator, aerial_robot_navigation::BaseNavigator);
