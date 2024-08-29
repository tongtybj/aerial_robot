#include <spidar/navigation/flight_navigation.h>

using namespace aerial_robot_navigation;
using namespace aerial_robot_navigation::Spider;

FlightNavigator::FlightNavigator():
  DragonNavigator()
{
}


void FlightNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                 boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                 boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                 double loop_du)
{
  /* initialize the flight control */
  DragonNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  joint_torque_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_torque_ctrl", 1);

  ros::NodeHandle nav_nh(nh_, "navigation");
  getParam<double>(nav_nh, "joint_servo_torque_limit", joint_servo_torque_limit_, 8.0);

  ros::NodeHandle land_nh(nh_, "navigation/land");
  getParam<double>(land_nh, "inside_land_pitch_angle", inside_land_pitch_angle_, -0.1);
  getParam<double>(land_nh, "outside_land_pitch_angle", outside_land_pitch_angle_, 0.2);

}

void FlightNavigator::update()
{
  DragonNavigator::update();

  if (initialize_joint_servo_torque_)
    {
      if (getNaviState() == START_STATE)
        {
          int rotor_num_ = robot_model_->getRotorNum();

          // send the limit of servo torque
          sensor_msgs::JointState torque_msg;
          torque_msg.header.stamp = ros::Time::now();
          for (int i = 0; i < rotor_num_; i++)
            {
              std::string id = std::to_string(i+1);
              torque_msg.name.push_back(std::string("joint") + id + std::string("_yaw"));
              torque_msg.name.push_back(std::string("joint") + id + std::string("_pitch"));
              torque_msg.effort.push_back(joint_servo_torque_limit_);
              torque_msg.effort.push_back(joint_servo_torque_limit_);
            }
          joint_torque_pub_.publish(torque_msg);

          initialize_joint_servo_torque_ = false;
        }
    }
}

void FlightNavigator::landingProcess()
{
  if(getForceLandingFlag() || getNaviState() == LAND_STATE)
    {
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
          final_target_baselink_rot_.setValue(0, 0, 0);

          double curr_roll = estimator_->getState(State::ROLL_BASE, estimate_mode_)[0];
          double curr_pitch = estimator_->getState(State::PITCH_BASE, estimate_mode_)[0];

          if(fabs(fabs(curr_pitch) - M_PI/2) < 0.05 && fabs(curr_roll) > M_PI/2) // singularity of XYZ Euler
            curr_roll = 0;

          /* force set the current deisre tilt to current estimated tilt */
          curr_target_baselink_rot_.setValue(curr_roll, curr_pitch, 0);
        }

      level_flag_ = true;

      if(getNaviState() == LAND_STATE && !landing_flag_)
        {
          landing_flag_ = true;
          setTeleopFlag(false);
          setTargetPosZ(estimator_->getState(State::Z_COG, estimate_mode_)[0]);
          setTargetVelZ(0);
          land_height_ = 0; // reset the land height, since it is updated in the first land_state which is forced to change to hover state to level the orientation. Thus, it is possible to have the same land height just after switching back to land state and thus stop in midair
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
