#include <dragon/gimbal_pitch_only_control.h>

using namespace aerial_robot_model;

namespace control_plugin
{
  DragonGimbalPitchOnly::DragonGimbalPitchOnly():
    FlatnessPid(), gimbal_control_stamp_(0)
  {
    need_yaw_d_control_ = true;
  }

  void DragonGimbalPitchOnly::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                BasicEstimator* estimator, Navigator* navigator,
                                double ctrl_loop_rate)
  {
    /* initialize the flight control */
    FlatnessPid::initialize(nh, nhp, estimator, navigator, ctrl_loop_rate);

    /* initialize the multilink kinematics */
    kinematics_ = std::make_unique<DragonRobotModel>(true);

    /* initialize the gimbal target angles */
    target_gimbal_angles_.resize(kinematics_->getRotorNum() * 2, 0);

    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("/gimbals_ctrl", 1);
    gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("gimbals_target_force", 1);
    joint_state_sub_ = nh_.subscribe("/joint_states", 1, &DragonGimbalPitchOnly::jointStateCallback, this);

    //dynamic reconfigure server
    yaw_pid_server_ = new dynamic_reconfigure::Server<aerial_robot_base::XYPidControlConfig>(ros::NodeHandle(nhp_, "yaw"));
    dynamic_reconf_func_yaw_pid_ = boost::bind(&DragonGimbalPitchOnly::cfgYawPidCallback, this, _1, _2);
    yaw_pid_server_->setCallback(dynamic_reconf_func_yaw_pid_);
  }

  void DragonGimbalPitchOnly::fourAxisGainCallback(const aerial_robot_msgs::FourAxisGainConstPtr & msg)
  {
    /* update the motor number */
    if(motor_num_ == 0)
      {
        motor_num_ = msg->motor_num;

        alt_i_term_.resize(motor_num_);
        alt_gains_.resize(motor_num_);

        target_throttle_.resize(motor_num_);

        ROS_WARN("gimbal flight control: update the motor number: %d", motor_num_);
      }

    for(int i = 0; i < msg->motor_num; i++)
      alt_gains_[i].setValue(msg->pos_p_gain_alt[i], msg->pos_i_gain_alt[i], msg->pos_d_gain_alt[i]);
  }

  bool DragonGimbalPitchOnly::update()
  {
    if(!ControlBase::update()) return false;

    stateError();

    pidUpdate(); //LQI thrust control
    gimbalControl(); //gimbal vectoring control
    sendCmd();
  }

  void DragonGimbalPitchOnly::gimbalControl()
  {
    if (control_timestamp_ < 0) return;

    /* get roll/pitch angle */
    double roll_angle = estimator_->getState(State::ROLL_COG, estimate_mode_)[0];
    double pitch_angle = estimator_->getState(State::PITCH_COG, estimate_mode_)[0];

    int rotor_num = kinematics_->getRotorNum();

    /* get links pose w.r.t. CoG frame */
    std::vector<Eigen::Vector3d> links_normal_from_cog = kinematics_->getLinksNormalFromCog<Eigen::Vector3d>();
    std::vector<Eigen::Vector3d> rotors_origin_from_cog = kinematics_->getRotorsOriginFromCog<Eigen::Vector3d>();
    Eigen::Matrix3d links_inertia_inv = kinematics_->getInertia<Eigen::Matrix3d>().inverse();

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, rotor_num);

    for(int i = 0; i < rotor_num; i++)
    {
      P(0, i) = (rotors_origin_from_cog.at(i)[0] * links_normal_from_cog.at(i)[1] -
                 rotors_origin_from_cog.at(i)[1] * links_normal_from_cog.at(i)[0]) * links_inertia_inv(2, 2); // yaw, omit the interference of yaw tilting control on roll and pitch
      P(1, i) = links_normal_from_cog.at(i)[0] / kinematics_->getMass(); // r_x
      P(2, i) = links_normal_from_cog.at(i)[1] / kinematics_->getMass(); // r_y
    }
    Eigen::VectorXd f = pseudoinverse(P) * Eigen::Vector3d(target_yaw_[0], target_pitch_ - (pitch_angle * 9.8), target_roll_ - (-roll_angle * 9.8));

    if(control_verbose_)
      {
        std::cout << "gimbal P:"  << std::endl << P << std::endl;
        std::cout << "gimbal f:"  << std::endl << f << std::endl;
      }
    std_msgs::Float32MultiArray target_force_msg;

    for(int i = 0; i < rotor_num; i++)
      {
        target_gimbal_angles_[2 * i] = 0; //gimbal_i_roll
        target_gimbal_angles_[2 * i + 1] = asin(f(i) / kinematics_->getOptimalHoveringThrust()[i]);

        /* ros publish */
        target_force_msg.data.push_back(f(i));
      }
    gimbal_target_force_pub_.publish(target_force_msg);
  }

  void DragonGimbalPitchOnly::sendCmd()
  {
    /* send base throttle command */
    spinal::FourAxisCommand flight_command_data;
    flight_command_data.angles[0] =  0;
    flight_command_data.angles[1] =  0;

    flight_command_data.base_throttle.resize(motor_num_);
    for(int i = 0; i < motor_num_; i++)
      flight_command_data.base_throttle[i] = target_throttle_[i];
    flight_cmd_pub_.publish(flight_command_data);

    /* send gimbal control command */
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < kinematics_->getRotorNum() * 2; i++)
      gimbal_control_msg.position.push_back(target_gimbal_angles_[i]);

    gimbal_control_pub_.publish(gimbal_control_msg);
  }

  void DragonGimbalPitchOnly::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
  {
    joint_state_ = *state;
    kinematics_->updateRobotModel(joint_state_);
    kinematics_->modelling();
  }

  void DragonGimbalPitchOnly::cfgYawPidCallback(aerial_robot_base::XYPidControlConfig &config, uint32_t level)
  {
    if(config.xy_pid_control_flag)
      {
        printf("Yaw Pid Param:");
        switch(level)
          {
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
            yaw_gains_[0][0] = -config.p_gain;
            printf("change the p gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
            yaw_gains_[0][1] = config.i_gain;
            printf("change the i gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
            yaw_gains_[0][2] = config.d_gain;
            printf("change the d gain\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
            yaw_limit_ = config.limit;
            printf("change the limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
            yaw_terms_limits_[0] = config.p_limit;
            printf("change the p limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
            yaw_terms_limits_[1] = config.i_limit;
            printf("change the i limit\n");
            break;
          case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
            yaw_terms_limits_[2] = config.d_limit;
            printf("change the d limit\n");
            break;
          default :
            printf("\n");
            break;
          }
      }
  }

  void DragonGimbalPitchOnly::rosParamInit()
  {
    FlatnessPid::rosParamInit();

    ros::NodeHandle nh_global("~");
    nh_global.param("simulation", simulation_, false);
    cout << nh_global.getNamespace() << ",  simulaiton  is " << simulation_ << endl;

    string ns = nhp_.getNamespace();
    nhp_.param("control_verbose", control_verbose_, false);
    if(param_verbose_) cout << ns << ": control_verbose is " << control_verbose_ << endl;
  }
};
