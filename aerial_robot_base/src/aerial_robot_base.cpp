#include <aerial_robot_base/aerial_robot_base.h>

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private), callback_spinner_(4), main_loop_spinner_(1, &main_loop_queue_),
    controller_loader_("aerial_robot_control", "aerial_robot_control::ControlBase"),
    navigator_loader_("aerial_robot_control", "aerial_robot_navigation::BaseNavigator")
{

  nhp_.param ("param_verbose", param_verbose_, true);
  nhp_.param ("main_rate", main_rate_, 0.0);

  // robot model
  robot_model_ros_ = boost::make_shared<aerial_robot_model::RobotModelRos>(nh_, nhp_);
  auto robot_model = robot_model_ros_->getRobotModel();

  // estimator
  estimator_ = boost::make_shared<aerial_robot_estimation::StateEstimator>();
  estimator_->initialize(nh_, nhp_, robot_model);

  // navigation
  std::string navi_plugin_name;
  if(nh_.getParam("flight_navigation_plugin_name", navi_plugin_name))
    {
      try
        {
          navigator_ = navigator_loader_.createInstance(navi_plugin_name);
        }
      catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }
  else
    {
      ROS_DEBUG("use default class for flight navigation: aerial_robot_navigation::BaseNavigator");
      navigator_ = boost::make_shared<aerial_robot_navigation::BaseNavigator>();
    }
  navigator_->initialize(nh_, nhp_, robot_model, estimator_);

  //  controller
  try
    {
      std::string aerial_robot_control_name;
      nh_.param ("aerial_robot_control_name", aerial_robot_control_name, std::string("aerial_robot_control/flatness_pid"));
      controller_ = controller_loader_.createInstance(aerial_robot_control_name);
      controller_->initialize(nh_, nhp_, robot_model, estimator_, navigator_, main_rate_);
    }
  catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

  if(param_verbose_) cout << nhp_.getNamespace() << ": main_rate is " << main_rate_ << endl;
  if(main_rate_ <= 0)
    ROS_ERROR_STREAM("mian rate is negative, can not run the main timer");
  else
    {
      // note1: separate the thread for main control (including navigation) loop to guarantee a relatively stable loop rate

      ros::TimerOptions ops(ros::Duration(1.0 / main_rate_),
                            boost::bind(&AerialRobotBase::mainFunc, this, _1),
                            &main_loop_queue_);
      main_timer_ = nhp_.createTimer(ops);
      main_loop_spinner_.start();
    }


  // note2: callback_spinner_ calls following items with 4 threads
  //  - all subscribers (joint state for robot model, sensor for state estimation, uav/nav for navigation)
  //  - statePublish timer in state estimator for publish odometry and tf
  //  - service server
  callback_spinner_.start();

}

AerialRobotBase::~AerialRobotBase()
{
  // stop manually to avoid following error (message)
  // terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
  // what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
  main_timer_.stop();
  main_loop_spinner_.stop();
}

void AerialRobotBase::mainFunc(const ros::TimerEvent & e)
{
  if (!e.last_real.isZero())
  {
      double dt_real = e.current_real.toSec() - e.last_real.toSec();
      double tolerance = 0.05;  // 5% tolerance. This value is set based on experience.
      double dt_desire = (1.0 + tolerance) * 1.0 / main_rate_;
      if (dt_real > dt_desire)
      {
          ROS_WARN("main loop rate is too low: (ts_real) %f s > (ts_desire with %2f%% tolerance) %f s ", dt_real,
                   tolerance * 100, dt_desire);
      }
  }
  navigator_->update();
  controller_->update();
}
