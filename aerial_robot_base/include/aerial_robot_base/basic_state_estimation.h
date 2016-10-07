#ifndef BASIC_STATE_ESTIMATION_H
#define BASIC_STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>
#include <aerial_robot_base/States.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <vector>

#include <Eigen/Core>

#include <std_msgs/UInt8.h>
#include <kalman_filter/kf_base_plugin.h>
#include <pluginlib/class_loader.h>

class BasicEstimator
{
 public:
 BasicEstimator(ros::NodeHandle nh, ros::NodeHandle nh_private)
   : nh_(nh, "estimator"), nhp_(nh_private, "estimator")
  {
    state_pub_ = nh_.advertise<nav_msgs::Odometry>("/uav/state", 1);
    full_states_pub_ = nh_.advertise<aerial_robot_base::States>("/uav/full_states", 1);
    estimate_height_mode_sub_ = nh_.subscribe<std_msgs::UInt8>("/estimate_height_mode", 1, &BasicEstimator::heightEstimateModeCallback, this, ros::TransportHints().tcpNoDelay());

    flying_flag_ = false;
    landing_mode_flag_ = false;

    landed_flag_ = false;
    un_descend_flag_ = false;

    gt_state_.resize(9);
    ee_state_.resize(9);
    ex_state_.resize(9);

    for(int i = 0; i < 9; i ++)
      {
        for(int j = 0; j < 3; j++)
          {
            (gt_state_[i])[j] = 0; //ground truth
            (ee_state_[i])[j] = 0; //egomtion estimate
            (ex_state_[i])[j] = 0; //experiment
          }
      }

    state_pos_z_offset_ = 1.0; //1m

    sys_stamp_ = ros::Time::now();//removed this

    setHeightEstimateMode(ONLY_BARO_MODE);
    landing_height_ = 0;
  }

  virtual ~BasicEstimator(){}

  //mode
  const static uint8_t GROUND_TRUTH = 0;
  const static uint8_t EGOMOTION_ESTIMATE = 1;

  static const uint8_t X_W = 0; //x in world coord
  static const uint8_t Y_W = 1; //y in world coord
  static const uint8_t Z_W = 2; //z in world coord
  static const uint8_t ROLL_W = 3; //roll in world coord
  static const uint8_t PITCH_W = 4; //pitch in world coord
  static const uint8_t YAW_W_COG = 5; //yaw of cog in world coord
  static const uint8_t X_B = 6; //x in board coord
  static const uint8_t Y_B = 7; //y in board coord
  static const uint8_t YAW_W_B = 8; //yaw of board in world coord

  inline ros::Time getSystemTimeStamp(){ return sys_stamp_;}
  inline void setSystemTimeStamp(ros::Time sys_stamp){ sys_stamp_ = sys_stamp;}

  inline float getGTState(int axis, int mode){ return (gt_state_[axis])[mode];}
  inline void setGTState(int axis, int mode, float value){ (gt_state_[axis])[mode] = value;}

 float getEEState(int axis, int mode)
  {
    boost::lock_guard<boost::mutex> lock(ee_state_mutex_); 
   return (ee_state_[axis])[mode];
  }
 void setEEState(int axis, int mode, float value)
  { 
    boost::lock_guard<boost::mutex> lock(ee_state_mutex_); 
    (ee_state_[axis])[mode] = value;
  }
 float getEXState(int axis, int mode)
  { 
    boost::lock_guard<boost::mutex> lock(ex_state_mutex_); 
    return (ex_state_[axis])[mode];
  }
 void setEXState(int axis, int mode, float value)
  {
    boost::lock_guard<boost::mutex> lock(ex_state_mutex_); 
    (ex_state_[axis])[mode] = value;
  }

  inline void setSensorFusionFlag(bool flag){sensor_fusion_flag_ = flag;  }
  inline bool getSensorFusionFlag(){return sensor_fusion_flag_; }

  virtual float getPosZOffset() {  return  state_pos_z_offset_;}
  virtual void setPosZOffset(float pos_z_offset){  state_pos_z_offset_ = pos_z_offset;}

  //start flying flag (~takeoff)
  virtual bool getFlyingFlag() {  return  flying_flag_;}
  virtual void setFlyingFlag(bool flag){  flying_flag_ = flag;}
  //start landing mode
  virtual bool getLandingMode() {  return  landing_mode_flag_;}
  virtual void setLandingMode(bool flag){  landing_mode_flag_ = flag;}
  // landed flag (acc_z check, ground shock)
  virtual bool getLandedFlag() {  return  landed_flag_;}
  virtual void setLandedFlag(bool flag){  landed_flag_ = flag;}

  /* when takeoff, should use the undescend mode be true */
  inline void setUnDescendMode(bool flag){un_descend_flag_ = flag;  }
  inline bool getUnDescendMode(){return un_descend_flag_; }

  /* landing height is set for landing to different terrain */
  virtual void setLandingHeight(float landing_height){ landing_height_ = landing_height;}
  virtual float getLandingHeight(){ return landing_height_;}

  inline boost::shared_ptr<kf_base_plugin::KalmanFilter> getFuserEgomotion(int no) { return fuser_egomotion_[no];}
  inline boost::shared_ptr<kf_base_plugin::KalmanFilter> getFuserExperiment(int no) { return fuser_experiment_[no];}
  inline std::string  getFuserEgomotionName(int no) { return fuser_egomotion_name_[no];}
  inline std::string  getFuserExperimentName(int no) { return fuser_experiment_name_[no];}
  inline std::string  getFuserEgomotionPluginName(int no) { return fuser_egomotion_plugin_name_[no];}
  inline std::string  getFuserExperimentPluginName(int no) { return fuser_experiment_plugin_name_[no];}

  inline int  getFuserEgomotionId(int no) { return fuser_egomotion_id_[no];}
  inline int  getFuserExperimentId(int no) { return fuser_experiment_id_[no];}
  inline int  getFuserEgomotionNo() { return fuser_egomotion_no_;}
  inline int  getFuserExperimentNo() { return fuser_experiment_no_;}

  inline int getStateMode() {return state_mode_;}
  inline void setStateMode(int state_mode) {state_mode_ = state_mode;}

  /* the height estimation related funrction */
  const static uint8_t ONLY_BARO_MODE = 0; //we estimate the height only based the baro, but the bias of baro is constant(keep the last eistamted value)
  const static uint8_t WITH_BARO_MODE = 1; //we estimate the height using range sensor etc. without the baro, but we are estimate the bias of baro
  const static uint8_t WITHOUT_BARO_MODE = 2; //we estimate the height using range sensor etc. with the baro, also estimating the bias of baro

  inline void setHeightEstimateMode(uint8_t height_estimate_mode){ height_estimate_mode_ = height_estimate_mode;}
  inline int getHeightEstimateMode(){return height_estimate_mode_;}

  inline bool onlyImuYaw(){return only_imu_yaw_;}

 protected:

  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher full_states_pub_, state_pub_;
  ros::Subscriber estimate_height_mode_sub_;

  ros::Time sys_stamp_;

  //for mutex
  boost::mutex ee_state_mutex_, ex_state_mutex_;

  int state_mode_;

  // xw, yw, zw, rollw, pitchw, yaww of cog, xb, yb, yaww of board (9) 
  // (0): x, (1): dx, (2); ddx
  std::vector< Eigen::Vector3d>  gt_state_; //ground truth
  std::vector< Eigen::Vector3d>  ee_state_; //egomotion estimate in world coord
  std::vector< Eigen::Vector3d>  ex_state_; //experiment in world coord

  float state_pos_z_offset_;

  //tf::TransformBroadcaster* br_;
  boost::shared_ptr< pluginlib::ClassLoader<kf_base_plugin::KalmanFilter> > sensor_fusion_loader_ptr_;
  //pluginlib::ClassLoader<kf_base_plugin::KalmanFilter>  sensor_fusion_loader_;

  // sensor fusion
  bool sensor_fusion_flag_;
  int fuser_egomotion_no_, fuser_experiment_no_;
  std::vector<std::string> fuser_egomotion_plugin_name_;
  std::vector<std::string> fuser_experiment_plugin_name_;
  std::vector<std::string> fuser_egomotion_name_;
  std::vector<std::string> fuser_experiment_name_;
  std::vector<int> fuser_egomotion_id_;
  std::vector<int> fuser_experiment_id_;
  std::vector< boost::shared_ptr<kf_base_plugin::KalmanFilter> > fuser_egomotion_;
  std::vector< boost::shared_ptr<kf_base_plugin::KalmanFilter> > fuser_experiment_;
  bool flying_flag_;
  bool landing_mode_flag_;
  bool landed_flag_;
  bool un_descend_flag_;
  uint8_t height_estimate_mode_;
  float landing_height_; //we have to consider the terrain change during the flight,then the landing height is no longer 0.

  /* the only imu yaw flag */
  bool only_imu_yaw_;

  /* force to change the estimate mode */
  void heightEstimateModeCallback(const std_msgs::UInt8ConstPtr & mode_msg)
  {
    height_estimate_mode_ = mode_msg->data;
    ROS_INFO("change the height estimate mode: %d", height_estimate_mode_);
  }

};


#endif
