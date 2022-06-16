/*
******************************************************************************
* File Name          : electromagnet.h
* Description        : electronic meganet device to pick up object in task3
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __BATTERY_STATUS_H
#define __BATTERY_STATUS_H

#include <config.h>
#include "flashmemory/flashmemory.h"

/* ros */
#include <ros.h>
#include <std_msgs/Float32.h>

#define VOLTAGE_CHECK_INTERVAL 20 // ms
#define ROS_PUB_INTERVAL 100 //ms

/* https://www.sparkfun.com/datasheets/Sensors/DC%20Voltage%20and%20Current%20Sense%20PCB%20Spec%20Sheet.pdf */

class BatteryStatus
{
public:
  BatteryStatus():  voltage_status_pub_("adc1/voltage", &voltage_status_msg_),
                    adc_scale_sub_("adc1/set/adc_scale", &BatteryStatus::adcScaleCallback, this),
					  vol_offset_sub_("adc1/set/vol_offset", &BatteryStatus::volOffsetCallback, this),
					  adc_lpf_rate_sub_("adc1/set/lpf_rate", &BatteryStatus::adcLpfRateCallback, this)
  {
  }

  ~BatteryStatus(){}


  void init(ADC_HandleTypeDef *hadc, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(voltage_status_pub_);
    nh_->subscribe<ros::Subscriber<std_msgs::Float32, BatteryStatus> >(adc_scale_sub_);
    nh_->subscribe<ros::Subscriber<std_msgs::Float32, BatteryStatus> >(vol_offset_sub_);
    nh_->subscribe<ros::Subscriber<std_msgs::Float32, BatteryStatus> >(adc_lpf_rate_sub_);
    hadc_ = hadc;

    reset_ = true;
    ros_pub_last_time_ = HAL_GetTick();

    FlashMemory::addValue(&adc_scale_, sizeof(float));
    FlashMemory::addValue(&vol_offset_, sizeof(float));
    FlashMemory::addValue(&lpf_rate_, sizeof(float));

    HAL_ADC_Start(hadc_);
  }

  void adcScaleCallback(const std_msgs::Float32& cmd_msg)
  {
    adc_scale_ = cmd_msg.data;
    FlashMemory::erase();
    FlashMemory::write();
    reset_ = true;
    nh_->loginfo("overwrite scale for ADC1");
  }

  void volOffsetCallback(const std_msgs::Float32& cmd_msg)
  {
    vol_offset_ = cmd_msg.data;
    FlashMemory::erase();
    FlashMemory::write();
    reset_ = true;
    nh_->loginfo("overwrite voltage offset for ADC1");
  }

  void adcLpfRateCallback(const std_msgs::Float32& cmd_msg)
  {
	 lpf_rate_ = cmd_msg.data;
    FlashMemory::erase();
    FlashMemory::write();
    reset_ = true;
    nh_->loginfo("overwrite LPF rate for ADC1");
  }

  void update()
  {
    if(HAL_ADC_PollForConversion(hadc_,10) == HAL_OK)
      adc_value_ = HAL_ADC_GetValue(hadc_);

    HAL_ADC_Start(hadc_);

    float voltage =  adc_scale_ * adc_value_ - vol_offset_; // offset voltage

    if (reset_) {
    	voltage_ = voltage;
    	reset_ = false;
    }

    /* filtering */
    voltage_ = (1 - lpf_rate_) * voltage_  + lpf_rate_ * voltage;

    if(HAL_GetTick() - ros_pub_last_time_ > ROS_PUB_INTERVAL)
      {
        ros_pub_last_time_ = HAL_GetTick();
        voltage_status_msg_.data = voltage_;
        voltage_status_pub_.publish(&voltage_status_msg_);
      }
  }

  ros::Publisher voltage_status_pub_;
  ros::Subscriber<std_msgs::Float32, BatteryStatus> adc_scale_sub_;
  ros::Subscriber<std_msgs::Float32, BatteryStatus> vol_offset_sub_;
  ros::Subscriber<std_msgs::Float32, BatteryStatus> adc_lpf_rate_sub_;
  std_msgs::Float32 voltage_status_msg_;

  inline float getVoltage() {return voltage_;}

private:
  ros::NodeHandle* nh_;
  ADC_HandleTypeDef *hadc_;

  float adc_value_;
  float adc_scale_;
  float vol_offset_;
  float lpf_rate_;
  float voltage_;
  bool reset_;

  uint32_t ros_pub_last_time_;
};

#endif
