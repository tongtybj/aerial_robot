/*
******************************************************************************
* File Name          : mag_encoder.h
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __MCP3425_H
#define __MCP3425_H

#include "config.h"
#include <ros.h>
#include <std_msgs/Float32.h>


class MCP3425
{
public:
  MCP3425();
  ~MCP3425(){};

  static const uint8_t MCP_I2C_ADDRESS =  0x68; // 1011000
  static const uint8_t MCP_CONFIG_REGISTER = 0b10011000; //16bit 15sps PGA x1
  static constexpr float V_REF = 2.048f;

  void init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh);
  void update(void);

private:
  I2C_HandleTypeDef* hi2c_;
  ros::NodeHandle* nh_;
  ros::Publisher adc_pub_;

  std_msgs::Float32 adc_msg_;

};

#endif
