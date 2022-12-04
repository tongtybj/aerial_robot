/*
******************************************************************************
* File Name          : mag_encoder.cpp
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/external_adc/mcp3425.h"

MCP3425::MCP3425():
  adc_pub_("mcp3425/adc_output", &adc_msg_)
{
}

void MCP3425::init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh)
{
  hi2c_ = hi2c;
  nh_ = nh;
  nh_->advertise(adc_pub_);


  // set the configuration
  uint8_t val[1];
  val[0] = MCP_CONFIG_REGISTER;
  int i2c_status = HAL_I2C_Master_Transmit(hi2c_, MCP_I2C_ADDRESS, val, 1, 100);
}

void MCP3425::update(void)
{
  uint8_t adc[2];
  HAL_I2C_Master_Receive(hi2c_, MCP_I2C_ADDRESS, adc, 2, 100);
  float rad_adc = adc[0] << 8 | adc[1];

  float raw_v = (int)rad_adc * V_REF / 32767.0 ;

  adc_msg_.data = raw_v;
  if(nh_->connected()) adc_pub_.publish(&adc_msg_);

}
