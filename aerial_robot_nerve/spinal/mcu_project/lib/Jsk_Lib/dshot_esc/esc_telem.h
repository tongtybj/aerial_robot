//
// Created by jinjie on 24/01/18.
//

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include "util/ring_buffer.h"
#include "math/AP_Math.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <spinal/ESCTelemetry.h>
#include <spinal/ESCTelemetryArray.h>

#define ESC_BUFFER_SIZE 512

namespace
{
#ifdef STM32H7
  uint8_t rx_buf_[ESC_BUFFER_SIZE] __attribute__((section(".EscRxBufferSection")));
#else
  uint8_t rx_buf_[ESC_BUFFER_SIZE];
#endif
  uint32_t rd_ptr_ = 0;
}

class ESCReader
{
public:
  ESCReader(){};
  ~ESCReader(){};

  UART_HandleTypeDef *huart_;

  spinal::ESCTelemetry esc_msg_1_;
  spinal::ESCTelemetry esc_msg_2_;
  spinal::ESCTelemetry esc_msg_3_;
  spinal::ESCTelemetry esc_msg_4_;

  void init(UART_HandleTypeDef* huart);
  void update(int motor_id);
  bool available();
  void clearDMABuffer();
  int readOneByte();

  bool is_new_msg_ = false;

private:
  uint8_t step_ = 0;
  uint8_t msg_id_ = 0;
  uint16_t payload_length_ = 0;
  uint16_t payload_counter_ = 0;
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t class_ = 0;

  void readOnePacket(spinal::ESCTelemetry& esc_msg);
};

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);

#endif  // ESC_TELEM_H
