/*
 * motor_can.h
 *
 *  Created on: May 16, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_PLATFORM_PROPAGATE_MOTOR_CAN_H_
#define INCLUDE_PLATFORM_PROPAGATE_MOTOR_CAN_H_

#include <platform/propagate/can_usb.h>

namespace agile_robot {

class MotorCan: public CanUsb {
public:
  MotorCan();
  virtual bool auto_init() override;

  virtual ~MotorCan();

public:
  virtual bool start()              override;
  virtual bool write(const Packet&) override;
  virtual bool read(Packet&)        override;

private:
  /*!
   * @brief The thread function that request the motor of status from the list of register
   */
  void state_tick();
  ///! Whether enable the proxy for request the state of each motor
  bool enable_proxy_;
  ///! The variable will read from the configure file.
  std::vector<unsigned char> proxy_list_;

private:
  ///! The relationship between MsgId and Data.
  unsigned int                         msgid2idx_lut_[MAX_MSG_NUM];
  std::map<unsigned int, unsigned int> idx2msgid_lut_;

  VCI_CAN_OBJ recv_can_obj_;
  VCI_CAN_OBJ send_can_obj_;

  ///! a few of data for parse the 301 protocol to control or read from motor.
  static unsigned int s_sendid_base_;
  static unsigned int s_recvid_base_;
  static VCI_CAN_OBJ  s_enable_can_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_PROPAGATE_MOTOR_CAN_H_ */
