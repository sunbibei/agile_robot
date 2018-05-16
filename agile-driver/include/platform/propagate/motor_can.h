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
  // virtual bool auto_init() override;

  virtual ~MotorCan();

public:
  virtual bool write(const Packet&) override;
  virtual bool read(Packet&)        override;

private:
  ///! The relationship between MsgId and Data.
  unsigned int                         msgid2idx_lut_[MAX_MSG_NUM];
  std::map<unsigned int, unsigned int> idx2msgid_lut_;

  VCI_CAN_OBJ recv_can_obj_;
  VCI_CAN_OBJ send_can_obj_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_PROPAGATE_MOTOR_CAN_H_ */
