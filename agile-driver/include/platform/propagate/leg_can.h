/*
 * leg_can.h
 *
 *  Created on: May 16, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_PLATFORM_PROPAGATE_LEG_CAN_H_
#define INCLUDE_PLATFORM_PROPAGATE_LEG_CAN_H_

#include <platform/propagate/can_usb.h>

namespace agile_robot {

class LegCan: public CanUsb {
public:
  LegCan();
//  virtual bool auto_init() override;

  virtual ~LegCan();

public:
  virtual bool write(const Packet&) override;
  virtual bool read(Packet&)        override;

private:
  VCI_CAN_OBJ recv_can_obj_;
  VCI_CAN_OBJ send_can_obj_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_PROPAGATE_LEG_CAN_H_ */
