/*
 * arm_pcan.h
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_ARM_PCAN_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_ARM_PCAN_H_

#include "system/platform/propagate/pcan.h"

namespace middleware {

class ArmPcan: public PcanPropagate {
public:
  ArmPcan(const MiiString& l = "arm_pcan");

  virtual ~ArmPcan();

public:
  virtual bool write(const class Packet&) override;
  virtual bool read (class Packet&)       override;

protected:
  TPCANMsg     send_msg_;
  TPCANMsg     recv_msg_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_ARM_PCAN_H_ */
