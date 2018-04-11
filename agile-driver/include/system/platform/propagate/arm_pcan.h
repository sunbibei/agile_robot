/*
 * arm_pcan.h
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_ARM_PCAN_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_ARM_PCAN_H_

#include "pcan.h"

namespace agile_robot {

class ArmPcan: public PCanPropa {
public:
  ArmPcan(const std::string& l = "arm_pcan");

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
