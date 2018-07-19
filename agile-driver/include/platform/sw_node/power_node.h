/*
 * power_node.h
 *
 *  Created on: Nov 14, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_POWER_NODE_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_POWER_NODE_H_

#include "platform/sw_node/sw_node.h"

namespace agile_robot {

class PowerNode: public SWNode {
public:
  PowerNode();
  virtual bool auto_init() override;

  virtual ~PowerNode();

  virtual bool requireCmdDeliver()      override;
  virtual void handleMsg(const Packet&) override;

protected:
  MiiPtr<class Power> power_info_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_POWER_NODE_H_ */
