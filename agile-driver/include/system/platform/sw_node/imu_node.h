/*
 * imu_node.h
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_IMU_NODE_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_IMU_NODE_H_

#include "system/platform/sw_node/sw_node.h"

namespace middleware {

class ImuNode: public SWNode {
public:
  ImuNode(const std::string& __l = Label::null);
  virtual bool auto_init() override;

  virtual ~ImuNode();

public:
  virtual bool requireCmdDeliver() override;
  virtual void handleMsg(const Packet&) override;

protected:
  class ImuSensor* imu_sensor_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_IMU_NODE_H_ */
