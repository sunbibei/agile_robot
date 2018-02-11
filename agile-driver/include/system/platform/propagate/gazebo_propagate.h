/*
 * gazebo_propagate.h
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_GAZEBO_PROPAGATE_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_GAZEBO_PROPAGATE_H_

#include "propagate.h"
///! forward declare
class MsgQueue;

namespace middleware {

class GzPropagateP : public Propagate {
public:
  GzPropagateP();
  virtual bool auto_init() override;

  virtual ~GzPropagateP();

public:
  virtual bool start() override;
  virtual void stop()  override;

  virtual bool write(const class Packet&) override;
  virtual bool read (class Packet&)       override;

protected:
  MsgQueue*       msgq_;
  MiiString       leg_node_msgq_name_;
  MiiString       cmd_msgq_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_GAZEBO_PROPAGATE_H_ */
