/*
 * gazebo_propagate.h
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_GAZEBO_PROPAGATE_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_GAZEBO_PROPAGATE_H_

#include "propagate.h"
//! The default style is msgq.
// #define USE_SHM
///! Forward declare
class MsgQueue;
class SharedMem;

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
  ///! The message queue or the shared memory
#ifdef USE_SHM
  SharedMem*                      ipc_;
  void*                           shm_r_buf_;
  void*                           shm_w_buf_;
  size_t                          start_pos_;
  MiiVector<unsigned char>        id_2_leg_lut_;
#else
  MsgQueue*                       ipc_;
#endif
  MiiString       leg_node_ipc_name_;
  MiiString       cmd_ipc_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_GAZEBO_PROPAGATE_H_ */
