/*
 * can_usb.h
 *
 *  Created on: Apr 26, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_
#define INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_

#include "propagate.h"

#include "controlcan.h"
#include <boost/lockfree/queue.hpp>

namespace agile_robot {

class CanUsb: public Propagate {
public:
  CanUsb();
  virtual bool auto_init() override;

  virtual ~CanUsb();

public:

  virtual bool start() override;
  virtual void stop()  override;

//  virtual bool write(const Packet&) override;
//  virtual bool read(Packet&)        override;

protected:
  boost::lockfree::queue<VCI_CAN_OBJ>* recv_buffer_;
  boost::lockfree::queue<VCI_CAN_OBJ>* send_buffer_;
  bool         connected_;
  uint64_t     tick_r_interval_;
  uint64_t     tick_w_interval_;

  virtual void do_exchange_w();
  virtual void do_exchange_r();

private:
  int      recv_buf_size_;
  PVCI_CAN_OBJ recv_msgs_;
  PVCI_CAN_OBJ send_msgs_;

  VCI_INIT_CONFIG config_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_ */
