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

  virtual bool write(const Packet&) override;
  virtual bool read(Packet&)        override;

private:
  void do_exchange_w();
  void do_exchange_r();

private:
  PVCI_CAN_OBJ recv_msgs_;
  PVCI_CAN_OBJ send_msgs_;

  boost::lockfree::queue<Packet>* recv_buffer_;
  boost::lockfree::queue<Packet>* send_buffer_;

  VCI_INIT_CONFIG config_;
  bool         connected_;

  int      recv_buf_size_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_PROPAGATE_CAN_USB_H_ */
