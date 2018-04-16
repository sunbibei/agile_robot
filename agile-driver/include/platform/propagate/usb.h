/*
 * usb.h
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_

#include "platform/propagate/propagate.h"

namespace agile_robot {

class UsbPropa: public Propagate {
public:
  UsbPropa(const std::string& l = "usb");
  virtual bool auto_init() override;

  virtual ~UsbPropa();

  struct UsbConfig {
    std::string file_name;
    std::string parity;
    int       baud_rate;
    int       data_bit;
    int       stop_bit;
  };

public:
  virtual bool start() override;
  virtual void stop()  override;

  /*virtual bool write(const Packet&) override;
  virtual bool read (class Packet&) override;*/

protected:
  bool  opened_;
  int   usb_fd_;
  unsigned char node_id_;

  UsbConfig usb_config_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_USB_H_ */
