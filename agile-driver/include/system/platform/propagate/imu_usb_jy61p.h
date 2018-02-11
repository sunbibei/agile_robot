/*
 * imu_usb_jy61p.h
 *
 *  Created on: Dec 26, 2017
 *      Author: robot
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_PROPAGATE_IMU_USB_JY61P_H_
#define INCLUDE_SYSTEM_PLATFORM_PROPAGATE_IMU_USB_JY61P_H_

#include "system/platform/propagate/usb.h"

namespace middleware {

class ImuJY61P: public UsbPropagate {
public:
  ImuJY61P(const MiiString& l = "jy61p");
  virtual ~ImuJY61P();

public:
  virtual bool write(const Packet&) override;
  virtual bool read (class Packet&) override;

protected:
  int         read_status_;

  char*       read_buf_;
  // pointer helper
  char*       buf_top_;
  char*       buf_btm_;
  const char* BUF_EOF_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_PLATFORM_PROPAGATE_IMU_USB_JY61P_H_ */
