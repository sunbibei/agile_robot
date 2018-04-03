/*
 * propagate_imp_pcan.h
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#ifndef INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_
#define INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_

#include "propagate.h"

#define linux
#include <unistd.h>
#include <asm/types.h>

#include <PCANBasic.h>

namespace middleware {

class PcanPropagate: public Propagate {
public:
  PcanPropagate(const std::string& l = "pcan");
  virtual ~PcanPropagate();
  virtual bool auto_init() override;

  virtual bool start() override;
  virtual void stop()  override;

  /*virtual bool write(const class Packet&) = 0;
  virtual bool read (class Packet&)       = 0;*/

protected:
  bool         connected_;

  struct PcanConfig {
    TPCANHandle   channel;
    TPCANBaudrate baud_rate;
    TPCANType     type;
    DWORD         port;
    WORD          interrupt;
  } pcan_config_;

  TPCANStatus tmp_pcan_status_;
};

} /* namespace qr_driver */

#endif /* INCLUDE_PROPAGATE_INTERFACE_PROPAGATE_IMP_PCAN_H_ */
