/*
 * propagate.h
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_
#define INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_

#include "system/platform/protocol/qr_protocol.h"
#include "foundation/label.h"
#include <vector>

namespace middleware {

class Propagate : public Label {
  friend class PropagateManager;
public:
  Propagate(const MiiString&);
  // 妥协方案
  virtual bool auto_init() override;

  /**
   * @brief This method initialize the communication device and get ready for
   *        @write and @read
   * @return Return true if initialization successful, or return false
   */
  virtual bool start();
  /**
   * @brief This method stop the communication device.
   */
  virtual void stop();

  /**
   * @brief This method completes the function that convert the Packet
   *        to the form of message which be needed by the specific communication
   *        type, and writing the converted message to robot.
   * @param pkt[in] The Packet object need to convert and write
   * @return Return true if successful, or return false
   */
  virtual bool write(const Packet& pkt);
  /**
   * @brief This method must be implemented by subclass, and convert the message
   *        which received from the specific communication type to the form of Packet,
   * @param pkt[out] The Packet object converted from the message
   * @return Return true if read successful, or return false
   */
  virtual bool read(Packet& pkt);

protected:
  unsigned char bus_id_;
  MiiString     propa_name_;
};

} /* namespace middleware */

#endif /* INCLUDE_MIDDLEWARE_PROPAGATE_PROPAGATE_H_ */
