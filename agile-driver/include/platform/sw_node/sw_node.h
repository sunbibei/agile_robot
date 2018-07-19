/*
 * robot_state_base.h
 *
 *  Created on: Nov 15, 2016
 *      Author: silence
 */

#ifndef ROBOT_STATE_BASE_H_
#define ROBOT_STATE_BASE_H_

#define PACKET_CAN

#include <map>
#include <string>
#include <tinyxml.h>
#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>

#include "platform/proto/agile_proto.h"
#include "foundation/label.h"

namespace agile_robot {

/**
 * @brief Each HwUnit associate to the real hardware in the robot,
 *        Note that the real hardware means the node what communicates with
 *        the master.
 */
struct SWNode : public Label {
  friend class SWNodeManager;

  SWNode();
  virtual bool auto_init() override;

  virtual ~SWNode();

  /*!
   * @brief Return true if you need to generate the command, or return false.
   */
  virtual bool requireCmdDeliver();

  /*!
   * @brief The sub-class must be complete this method, parse the Packet.
   */
  virtual void handleMsg(const Packet&);

  /*!
   * @brief The new command packet will be pushed back into queue;
   *        If generate the Command packet, return true. Or return false.
   */
  virtual bool generateCmd(std::vector<Packet>&);

protected:
  ///! the new attributes in agile robot.
  unsigned char bus_id_;
  unsigned char node_id_;
};

} /* namespace quadruped_robot_driver */

#endif /* ROBOT_STATE_BASE_H_ */
