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

#include "system/platform/protocol/qr_protocol.h"
#include "foundation/label.h"

namespace middleware {

/**
 * @brief Each HwUnit associate to the real hardware in the robot,
 *        Note that the real hardware means the node what communicates with
 *        the master.
 */
struct SWNode : public Label {
  friend class SWNodeManager;
  SWNode(const MiiString& __l = Label::null);
  virtual bool auto_init() override;
  virtual ~SWNode();

  /**
   * 该类是否会产生命令下发给机器人
   * 若发回true, 则会每次询问是否有指令下发
   */
  virtual bool requireCmdDeliver();
  /**
   * Propagate接收到的所有Message，通过handleMsg职责链完成解析
   * 因此，将所有数据解析工作延迟到每个具体的硬件子类中进行实现
   */
  virtual void handleMsg(const Packet&);
  /**
   * 某些硬件类型或许会有命令需要下发
   * The new command packet will be pushed back into queue;
   * if generate the Command packet, return true. Or return false.
   */
  virtual bool generateCmd(MiiVector<Packet>&);

protected:
  unsigned char node_id_;
};

} /* namespace quadruped_robot_driver */

#endif /* ROBOT_STATE_BASE_H_ */
