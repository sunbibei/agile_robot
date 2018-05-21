/*
 * leg_node.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_APPS_HW_UNIT_LEG_NODE_H_
#define INCLUDE_APPS_HW_UNIT_LEG_NODE_H_

#include "platform/sw_node/sw_node.h"
#include "foundation/utf.h"

namespace agile_robot {

class LegNode: public SWNode {
public:
  LegNode(const std::string& __l = Label::null);
  virtual bool auto_init() override;

  virtual ~LegNode();

  virtual void handleMsg(const Packet&)          override;
  ///! return false, this node is not need to send command.
  virtual bool requireCmdDeliver()               override
  { return false; }
  ///! In the agile robot, this interface has Deprecated.
  // virtual bool generateCmd(std::vector<Packet>&) override;

protected:
  // there are three joint in each leg
  LegType                                leg_;
  std::vector<class Joint*>              jnts_by_type_;
  // No exist information of motor.
  // std::vector<class Motor*>              motors_by_type_;
  class ForceSensor*                     td_;

  // The order match the @joints_by_type_
  std::vector<class __PrivateLinearParams*> jnt_params_;
  // The constant pointer of the joint command
//  const double*             jnt_cmds_[JntType::N_JNTS];
//  const short*              motor_cmds_[JntType::N_JNTS];
//  const JntCmdType&         jnt_mode_;

///! Helper methods
private:
  void __parse_heart_beat_1(const unsigned char*);
  void __parse_heart_beat_2(const unsigned char*);
//  void __parse_motor_cmd_1(const unsigned char*);
//  void __parse_motor_cmd_2(const unsigned char*);

//  bool __fill_pos_cmd(std::vector<Packet>& pkts);
//  bool __fill_vel_cmd(std::vector<Packet>& pkts);
//  bool __fill_tor_cmd(std::vector<Packet>& pkts);
//  bool __fill_pos_vel_cmd(std::vector<Packet>& pkts);
//  bool __fill_motor_vel_cmd(std::vector<Packet>& pkts);
};

} /* namespace middleware */

#endif /* INCLUDE_APPS_HW_UNIT_LEG_NODE_H_ */
