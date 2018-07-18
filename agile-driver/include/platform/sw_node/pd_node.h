/*
 * leg_node.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_APPS_HW_UNIT_LEG_NODE_PD_H_
#define INCLUDE_APPS_HW_UNIT_LEG_NODE_PD_H_

#include "platform/sw_node/sw_node.h"
#include "foundation/utf.h"

namespace agile_robot {

class PdNode: public SWNode {
public:
  PdNode();
  virtual bool auto_init() override;

  virtual ~PdNode();

public:
  virtual void handleMsg(const Packet&)          override;
  ///! In the agile robot, this interface has Deprecated.
  virtual bool generateCmd(std::vector<Packet>&) override;

protected:
  std::vector<std::vector<class Joint*>>          jnts_by_type_;
  // The order match the @joints_by_type_
  std::vector<std::vector<class __LinearParams*>> jnt_params_;
  // The constant pointer of the joint command
//  const double*             jnt_cmds_[JntType::N_JNTS];
//  const short*              motor_cmds_[JntType::N_JNTS];
//  const JntCmdType&         jnt_mode_;

///! Helper methods
private:
  void __parse_heart_beat_1(const unsigned char*);
//  void __parse_motor_cmd_1(const unsigned char*);
//  void __parse_motor_cmd_2(const unsigned char*);

//  bool __fill_pos_cmd(std::vector<Packet>& pkts);
//  bool __fill_vel_cmd(std::vector<Packet>& pkts);
//  bool __fill_tor_cmd(std::vector<Packet>& pkts);
//  bool __fill_pos_vel_cmd(std::vector<Packet>& pkts);
//  bool __fill_motor_vel_cmd(std::vector<Packet>& pkts);
};

} /* namespace middleware */

#endif /* INCLUDE_APPS_HW_UNIT_LEG_NODE_PD_H_ */
