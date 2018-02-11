/*
 * leg_node.h
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#ifndef INCLUDE_APPS_HW_UNIT_LEG_NODE_H_
#define INCLUDE_APPS_HW_UNIT_LEG_NODE_H_

#include "system/platform/sw_node/sw_node.h"
#include "foundation/utf.h"

namespace middleware {

class LegNode: public SWNode {
public:
  LegNode(const MiiString& __l = Label::null);
  virtual bool auto_init() override;

  virtual ~LegNode();

  virtual void handleMsg(const Packet&)        override;
  virtual bool generateCmd(MiiVector<Packet>&) override;

protected:
  // there are three joint in each leg
  LegType                                leg_;
  MiiVector<class Joint*>                jnts_by_type_;
  MiiVector<class Motor*>                motors_by_type_;
  class ForceSensor*                     td_;

  // The order match the @joints_by_type_
  MiiVector<class __PrivateLinearParams*> jnt_params_;
  // The constant pointer of the joint command
  const double*             jnt_cmds_[JntType::N_JNTS];
  const short*              motor_cmds_[JntType::N_JNTS];
  const JntCmdType&         jnt_mode_;

///! Helper methods
private:
  void __parse_heart_beat_1(const unsigned char*);
  void __parse_motor_cmd_1(const unsigned char*);
  void __parse_motor_cmd_2(const unsigned char*);

  bool __fill_pos_cmd(MiiVector<Packet>& pkts);
  bool __fill_vel_cmd(MiiVector<Packet>& pkts);
  bool __fill_tor_cmd(MiiVector<Packet>& pkts);
  bool __fill_pos_vel_cmd(MiiVector<Packet>& pkts);
  bool __fill_motor_vel_cmd(MiiVector<Packet>& pkts);
};

} /* namespace middleware */

#endif /* INCLUDE_APPS_HW_UNIT_LEG_NODE_H_ */
