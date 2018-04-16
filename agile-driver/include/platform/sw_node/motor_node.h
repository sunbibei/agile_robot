/*
 * motor_node.h
 *
 *  Created on: Apr 12, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_

#include "sw_node.h"

namespace agile_robot {

class MotorNode: public SWNode {
public:
  MotorNode();
  virtual bool auto_init() override;

  virtual ~MotorNode();

  virtual void handleMsg(const Packet&)          override;
  virtual bool generateCmd(std::vector<Packet>&) override;

protected:
  // there are three joint in each leg
  LegType                   leg_;
  ///! update each motor, this vector order by the type of joint.
  std::vector<class Motor*> motors_by_type_;
  ///! the source of command, this vector order by the type of joint.
  std::vector<class Joint*> joints_by_type_;

  // The constant pointer of the joint command
  const JntCmdType&         jnt_mode_;
  const double*             jnt_cmds_[JntType::N_JNTS];
  const short*              motor_cmds_[JntType::N_JNTS];
};

} /* namespace agile_robot */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_ */
