/*
 * motor_node.h
 *
 *  Created on: Apr 12, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_
#define INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_

#include "sw_node.h"

///! forward declares
class Pid;
class TimeControl;
namespace agile_robot {

class MotorNode0: public SWNode {
public:
  MotorNode0();
  virtual bool auto_init() override;

  virtual ~MotorNode0();

  virtual void handleMsg(const Packet&)          override;
  virtual bool generateCmd(std::vector<Packet>&) override;

///! The helper methods
private:
  bool __fill_pos_cmd(std::vector<Packet>& pkts);
  // bool __fill_vel_cmd(std::vector<Packet>& pkts);
  // bool __fill_tor_cmd(std::vector<Packet>& pkts);
  // bool __fill_pos_vel_cmd(std::vector<Packet>& pkts);
  // bool __fill_motor_vel_cmd(std::vector<Packet>& pkts);
  void __parse_heart_beat_1(const unsigned char*);

protected:
  ///! Whether is startup the motor
  int                       is_startup_;
  // there are three joint in each leg
  LegType                   leg_;
  ///! The type of joint.
  JntType                   jnt_;
  ///! update each motor, this vector order by the type of joint.
  class Motor*              motor_handle_;
  ///! the source of command, this vector order by the type of joint.
  class Joint*              joint_handle_;
  ///! the handle of pid control
  class Pid*                joint_pid_;
  ///! the command of the output of pid
  double                    motor_pidout_;
  // The constant pointer of the joint command
  const JntCmdType&         jnt_mode_;

  ///! For generateCmd delay
  class TimeControl*        cmd_tick_time_ctrl_;
  uint64_t                  cmd_tick_interval_;
  uint64_t                  sum_tick_interval_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_SYSTEM_PLATFORM_SW_NODE_MOTOR_NODE_H_ */
