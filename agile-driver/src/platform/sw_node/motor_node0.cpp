/*
 * motor_node.cpp
 *
 *  Created on: Apr 12, 2018
 *      Author: bibei
 */

#include "platform/sw_node/motor_node0.h"

#include "foundation/cfg_reader.h"
#include "foundation/utf.h"

#include "repository/resource/joint_manager.h"
#include "repository/resource/joint.h"
#include "repository/resource/motor.h"
#include "repository/registry.h"

#include "toolbox/time_control.h"
#include "toolbox/pid.h"

#include <boost/algorithm/string.hpp>
#include <stdio.h>
#include <iostream>
#include <cstring>

namespace agile_robot {

// angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
// so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
// offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
struct __PrivateLinearParams {
  double scale;
  double offset;
};

MotorNode0::MotorNode0()
  : SWNode("motor_node"), is_startup_(false),
    leg_(LegType::UNKNOWN_LEG),  jnt_(JntType::UNKNOWN_JNT),
    motor_handle_(nullptr), jnt_param_(nullptr), joint_handle_(nullptr),
    joint_pid_(nullptr), motor_pidout_(0),
    jnt_mode_(JointManager::instance()->getJointCommandMode()),
    cmd_tick_time_ctrl_(nullptr), cmd_tick_interval_(2),
    sum_tick_interval_(0) {
  ;
}

MotorNode0::~MotorNode0() {
  delete jnt_param_;
  delete joint_pid_;
  delete cmd_tick_time_ctrl_;
  jnt_param_ = nullptr;
  joint_pid_ = nullptr;
  cmd_tick_time_ctrl_ = nullptr;
  ; // Nothing to do here.
}

bool MotorNode0::auto_init() {
  if (!SWNode::auto_init()) return false;
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "leg", leg_);
  cfg->get_value_fatal(getLabel(), "jnt", jnt_);

  joint_handle_ = JointManager::instance()->getJointHandle(leg_, jnt_);
  if (!joint_handle_ || !joint_handle_->joint_motor())
    LOG_FATAL << "Can't get the joint '" << LEGTYPE2STR(leg_) << " - "
        << JNTTYPE2STR(jnt_) << "' pointer from JointManager, or the motor within joint.";
  ///! Add the instance of motor
  motor_handle_  = joint_handle_->joint_motor();

  jnt_param_  = new __PrivateLinearParams;
  cfg->get_value_fatal(getLabel(), "scale",  jnt_param_->scale);
  cfg->get_value_fatal(getLabel(), "offset", jnt_param_->offset);
  cfg->get_value_fatal(getLabel(), "interval", cmd_tick_interval_);


  ///! Got the parameters of PID
  std::vector<float> gains;
  std::string _sub_tag = Label::make_label(getLabel(), "pid");
  cfg->get_value_fatal(_sub_tag, "gains", gains);
  if (6 != gains.size()) {
    LOG_FATAL << "The input parameters of PID in " << LEGTYPE2STR(leg_)
      << " - " << JNTTYPE2STR(jnt_) << " error!";
  }
  ///! new a pid controller
  joint_pid_ = new Pid(joint_handle_->joint_name() + "-pid");
  joint_pid_->gains(gains[0], gains[1], gains[2]);

  _sub_tag = Label::make_label(getLabel(), "registry");
  bool is_reg = false;
  cfg->get_value(_sub_tag, "enable", is_reg);
  if (is_reg) {
    std::string name = joint_handle_->joint_name() + "_cmd";
    cfg->get_value(_sub_tag, "name", name);
    REG_RESOURCE(name, &motor_pidout_);
  }

  cmd_tick_time_ctrl_ = new TimeControl();
  cmd_tick_time_ctrl_->start();
  return true;
}

void MotorNode0::handleMsg(const Packet& pkt) {
  static std::map<short, unsigned char> _s_data2msgid = {
      {0x5149, MII_MSG_HEARTBEAT_1},
      {0x5856, MII_MSG_HEARTBEAT_2},
      {0x5850, MII_MSG_HEARTBEAT_3}
  };

  short data = 0;
  memcpy(&data, pkt.data, sizeof(short));
  if (_s_data2msgid.end() == _s_data2msgid.find(data)) {
    LOG_ERROR << "ERROR data of msg!";
    LOG_HEADER;
    printf(" NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
      (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
      (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
  }

  float _tor = 0.0;
  int   _vel = 0, _pos = 0;
  switch (_s_data2msgid[data]) {
    case MII_MSG_HEARTBEAT_1:
      if (8 != pkt.size) {
        LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
            << ", the expect size is 8, but the real size is " << pkt.size;
        return;
      }

      memcpy(&_tor, pkt.data + 4, sizeof(_tor));
      // parse the joint state and touchdown data
      motor_handle_->updateMotorTorque(_tor);
      break;
    case MII_MSG_HEARTBEAT_2:
    if (8 != pkt.size) {
        LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
          << ", the expect size is 8, but the real size is " << pkt.size;
        return;
      }
      memcpy(&_vel, pkt.data + 4, sizeof(_vel));
      // parse the joint state and touchdown data
      motor_handle_->updateMotorVelocity(_vel);
     break;
    case MII_MSG_HEARTBEAT_3:
      if (8 != pkt.size) {
        LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
          << ", the expect size is 8, but the real size is " << pkt.size;
        return;
      }
      memcpy(&_pos, pkt.data + 4, sizeof(_pos));
      // parse the joint state and touchdown data
      motor_handle_->updateMotorPosition(_pos);
      break;
    default:
      SWNode::handleMsg(pkt);
  }

}

bool MotorNode0::generateCmd(std::vector<Packet>& pkts) {
  if (!is_startup_) { ///! The first, we need to startup the motor.
    static uint64_t _s_startup_cmds[] = {
        0x0000000000000001, 0x0000000200004D55, 0x0000000100004F4D
    };
    Packet cmd = {bus_id_, node_id_, MII_MSG_MOTOR_3, 8, {0}};
    for (const auto& c : _s_startup_cmds) {
      memcpy(cmd.data, &c, cmd.size);
      pkts.push_back(cmd);
    }

    is_startup_ = true;
    return true;
  }

  bool is_any_valid = false;
  switch (jnt_mode_) {
  case JntCmdType::CMD_POS:
    is_any_valid = __fill_pos_cmd(pkts);
    break;
  default:
    LOG_ERROR << "What a fucking the command mode of joint.";
  }

  return is_any_valid;
}

bool MotorNode0::__fill_pos_cmd(std::vector<Packet>& pkts) {
  sum_tick_interval_ += cmd_tick_time_ctrl_->dt();
  if (sum_tick_interval_ < cmd_tick_interval_) return false;
  sum_tick_interval_ = 0;

  // [0] -> The header of velocity command; [1] -> The command of start move.
  static uint32_t _s_motor_pos_cmds[] = { 0x0000564A, 0x00004742 };

  if (joint_handle_->new_command_) {
    joint_pid_->target(joint_handle_->joint_command());

    joint_handle_->new_command_.store(false);
  }

  Packet cmd = {bus_id_, node_id_, MII_MSG_MOTOR_3, 8, {0}};
  joint_pid_->control(joint_handle_->joint_position(), motor_pidout_);

  ///! push the actual velocity command.
  int motor_cmd = motor_pidout_;
  memcpy(cmd.data + 0, _s_motor_pos_cmds + 0, 4);
  memcpy(cmd.data + 4, &motor_cmd,            4);
  pkts.push_back(cmd);
  ///! push the start command
  cmd.size = 4;
  memcpy(cmd.data + 0, _s_motor_pos_cmds + 1, 4);
  memset(cmd.data + 4, 0x00,                  4);
  pkts.push_back(cmd);
  return true;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorNode0, Label)
