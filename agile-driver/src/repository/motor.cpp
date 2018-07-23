/*
 * motor.cpp
 *
 *  Created on: Dec 15, 2017
 *      Author: robot
 */

#include "repository/motor.h"
#include "repository/joint.h"
#include "repository/joint_manager.h"
#include "foundation/cfg_reader.h"

#include <boost/algorithm/clamp.hpp>

namespace agile_robot {

struct MotorState {
  short position;
  short velocity;
  short torque;
};

struct MotorCommand {
  MotorCommand(const JntCmdType& _mode)
    : command(0), mode(_mode),
      MIN_POS(-10000), MAX_POS(10000),
      MIN_VEL(-5000),  MAX_VEL(5000) {
    ;
  }

  short             command;

  const JntCmdType& mode;
  /*const*/ short   MIN_POS;
  /*const*/ short   MAX_POS;
  /*const*/ short   MIN_VEL;
  /*const*/ short   MAX_VEL;
};

Motor::Motor()
  : Label("motor"), state_(nullptr),
    cmd_(nullptr),  new_command_(false) {
  ;
}

bool Motor::auto_init() {
  state_ = new MotorState;
  cmd_   = new MotorCommand(JointManager::instance()->getJointCommandMode());

  auto cfg = CfgReader::instance();
  cfg->get_value(getLabel(), "name", name_);

  std::vector<short> lims;
  cfg->get_value(getLabel(), "vel_limits", lims);
  if (2 != lims.size()) {
    LOG_WARNING << "You should fill the limits of the motor velocity.";
    cmd_->MIN_VEL = -5000;
    cmd_->MAX_VEL = 5000;
  } else {
    cmd_->MIN_VEL = lims[0];
    cmd_->MAX_VEL = lims[1];
  }

  return true;
}

Motor::~Motor() {
  if (state_) delete state_;
  if (cmd_)   delete cmd_;
  state_ = nullptr;
  cmd_   = nullptr;
}

void Motor::updateMotorCommand(double v) {
  switch (cmd_mode()) {
  case JntCmdType::CMD_MOTOR_VEL:
    cmd_->command = boost::algorithm::clamp(v,
                                  cmd_->MIN_VEL,
                                  cmd_->MAX_VEL);
    // motor_cmd_->command = v;
    new_command_ = true;
    // LOG_INFO << "Motor[" << motor_name() << "]: " << v;
    break;
  default:
    LOG_ERROR << "What fucking joint command mode. The current mode is "
      << cmd_->mode << "!!!!";
    return;
  }
}

const std::string&  Motor::motor_name() const {
  return name_;
}

const JntCmdType& Motor::cmd_mode()   const {
  return cmd_->mode;
}

///! About the state of motor
const short  Motor::motor_position() const {
  return state_->position;
}

const short& Motor::motor_position_const_ref() const {
  return state_->position;
}

const short* Motor::motor_position_const_pointer() const {
  return &state_->position;
}


const short  Motor::motor_velocity() const {
  return state_->velocity;
}

const short& Motor::motor_velocity_const_ref() const {
  return state_->velocity;
}

const short* Motor::motor_velocity_const_pointer() const {
  return &state_->velocity;
}


const short  Motor::motor_torque() const {
  return state_->torque;
}

const short& Motor::motor_torque_const_ref() const {
  return state_->torque;
}

const short* Motor::motor_torque_const_pointer() const {
  return &state_->torque;
}


///! About the command of motor
const short  Motor::motor_command() const {
  return cmd_->command;
}

const short& Motor::motor_command_const_ref() const {
  return cmd_->command;
}

const short* Motor::motor_command_const_pointer() const {
  return &cmd_->command;
}

void Motor::updateMotorPosition(short s) {
  state_->position = s;
}

void Motor::updateMotorVelocity(short s) {
  state_->velocity = s;
}

void Motor::updateMotorTorque(short s) {
  state_->torque   = s;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::Motor, Label)
