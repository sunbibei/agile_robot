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
  : Label("motor"), joint_handle_(nullptr), motor_state_(nullptr),
    motor_cmd_(nullptr), new_command_(false) {
  ;
}

bool Motor::auto_init() {
  auto cfg = CfgReader::instance();
  std::string _x, jnt_tag;
  Label::split_label(getLabel(), jnt_tag, _x);
  joint_handle_ = Label::getHardwareByName<Joint>(jnt_tag);
  if (nullptr == joint_handle_) {
    LOG_ERROR << "Can't get the handle of joint in the " << getLabel();
    return false;
  }
  joint_handle_->joint_motor_ = this;

  motor_state_ = new MotorState;
  motor_cmd_   = new MotorCommand(JointManager::instance()->getJointCommandMode());
  std::vector<short> lims;
  cfg->get_value(getLabel(), "vel_limits", lims);
  if (2 != lims.size()) {
    LOG_WARNING << "You should fill the limits of the motor velocity.";
    motor_cmd_->MIN_VEL = -5000;
    motor_cmd_->MAX_VEL = 5000;
  } else {
    motor_cmd_->MIN_VEL = lims[0];
    motor_cmd_->MAX_VEL = lims[1];
  }


  cfg->get_value(getLabel(), "name", motor_name_);
  return true;
}

Motor::~Motor() {
  if (motor_state_) delete motor_state_;
  if (motor_cmd_)   delete motor_cmd_;
  motor_state_ = nullptr;
  motor_cmd_   = nullptr;
}

void Motor::updateMotorCommand(double v) {
  switch (cmd_mode()) {
  case JntCmdType::CMD_MOTOR_VEL:
    motor_cmd_->command = boost::algorithm::clamp(v,
                                  motor_cmd_->MIN_VEL,
                                  motor_cmd_->MAX_VEL);
    // motor_cmd_->command = v;
    new_command_ = true;
    // LOG_INFO << "Motor[" << motor_name() << "]: " << v;
    break;
  default:
    LOG_ERROR << "What fucking joint command mode. The current mode is "
      << motor_cmd_->mode << "!!!!";
    return;
  }
}

const std::string&  Motor::motor_name() const {
  return joint_handle_->joint_name();
}

const std::string&  Motor::joint_name() const {
  return joint_handle_->joint_name();
}

const JntType&    Motor::joint_type() const {
  return joint_handle_->joint_type();
}

const LegType&    Motor::leg_type()   const {
  return joint_handle_->leg_type();
}

const JntCmdType& Motor::cmd_mode()   const {
  return motor_cmd_->mode;
}

///! About the state of motor
const short  Motor::motor_position() const {
  return motor_state_->position;
}

const short& Motor::motor_position_const_ref() const {
  return motor_state_->position;
}

const short* Motor::motor_position_const_pointer() const {
  return &motor_state_->position;
}


const short  Motor::motor_velocity() const {
  return motor_state_->velocity;
}

const short& Motor::motor_velocity_const_ref() const {
  return motor_state_->velocity;
}

const short* Motor::motor_velocity_const_pointer() const {
  return &motor_state_->velocity;
}


const short  Motor::motor_torque() const {
  return motor_state_->torque;
}

const short& Motor::motor_torque_const_ref() const {
  return motor_state_->torque;
}

const short* Motor::motor_torque_const_pointer() const {
  return &motor_state_->torque;
}


///! About the command of motor
const short  Motor::motor_command() const {
  return motor_cmd_->command;
}

const short& Motor::motor_command_const_ref() const {
  return motor_cmd_->command;
}

const short* Motor::motor_command_const_pointer() const {
  return &motor_cmd_->command;
}

void Motor::updateMotorPosition(short s) {
  motor_state_->position = s;
}

void Motor::updateMotorVelocity(short s) {
  motor_state_->velocity = s;
}

void Motor::updateMotorTorque(short s) {
  motor_state_->torque   = s;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::Motor, Label)
