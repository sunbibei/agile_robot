/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "foundation/cfg_reader.h"
#include "foundation/utf.h"
#include "repository/motor.h"
#include "repository/joint.h"
#include "repository/joint_manager.h"

#include <chrono>
#include <tinyxml.h>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/clamp.hpp>

namespace agile_robot {
struct JointState {
  JointState(double pos = 0, double vel = 0, double s = 0, double o = 0)
  : pos_(pos), vel_(vel), tor_(0)
  { };
  double pos_;
  double vel_;
  double tor_;

  std::chrono::high_resolution_clock::time_point previous_time_;
};

enum {
  POS_CMD_IDX = 0,
  VEL_CMD_IDX,
};

struct JointCommand {
  double*           command_;
  const JntCmdType& mode_;
  const double      MIN_POS_;
  const double      MAX_POS_;

  JointCommand(double min, double max, const JntCmdType& mode_ref, double cmd = 0)
    : /*id_(0), */command_(nullptr), mode_(mode_ref),
      MIN_POS_(min), MAX_POS_(max) {
    command_  = new double[2];
    *command_ = cmd;
  };
  ~JointCommand() {
    if (command_) delete[] command_;
    command_ = nullptr;
  }
};

Joint::Joint()
  : Label(Label::null), new_command_(false), jnt_type_(JntType::UNKNOWN_JNT),
    leg_type_(LegType::UNKNOWN_LEG), name_(""), motor_handle_(nullptr),/*msg_id_(INVALID_BYTE),*/
    state_(nullptr), command_(nullptr) {
  // The code as follow should be here.
  // JointManager::instance()->add(this);
}

Joint::~Joint() {
  if (nullptr != state_) {
    delete state_;
    state_ = nullptr;
  }
  if (nullptr != command_) {
    delete command_;
    command_ = nullptr;
  }
}

bool Joint::auto_init() {
  auto cfg = CfgReader::instance();
  cfg->get_value_fatal(getLabel(), "jnt",  jnt_type_);
  cfg->get_value_fatal(getLabel(), "leg",  leg_type_);
  cfg->get_value_fatal(getLabel(), "name", name_);

  state_   = new JointState();

  double pos_min, pos_max;
  std::vector<double> limits;
  cfg->get_value(getLabel(), "limits", limits);
  if (limits.size() < 2) {
    LOG_WARNING << "The attribute of " << getLabel() << " is wrong!"
        << "The attribute of limits should be equal to two(min, max).";
    pos_min = -100;
    pos_max = 100;
  } else {
    pos_min = std::min(limits[0], limits[1]);
    pos_max = std::max(limits[0], limits[1]);
  }
  command_ = new JointCommand(pos_min, pos_max,
      JointManager::instance()->getJointCommandMode(), 0.5*pos_min+0.5*pos_max);

  std::string label;
  if (CfgReader::instance()->get_value(getLabel(), "motor", label))
    motor_handle_ = Label::getHardwareByName<Motor>(label);

  JointManager::instance()->add(this);
  return true;
}

const std::string& Joint::joint_name() const { return name_; }
const JntType&     Joint::joint_type() const { return jnt_type_; }
const LegType&     Joint::leg_type()   const { return leg_type_; }

MiiPtr<class Motor> Joint::motor_handle() {
  return motor_handle_;
}

void Joint::motor_handle(MiiPtr<class Motor> handle) {
  motor_handle_ = handle;
}

void Joint::updateJointPosition(double pos) {
  auto t0 = std::chrono::high_resolution_clock::now();
  auto duration = t0 - state_->previous_time_;
  auto count = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
  state_->vel_ = (pos - state_->pos_) / count;
  state_->pos_ = pos;
//  std::cout << "TEST:" << pos << std::endl;

  // LOG_WARNING << jnt_name_ << ": " << joint_state_->pos_ << ", " << joint_state_->vel_;
}

void Joint::updateJointTorque(double tor) {
  state_->tor_ = tor;

  // LOG_WARNING << jnt_name_ << ": " << joint_state_->pos_ << ", " << joint_state_->vel_;
}

double Joint::joint_position() const {
  return state_->pos_;
}

const double& Joint::joint_position_const_ref() const {
  return state_->pos_;
}

const double* Joint::joint_position_const_pointer() const {
  return &(state_->pos_);
}

double Joint::joint_position_min() const {
  return command_->MIN_POS_;
}

double Joint::joint_position_max() const {
  return command_->MAX_POS_;
}

double Joint::joint_velocity() const {
  return state_->vel_;
}

const double& Joint::joint_velocity_const_ref() const {
  return state_->vel_;
}

const double* Joint::joint_velocity_const_pointer() const {
  return &(state_->vel_);
}

double Joint::joint_velocity_min() const {
  LOG_ERROR << "Call the 'joint_velocity_min' which has does not complemented.";
  return -10000.0;
}

double Joint::joint_velocity_max() const {
  LOG_ERROR << "Call the 'joint_velocity_max' which has does not complemented.";
  return 10000.0;
}

double Joint::joint_torque() const {
  return state_->tor_;
}

const double& Joint::joint_torque_const_ref() const {
  return state_->tor_;
}

const double* Joint::joint_torque_const_pointer() const {
  return &(state_->tor_);
}

double Joint::joint_torque_min() const {
  LOG_ERROR << "Call the 'joint_torque_min' which has does not complemented.";
  return -10000.0;
}

double Joint::joint_torque_max() const {
  LOG_ERROR << "Call the 'joint_torque_max' which has does not complemented.";
  return 10000.0;
}

// About joint command
void Joint::updateJointCommand(double v) {
  switch (command_->mode_) {
  case JntCmdType::CMD_POS:
    command_->command_[POS_CMD_IDX] = boost::algorithm::clamp(v,
                                    command_->MIN_POS_,
                                    command_->MAX_POS_);
    LOG_INFO << "Joint[" << name_ << "]: " << command_->command_[POS_CMD_IDX];
    new_command_ = true;
    break;
  case JntCmdType::CMD_MOTOR_VEL:
    motor_handle_->updateMotorCommand(v);
    break;
  case JntCmdType::CMD_VEL:
  case JntCmdType::CMD_TOR:
  case JntCmdType::CMD_POS_VEL:
    LOG_ERROR << "Mode not match! The current mode: " << command_->mode_
        << ", but the given " << JntCmdType::CMD_POS << " command";
    return;
  default:
    LOG_ERROR << "What fucking joint command mode. The current mode is "
      << command_->mode_ << "!!!!";
    return;
  }
}

void Joint::updateJointCommand(double v0, double v1) {
  if (JntCmdType::CMD_POS_VEL != command_->mode_) {
    LOG_ERROR << "Mode not match! The current mode: " << command_->mode_
        << ", but the given " << JntCmdType::CMD_POS_VEL << " command";
    return;
  }
  command_->command_[POS_CMD_IDX] = boost::algorithm::clamp(v0,
                                  command_->MIN_POS_,
                                  command_->MAX_POS_);
  // joint_command_->command_[POS_CMD_IDX] = v0;
  command_->command_[VEL_CMD_IDX] = v1;
  // LOG_DEBUG << "update joint(" << jnt_name_ << ") command: "
  //           << joint_command_->command_;
  new_command_ = true;
}

void Joint::stop() {
  switch (command_->mode_) {
  case JntCmdType::CMD_MOTOR_VEL:
    motor_handle_->updateMotorCommand(0);
    break;
  case JntCmdType::CMD_POS_VEL:
    updateJointCommand(state_->pos_, 0);
    break;
  default:
    break;
  }
}

double Joint::joint_command(size_t idx /*= POS_CMD_IDX*/) const {
  return command_->command_[idx];
}

const double& Joint::joint_command_const_ref(size_t idx /*= POS_CMD_IDX*/) const {
  return command_->command_[idx];
}

const double* Joint::joint_command_const_pointer() const {
  return command_->command_;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::Joint, Label)
