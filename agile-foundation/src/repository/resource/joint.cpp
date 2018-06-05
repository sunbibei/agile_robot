/*
 * joint.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#include "foundation/cfg_reader.h"
#include "foundation/utf.h"
#include "repository/resource/motor.h"
#include "repository/resource/joint.h"
#include "repository/resource/joint_manager.h"

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
    leg_type_(LegType::UNKNOWN_LEG), jnt_name_(""), joint_motor_(nullptr),/*msg_id_(INVALID_BYTE),*/
    joint_state_(nullptr), joint_command_(nullptr) {
  // The code as follow should be here.
  // JointManager::instance()->add(this);
}

Joint::~Joint() {
  if (nullptr != joint_state_) {
    delete joint_state_;
    joint_state_ = nullptr;
  }
  if (nullptr != joint_command_) {
    delete joint_command_;
    joint_command_ = nullptr;
  }
}

bool Joint::auto_init() {
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "jnt",  jnt_type_);
  cfg->get_value_fatal(getLabel(), "leg",  leg_type_);
  cfg->get_value_fatal(getLabel(), "name", jnt_name_);
  JointManager::instance()->add(this);

  joint_state_   = new JointState();

  double pos_min, pos_max;
  std::vector<double> limits;
  cfg->get_value_fatal(getLabel(), "limits", limits);
  if (limits.size() < 2) {
    LOG_WARNING << "The attribute of " << getLabel() << " is wrong!"
        << "The attribute of limits should be equal to two(min, max).";
    pos_min = -100;
    pos_max = 100;
  } else {
    pos_min = std::min(limits[0], limits[1]);
    pos_max = std::max(limits[0], limits[1]);
  }
  joint_command_ = new JointCommand(pos_min, pos_max,
      JointManager::instance()->getJointCommandMode(), 0.5*pos_min+0.5*pos_max);
  return true;
}

const std::string& Joint::joint_name() const { return jnt_name_; }
const JntType&   Joint::joint_type()   const { return jnt_type_; }
const LegType&   Joint::leg_type()     const { return leg_type_; }

void Joint::updateJointPosition(double pos) {
  auto t0 = std::chrono::high_resolution_clock::now();
  auto duration = t0 - joint_state_->previous_time_;
  auto count = std::chrono::duration_cast<std::chrono::duration<double>>(duration).count();
  joint_state_->vel_ = (pos - joint_state_->pos_) / count;
  joint_state_->pos_ = pos;
//  std::cout << "TEST:" << pos << std::endl;

  // LOG_WARNING << jnt_name_ << ": " << joint_state_->pos_ << ", " << joint_state_->vel_;
}

void Joint::updateJointTorque(double tor) {
  joint_state_->tor_ = tor;

  // LOG_WARNING << jnt_name_ << ": " << joint_state_->pos_ << ", " << joint_state_->vel_;
}

double Joint::joint_position() const {
  return joint_state_->pos_;
}

const double& Joint::joint_position_const_ref() const {
  return joint_state_->pos_;
}

const double* Joint::joint_position_const_pointer() const {
  return &(joint_state_->pos_);
}

double Joint::joint_position_min() const {
  return joint_command_->MIN_POS_;
}

double Joint::joint_position_max() const {
  return joint_command_->MAX_POS_;
}

double Joint::joint_velocity() const {
  return joint_state_->vel_;
}

const double& Joint::joint_velocity_const_ref() const {
  return joint_state_->vel_;
}

const double* Joint::joint_velocity_const_pointer() const {
  return &(joint_state_->vel_);
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
  return joint_state_->tor_;
}

const double& Joint::joint_torque_const_ref() const {
  return joint_state_->tor_;
}

const double* Joint::joint_torque_const_pointer() const {
  return &(joint_state_->tor_);
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
  switch (joint_command_->mode_) {
  case JntCmdType::CMD_POS:
    joint_command_->command_[POS_CMD_IDX] = boost::algorithm::clamp(v,
                                    joint_command_->MIN_POS_,
                                    joint_command_->MAX_POS_);
    // LOG_INFO << "Joint[" << jnt_name_ << "]: " << joint_command_->command_[POS_CMD_IDX];
    new_command_ = true;
    break;
  case JntCmdType::CMD_MOTOR_VEL:
    joint_motor_->updateMotorCommand(v);
    break;
  case JntCmdType::CMD_VEL:
  case JntCmdType::CMD_TOR:
  case JntCmdType::CMD_POS_VEL:
    LOG_ERROR << "Mode not match! The current mode: " << joint_command_->mode_
        << ", but the given " << JntCmdType::CMD_POS << " command";
    return;
  default:
    LOG_ERROR << "What fucking joint command mode. The current mode is "
      << joint_command_->mode_ << "!!!!";
    return;
  }
}

void Joint::updateJointCommand(double v0, double v1) {
  if (JntCmdType::CMD_POS_VEL != joint_command_->mode_) {
    LOG_ERROR << "Mode not match! The current mode: " << joint_command_->mode_
        << ", but the given " << JntCmdType::CMD_POS_VEL << " command";
    return;
  }
  joint_command_->command_[POS_CMD_IDX] = boost::algorithm::clamp(v0,
                                  joint_command_->MIN_POS_,
                                  joint_command_->MAX_POS_);
  // joint_command_->command_[POS_CMD_IDX] = v0;
  joint_command_->command_[VEL_CMD_IDX] = v1;
  // LOG_DEBUG << "update joint(" << jnt_name_ << ") command: "
  //           << joint_command_->command_;
  new_command_ = true;
}

void Joint::stop() {
  switch (joint_command_->mode_) {
  case JntCmdType::CMD_MOTOR_VEL:
    joint_motor_->updateMotorCommand(0);
    break;
  case JntCmdType::CMD_POS_VEL:
    updateJointCommand(joint_state_->pos_, 0);
    break;
  default:
    break;
  }
}

double Joint::joint_command(size_t idx /*= POS_CMD_IDX*/) const {
  return joint_command_->command_[idx];
}

const double& Joint::joint_command_const_ref(size_t idx /*= POS_CMD_IDX*/) const {
  return joint_command_->command_[idx];
}

const double* Joint::joint_command_const_pointer() const {
  return joint_command_->command_;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::Joint, Label)
