/*
 * data_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include "foundation/cfg_reader.h"
#include "repository/registry.h"

#include "robot/leg/data_leg.h"


namespace qr_control {

DataLeg::DataLeg()
  : Label("robot-leg"), leg_type_(LegType::UNKNOWN_LEG),
    foot_force_(nullptr), jnt_mode_(nullptr),
    jnt_cmd_(nullptr), jnt_cmd_flag_(nullptr) {
  for (auto& c : jnt_pos_)
    c = nullptr;
}

bool DataLeg::auto_init() {
  auto cfg    = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "leg", leg_type_);

  MiiString tmp_str;
  cfg->get_value_fatal(getLabel(), "mode", tmp_str);
  jnt_mode_ = (JntCmdType*)GET_COMMAND_NO_FLAG(tmp_str, int*);

  cfg->get_value_fatal(getLabel(), "command", tmp_str);
  jnt_cmd_ = GET_COMMAND(tmp_str, &jnt_cmd_flag_, Eigen::VectorXd*);

  cfg->get_value_fatal(getLabel(), "tdlo", tmp_str);
  foot_force_  = GET_RESOURCE(tmp_str, const double*);

  cfg->get_value_fatal(getLabel(), "pos", tmp_str);
  jnt_pos_[JntDataType::POS] = GET_RESOURCE(tmp_str, const EVX*);

  cfg->get_value_fatal(getLabel(), "vel", tmp_str);
  jnt_pos_[JntDataType::VEL] = GET_RESOURCE(tmp_str, const EVX*);

  cfg->get_value_fatal(getLabel(), "tor", tmp_str);
  jnt_pos_[JntDataType::TOR] = GET_RESOURCE(tmp_str, const EVX*);

  if (false && _DEBUG_INFO_FLAG) {
    LOG_INFO << "get command (" << leg_type_ << "): " << jnt_cmd_;
    LOG_INFO << "get resource(JntDataType::TD /" << leg_type_ << "): " << foot_force_;
    LOG_INFO << "get resource(JntDataType::POS/" << leg_type_ << "): " << jnt_pos_[JntDataType::POS];
    LOG_INFO << "get resource(JntDataType::VEL/" << leg_type_ << "): " << jnt_pos_[JntDataType::VEL];
    LOG_INFO << "get resource(JntDataType::TOR/" << leg_type_ << "): " << jnt_pos_[JntDataType::TOR];
  }
  return true;
}

DataLeg::~DataLeg() {
  ; // Nothing to do here
}

LegType DataLeg::leg_type() { return leg_type_; }

double         DataLeg::foot_force()               const { return *foot_force_; }
const double&  DataLeg::foot_force_const_ref()     const { return *foot_force_; }
const double*  DataLeg::foot_force_const_pointer() const { return  foot_force_; }

EVX        DataLeg::joint_position()               const { return *jnt_pos_[JntDataType::POS]; }
const EVX& DataLeg::joint_position_const_ref()     const { return *jnt_pos_[JntDataType::POS]; }
const EVX* DataLeg::joint_position_const_pointer() const { return  jnt_pos_[JntDataType::POS]; }

EVX        DataLeg::joint_velocity()               const { return *jnt_pos_[JntDataType::VEL]; }
const EVX& DataLeg::joint_velocity_const_ref()     const { return *jnt_pos_[JntDataType::VEL]; }
const EVX* DataLeg::joint_velocity_const_pointer() const { return  jnt_pos_[JntDataType::VEL]; }

EVX        DataLeg::joint_torque()               const { return *jnt_pos_[JntDataType::TOR]; }
const EVX& DataLeg::joint_torque_const_ref()     const { return *jnt_pos_[JntDataType::TOR]; }
const EVX* DataLeg::joint_torque_const_pointer() const { return  jnt_pos_[JntDataType::TOR]; }

// Only get the last command.
EVX        DataLeg::joint_command()               const { return *jnt_cmd_; }
EVX&       DataLeg::joint_command_const_ref()     const { return *jnt_cmd_; }
EVX*       DataLeg::joint_command_const_pointer() const { return  jnt_cmd_; }

JntCmdType  DataLeg::joint_mode()               const { return *jnt_mode_; }
JntCmdType& DataLeg::joint_mode_const_ref()     const { return *jnt_mode_; }
JntCmdType* DataLeg::joint_mode_const_pointer() const { return  jnt_mode_; }

void DataLeg::joint_command(JntType jnt, double val) {
  if (JntType::UNKNOWN_JNT == jnt) return;

  if (JntType::N_JNTS == jnt)
    jnt_cmd_->fill(val);
  else
    (*jnt_cmd_)(jnt) = val;

  jnt_cmd_flag_->store(true);
}

void DataLeg::joint_command(const EVX& vals) {
  if (JntType::N_JNTS != vals.size()) return;

  *jnt_cmd_ = vals;
  jnt_cmd_flag_->store(true);
}

void DataLeg::joint_mode(JntCmdType mode) {
  *jnt_mode_ = mode;
}

} /* namespace qr_control */
