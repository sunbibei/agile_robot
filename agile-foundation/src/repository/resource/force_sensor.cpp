/*
 * touchdown.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: bibei
 */

#include "foundation/cfg_reader.h"
#include "repository/resource/force_sensor.h"

#include <boost/algorithm/string.hpp>
#include <tinyxml.h>

namespace middleware {

struct ForceState {
  ForceState(double d = 0) : data(0) {}
  double data;
};

ForceSensor::ForceSensor(const std::string& l)
  : Label(l),leg_type_(LegType::UNKNOWN_LEG),
    td_state_(new ForceState),
    scale_(0), offset_(0) {
  // The implement of init() should be here.
  ;
}

ForceSensor::~ForceSensor() {
  // Nothing to do here
  if (nullptr != td_state_) {
    delete td_state_;
    td_state_ = nullptr;
  }
}

bool ForceSensor::auto_init() {
  auto cfg = MiiCfgReader::instance();

  cfg->get_value(getLabel(), "scale",     scale_);
  cfg->get_value(getLabel(), "offset",    offset_);
  cfg->get_value_fatal(getLabel(), "leg", leg_type_);

  return true;
}

void ForceSensor::updateForceCount(short _count) {
  // if (LegType::HL == leg_type_) std::cout << " " << _count << std::endl;
  td_state_->data = _count; // * scale_ + offset_;
}

double ForceSensor::force_data() {
  return td_state_->data;
}

double* ForceSensor::force_data_const_pointer() {
  return &(td_state_->data);
}

const double& ForceSensor::force_data_const_ref() {
  return td_state_->data;
}

const LegType& ForceSensor::leg_type() const
{ return leg_type_; }

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::ForceSensor, Label)
