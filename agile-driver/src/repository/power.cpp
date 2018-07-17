/*
 * power.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: bibei
 */

#include "repository/power.h"
#include "foundation/cfg_reader.h"

namespace agile_robot {

const double INVALID_CURRNET         = 0x88;
const size_t MAX_POWER_NODE_SIZE     = 3;
const size_t MAX_POWER_SUB_NODE_SIZE = 3;
const size_t MAX_SUB_POWER_SIZE      = 2;
const size_t MAX_CURR_TYPE_SIZE      = MAX_POWER_NODE_SIZE * (MAX_POWER_SUB_NODE_SIZE + 1) + MAX_SUB_POWER_SIZE + 1;


struct PowerInfor {
  double currents_[MAX_CURR_TYPE_SIZE];
};

struct PowerError {

};

struct PowerDescription {
  int power_node;
  int power_sub_node;
};

std::map<LegType, PowerDescription> LEG_IDX_MAP;

Power::Power(const std::string& _l)
    : Label(_l), power_infor_(nullptr), power_error_(nullptr) {
  ;
}

bool Power::auto_init() {
  // auto cfg = CfgReader::instance();

  /*LegType leg = LegType::UNKNOWN_LEG;
  int count = 0;
  MiiString _l = Label::make_label(getLabel(), "map_" + std::to_string(count));
  while (cfg->get_value(_l, "leg", leg)) {
    PowerDescription des;
    cfg->get_value(_l, "node_id", des.power_node);
    cfg->get_value(_l, "sub_node_id", des.power_sub_node);
    LEG_IDX_MAP.insert(std::make_pair(leg, des));
  }*/
  power_infor_ = new PowerInfor;
  power_error_ = new PowerError;
  return true;
}

Power::~Power() {
  if (nullptr != power_infor_) {
    delete power_infor_;
    power_infor_ = nullptr;
  }
  if (nullptr != power_error_) {
    delete power_error_;
    power_error_ = nullptr;
  }
}

double        Power::current(const LegType& leg) {
  auto itr = LEG_IDX_MAP.find(leg);
  if (LEG_IDX_MAP.end() == itr) return INVALID_CURRNET;
  auto nodes = itr->second;
  return power_infor_->currents_[nodes.power_node * MAX_POWER_SUB_NODE_SIZE + nodes.power_sub_node];
}

const double& Power::current_const_ref(const LegType& leg) {
  auto itr = LEG_IDX_MAP.find(leg);
  if (LEG_IDX_MAP.end() == itr) return INVALID_CURRNET;
  auto nodes = itr->second;
  return power_infor_->currents_[nodes.power_node * MAX_POWER_SUB_NODE_SIZE + nodes.power_sub_node];
}

const double* Power::current_const_pointer(const LegType& leg) {
  auto itr = LEG_IDX_MAP.find(leg);
  if (LEG_IDX_MAP.end() == itr) return nullptr;
  auto nodes = itr->second;
  return power_infor_->currents_ + (nodes.power_node * MAX_POWER_SUB_NODE_SIZE + nodes.power_sub_node);
}

void Power::updatePowerInfo(size_t w, double c) {
  if (w > MAX_CURR_TYPE_SIZE) return;
  power_infor_->currents_[w] = c;
  // if (w == 4) LOG_DEBUG << "Update Power InforXXX: " << w << " - " << c;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::Power, Label)
