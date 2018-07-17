/*
 * power_node.cpp
 *
 *  Created on: Nov 14, 2017
 *      Author: bibei
 */

#include "platform/sw_node/power_node.h"
#include "foundation/cfg_reader.h"
#include "repository/power.h"

namespace agile_robot {

PowerNode::PowerNode(const std::string& __l)
  : SWNode(__l), power_info_(nullptr) {
  ;
}

bool PowerNode::auto_init() {
  if (!SWNode::auto_init()) return false;
  auto cfg = MiiCfgReader::instance();

  std::string power_label;
  cfg->get_value_fatal(getLabel(), "label", power_label);
  power_info_ = Label::getHardwareByName<Power>(power_label);
  if (nullptr == power_info_) {
    LOG_ERROR << "Can't get power object '" << power_label << "'";
  }

  return true;;
}

PowerNode::~PowerNode() {
  ;
}

void PowerNode::handleMsg(const Packet& pkt) {
  if ((pkt.node_id != node_id_) || (pkt.size != 2)) {
    LOG_ERROR << "Wrong match id between Packet and Joint";
    return;
  }

  short tmp;
  memcpy(&tmp, pkt.data, sizeof(short));
  power_info_->updatePowerInfo(pkt.msg_id, tmp/1000.0);

  /*switch (pkt.msg_id) {
  case MII_MSG_HEARTBEAT_MSG_1:
    // parse the joint state and touchdown data
    // updateFromBuf(pkt.data);
    break;
  case MII_MSG_HEARTBEAT_MSG_1:
    // parse the joint state and touchdown data
    // updateFromBuf(pkt.data);
    break;
  case MII_MSG_HEARTBEAT_MSG_1:
    // parse the joint state and touchdown data
    // updateFromBuf(pkt.data);
    break;
  default:
    SWNode::handleMsg(pkt);
  }*/
}

bool PowerNode::requireCmdDeliver() {
  return false;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::PowerNode, Label)
