/*
 * hw_unit.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: silence
 */

#include "foundation/utf.h"
#include "foundation/cfg_reader.h"

#include "platform/sw_node/sw_node_manager.h"
#include "platform/sw_node/sw_node.h"

namespace agile_robot {

SWNode::SWNode(const std::string& l)
: Label(l), bus_id_(INVALID_BYTE), node_id_(INVALID_BYTE) {
  SWNodeManager::instance()->add(this);
}

bool SWNode::auto_init() {
  auto cfg = CfgReader::instance();
  cfg->get_value_fatal(getLabel(), "bus_id",  bus_id_);
  cfg->get_value_fatal(getLabel(), "node_id", node_id_);

  return true;
}

void SWNode::handleMsg(const Packet&) {
  ;
}

SWNode::~SWNode()                 { }
bool SWNode::generateCmd(std::vector<Packet>&) { return false;  }
bool SWNode::requireCmdDeliver()  { return true;   }

} /* namespace quadruped_robot_driver */


