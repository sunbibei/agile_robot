/*
 * sw_node_manager.cpp
 *
 *  Created on: Sep 8, 2017
 *      Author: bibei
 */

#include "platform/sw_node/sw_node_manager.h"
#include <iomanip>

namespace agile_robot {

SINGLETON_IMPL(SWNodeManager)

SWNodeManager::SWNodeManager()
  : internal::ResourceManager<SWNode>() {

}

SWNodeManager::~SWNodeManager() {
}

bool SWNodeManager::init() {
  if (res_list_.empty()) {
    LOG_WARNING << "There are no any hardware push to the HwManager";
    return false;
  }

  LOG_DEBUG << "Start to order the hw_unit, in total " << res_list_.size();
  // We have any idea about the maxium id, so we use the maxium value about unsigned char.
  hw_list_by_id_.resize(MAX_NODE_NUM);
  hw_list_by_cmd_.reserve(res_list_.size());

  for (auto hw : res_list_) {
    hw_list_by_id_[hw->node_id_] = hw;
    hw_list_by_name_.insert(std::make_pair(hw->getLabel().c_str(), hw));
    if (hw->requireCmdDeliver()) {
      hw_list_by_cmd_.push_back(hw);
    }
  }

  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "\tNAME\t\tADDR\t\tNODE_ID\tCMD";
    for (auto hw : res_list_) {
      LOG_INFO << hw->getLabel() << "\t" << hw << "\t0x"
          << std::setw(2) << std::setfill('0') << std::hex << (int)hw->node_id_
          << "\t" << (hw->requireCmdDeliver()?"YES":"NO");
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }

  return true;
}

void SWNodeManager::handleMsg(const std::vector<Packet>& pkts) {
  for (const auto& pkt : pkts) {
    if ((hw_list_by_id_.size() <= pkt.node_id)
      || (nullptr == hw_list_by_id_[pkt.node_id])) {
      LOG_ERROR << "What fucking message!(" << (int)pkt.node_id << "/"
        << hw_list_by_id_.size() << "), Address: 0x" << hw_list_by_id_[pkt.node_id];
      continue;
    }
    hw_list_by_id_[pkt.node_id]->handleMsg(pkt);
  }
}

void SWNodeManager::generateCmd(std::vector<Packet>& pkts) {
  for (auto& node : hw_list_by_cmd_) {
    if (node->generateCmd(pkts))
      ; //LOG_DEBUG << "SW Node: " << node->getLabel() << " generate the command.";
  }
}

} /* namespace middleware */
