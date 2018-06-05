/*
 * sw_node_manager.cpp
 *
 *  Created on: Sep 8, 2017
 *      Author: bibei
 */

#include "platform/sw_node/sw_node_manager.h"
#include <iomanip>

#include "toolbox/time_control.h"

namespace agile_robot {

SINGLETON_IMPL(SWNodeManager)

//size_t counter = 0;
//TimeControl* timer = nullptr;
//std::vector<int64_t> freqs;
SWNodeManager::SWNodeManager()
  : internal::ResourceManager<SWNode>() {

}

SWNodeManager::~SWNodeManager() {
//  LOG_WARNING << "Last occur the error message has lapsed: " << timer->dt();
//  LOG_WARNING << "Lapse: " << timer->span()
//      << ", Count: " << counter;
//  delete timer;
//
//  counter = 0;
//  for (const auto& dt : freqs) {
//    printf("%3ld ", dt);
//    if (0 == counter++%20) std::cout << std::endl;
//  }
//  std::cout << std::endl;
}

bool SWNodeManager::init() {
  if (res_list_.empty()) {
    LOG_WARNING << "There are no any hardware push to the HwManager";
    return false;
  }

  LOG_DEBUG << "Start to order the hw_unit, in total " << res_list_.size();
  // We have any idea about the maxium id, so we use the maxium value about unsigned char.
  hw_list_by_id_.resize(MAX_BUS_NUM);
  for (auto& bus : hw_list_by_id_)
    bus.resize(MAX_NODE_NUM);

  hw_list_by_cmd_.reserve(res_list_.size());

  for (auto hw : res_list_) {
    hw_list_by_id_[hw->bus_id_][hw->node_id_] = hw;
    hw_list_by_name_.insert(std::make_pair(hw->getLabel().c_str(), hw));
    if (hw->requireCmdDeliver()) {
      hw_list_by_cmd_.push_back(hw);
    }
  }

  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "\tNAME\tBUS_ID\tNODE_ID\tCMD\tADDR";
    for (auto hw : res_list_) {
      LOG_INFO << hw->getLabel() << "\t0x"
          << std::setw(2) << std::setfill('0') << std::hex << (int)hw->bus_id_
          << "\t" << std::setw(2) << std::setfill('0') << std::hex << (int)hw->node_id_
          << "\t" << (hw->requireCmdDeliver()?"YES":"NO") << "\t" << hw;
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }

//  timer = new TimeControl(true);
//  freqs.reserve(1024*16);
  return true;
}

void SWNodeManager::handleMsg(const std::vector<Packet>& pkts) {
  for (const auto& pkt : pkts) {
//    if (true) {
//      std::cout << "[" << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__  << " ]";
//      printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//        (int)pkt.node_id,
//        (int)pkt.msg_id,  (int)pkt.size,
//        (int)pkt.data[0], (int)pkt.data[1],
//        (int)pkt.data[2], (int)pkt.data[3],
//        (int)pkt.data[4], (int)pkt.data[5],
//        (int)pkt.data[6], (int)pkt.data[7]);
//    }

    if ( (pkt.node_id >= MAX_NODE_NUM) || (pkt.bus_id >= MAX_BUS_NUM) ) {
      LOG_ERROR << "What fucking message! Out of bound for bus or node."
          << "BUS ID=" << (int)pkt.bus_id << ", NODE ID=" << (int)pkt.node_id;
      std::cout << "[" << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__  << " ]";
      printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        (int)pkt.node_id,
        (int)pkt.msg_id,  (int)pkt.size,
        (int)pkt.data[0], (int)pkt.data[1],
        (int)pkt.data[2], (int)pkt.data[3],
        (int)pkt.data[4], (int)pkt.data[5],
        (int)pkt.data[6], (int)pkt.data[7]);
//      freqs.push_back(timer->dt());
//      ++counter;
      continue;
    }
    if (nullptr == hw_list_by_id_[pkt.bus_id][pkt.node_id]) {
      LOG_ERROR << "What fucking message! BUS ID=" << (int)pkt.bus_id
          << ", NODE ID=" << (int)pkt.node_id << ", SW NODE: nullptr;";
      std::cout << "[" << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__  << " ]";
      printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        (int)pkt.node_id,
        (int)pkt.msg_id,  (int)pkt.size,
        (int)pkt.data[0], (int)pkt.data[1],
        (int)pkt.data[2], (int)pkt.data[3],
        (int)pkt.data[4], (int)pkt.data[5],
        (int)pkt.data[6], (int)pkt.data[7]);
//      freqs.push_back(timer->dt());
//      ++counter;
      ///! try to the others node whether handle this package.
      for (auto& node : res_list_) {
        node->handleMsg(pkt);
      }
      continue;
    }
    ///! handle the package.
    hw_list_by_id_[pkt.bus_id][pkt.node_id]->handleMsg(pkt);
  }
}

void SWNodeManager::generateCmd(std::vector<Packet>& pkts) {
  for (auto& node : hw_list_by_cmd_) {
    if (node->generateCmd(pkts))
      ; //LOG_DEBUG << "SW Node: " << node->getLabel() << " generate the command.";
  }
}

} /* namespace middleware */
