/*
 * hw_manager.cpp
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#include "platform/propagate/propagate_manager.h"
#include "platform/sw_node/sw_node_manager.h"
#include "platform/master.h"

#include "foundation/thread/threadpool.h"
#include "foundation/utf.h"

namespace agile_robot {

const size_t MAX_PKTS_SIZE = 512;

SINGLETON_IMPL(Master)

Master::Master()
: /*tick_interval_(1), */thread_alive_(false),
  propagate_manager_(PropagateManager::create_instance()),
  sw_node_manager_(SWNodeManager::create_instance()) {
  queue_4_w_.reserve(MAX_PKTS_SIZE);
  queue_4_r_.reserve(MAX_PKTS_SIZE);
}

Master::~Master() {
  thread_alive_ = false;

  PropagateManager::destroy_instance();
  SWNodeManager::destroy_instance();

  propagate_manager_ = nullptr;
  sw_node_manager_   = nullptr;
}

bool Master::init(int mr, int mw, int pr, int pw) {
  if (!propagate_manager_ || !sw_node_manager_) return false;

  w_interval_ = std::chrono::microseconds((int)(1000000.0 / mw));
  r_interval_ = std::chrono::microseconds((int)(1000000.0 / mr));
  return propagate_manager_->init(pw, pr) && sw_node_manager_->init();
}

bool Master::run() {
  if (_DEBUG_INFO_FLAG) {
    propagate_manager_->print();
    sw_node_manager_->print();
  }
  // LOG_DEBUG << "<<==========Master::run==========";
  if (!propagate_manager_->run()) {
    LOG_WARNING << "PropagateManager::run fail!";
    return false;
  }
  // LOG_DEBUG << "Starting PropagateManager";

  thread_alive_ = true;
  ThreadPool::instance()->add("master-w", &Master::tick_w, this);
  ThreadPool::instance()->add("master-r", &Master::tick_r, this);
  // LOG_DEBUG << "==========Master::run==========>>";
  return true;
}

void Master::tick_r() {
  TICKER_INIT(std::chrono::microseconds);

  while (thread_alive_) {
    // The manager delivers each packet which read from Propagate for hardware update.
    queue_4_r_.clear();
    propagate_manager_->readPackets(queue_4_r_);
    // LOG_INFO << "read: " << queue_4_r_.size();
    // just for debug
//    if (false)
//    for (auto& pkt : queue_4_r_) {
//      printf(" <- NODE ID:0x%02X MSG ID: 0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//            (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
//            (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
//            (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
//    }

    sw_node_manager_->handleMsg(queue_4_r_);

    TICKER_CONTROL(r_interval_, std::chrono::microseconds);
  }
}

void Master::tick_w() {
  TICKER_INIT(std::chrono::microseconds);

  while (thread_alive_) {
    // Collecting all of the new command to control the robot
    queue_4_w_.clear();
    sw_node_manager_->generateCmd(queue_4_w_);
    // just for debug
//    if (false)
//    for (auto& pkt : queue_4_w_) {
//      printf(" -> NODE ID:0x%02X MSG ID: 0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//            (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
//            (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
//            (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
//    }

    // TODO
    propagate_manager_->writePackets(queue_4_w_);
    TICKER_CONTROL(w_interval_, std::chrono::microseconds);
  }
}

} /* namespace middleware */
