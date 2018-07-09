/*
 * propagate_manager.cpp
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#include "platform/propagate/propagate_manager.h"
#include "foundation/thread/threadpool.h"

namespace agile_robot {

const size_t  MAX_BUS_NUM = 10;

#define MUTEX_TRY_LOCK(locker)    while (!locker.try_lock()) { };
#define MUTEX_UNLOCK(locker)  locker.unlock();

const std::string THREAD_R_NAME = "propagate-r";
const std::string THREAD_W_NAME = "propagate-w";
const size_t MAX_QUEUE_SIZE = 1024;
std::thread update_thread_;

SINGLETON_IMPL(PropagateManager)

PropagateManager::PropagateManager()
  : internal::ResourceManager<Propagate>(),
    /*propa_interval_(1), */thread_alive_(true) {

  propa_list_by_bus_.reserve(MAX_BUS_NUM);
  pkts_queue_4_send_.reserve(MAX_QUEUE_SIZE);
  pkts_queue_4_recv_.reserve(MAX_QUEUE_SIZE);
}

PropagateManager::~PropagateManager() {
  thread_alive_ = false;
  ThreadPool::instance()->stop(THREAD_R_NAME);
  ThreadPool::instance()->stop(THREAD_W_NAME);
  for (auto& c : res_list_) {
    c->stop();
  }
}

bool PropagateManager::init() {
  return true; // Nothing to do here.
}

bool PropagateManager::run() {
  if (ThreadPool::instance()->is_running(THREAD_R_NAME)
      || ThreadPool::instance()->is_running(THREAD_W_NAME)) {
    LOG_WARNING << "Call PropagateManager::run() twice!";
    return false;
  }
  // LOG_DEBUG << "<<==========PropagateManager::run==========";

  for (auto& c : res_list_) {
    propa_list_by_bus_[c->bus_id_] = c;
  }

  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_INFO << "The list of Propagate, size = " << res_list_.size();
    LOG_WARNING << "-----------------------------------------------------";
    for (size_t i = 0; i < res_list_.size(); ++i)
      LOG_INFO << i + 1 << ": " << res_list_[i]->propa_name_ << "\t"
          << res_list_[i]->getLabel() << "\t" << res_list_[i];
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
  bool all_fail = true;
  for (auto c : res_list_) {
    // for (auto c = begin(); c != end(); ++c) {
    // LOG_DEBUG << c->getLabel() << " is starting.";
    if (!c->start())
      LOG_ERROR << "The propagate '" << c->propa_name_ << "' starting FAIL.";
    else
      all_fail = false;
    //   LOG_DEBUG << "The propagate '" << c->propa_name_ << "' has started.";
  }
  if (all_fail && !_DEBUG_INFO_FLAG) return false;

  ThreadPool::instance()->add(THREAD_R_NAME, &PropagateManager::updateRead,  this);
  ThreadPool::instance()->add(THREAD_W_NAME, &PropagateManager::updateWrite, this);
  return true;
}

void PropagateManager::updateRead() {
  TICKER_INIT(std::chrono::microseconds);

  while (thread_alive_) {
    MUTEX_TRY_LOCK(lock_4_recv_)
    for (auto& c : res_list_) {
      Packet pkt;
      if (c->read(pkt)) {
//        if (false) {
//          printf("%s", (std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) + ":" + std::to_string(__LINE__)).c_str());
//          printf(" NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//            (int)pkt.node_id,
//            (int)pkt.msg_id,  (int)pkt.size,
//            (int)pkt.data[0], (int)pkt.data[1],
//            (int)pkt.data[2], (int)pkt.data[3],
//            (int)pkt.data[4], (int)pkt.data[5],
//            (int)pkt.data[6], (int)pkt.data[7]);
//        }

        // pkts_queue_4_recv_->push(pkt);
        pkts_queue_4_recv_.push_back(pkt);
      }
    }
    MUTEX_UNLOCK(lock_4_recv_)

    TICKER_CONTROL(200, std::chrono::microseconds);
  }
}

void PropagateManager::updateWrite() {
  TICKER_INIT(std::chrono::microseconds);

  while (thread_alive_) {
    MUTEX_TRY_LOCK(lock_4_send_)
    while (!pkts_queue_4_send_.empty()) {
      // pkts_queue_4_send_->pop(pkt);
      const auto& pkt = pkts_queue_4_send_.back();
      if (nullptr == propa_list_by_bus_[pkt.bus_id]) {
        for (auto& c : res_list_)
          if (c->write(pkt)) break;
      } else
        propa_list_by_bus_[pkt.bus_id]->write(pkt);

      pkts_queue_4_send_.pop_back();
//      if (false) {
//        // printf("%s", (std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) + ":" + std::to_string(__LINE__)).c_str());
//        printf(" -> NODE ID:0x%02X MSG ID: 0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//              (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
//              (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
//              (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
//      }
    }
    MUTEX_UNLOCK(lock_4_send_)

    TICKER_CONTROL(200, std::chrono::microseconds);
  }
}

bool PropagateManager::readPackets(std::vector<Packet>& pkts) {
  MUTEX_TRY_LOCK(lock_4_recv_)

  if (!pkts_queue_4_recv_.empty()) {
    // pkts.push_back(Packet());
    // pkts_queue_4_recv_->pop(pkts.back());
    for (const auto& pkt : pkts_queue_4_recv_)
      pkts.push_back(pkt);

    pkts_queue_4_recv_.clear();
  }
  MUTEX_UNLOCK(lock_4_recv_)
  return true;
}

bool PropagateManager::writePackets(const std::vector<Packet>& pkts) {
  MUTEX_TRY_LOCK(lock_4_send_)
  if (!pkts.empty()) {
    for (const auto& pkt : pkts)
      pkts_queue_4_send_.push_back(pkt);
  }
  MUTEX_UNLOCK(lock_4_send_)
  return true;
}

} /* namespace middleware */
