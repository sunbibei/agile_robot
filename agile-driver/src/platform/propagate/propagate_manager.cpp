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
#define MUTEX_UNLOCK(locker)      locker.unlock();

const size_t MAX_QUEUE_SIZE = 1024;

SINGLETON_IMPL(PropagateManager)

PropagateManager::PropagateManager()
  : internal::ResourceManager<Propagate>(),
    /*propa_interval_(1), */thread_alive_(true) {

  propa_list_by_bus_.resize(MAX_BUS_NUM);
  pkts_queue_4_send_.reserve(MAX_QUEUE_SIZE);
  pkts_queue_4_recv_.reserve(MAX_QUEUE_SIZE);
}

PropagateManager::~PropagateManager() {
  thread_alive_ = false;
//  ThreadPool::instance()->stop(THREAD_R_NAME);
//  ThreadPool::instance()->stop(THREAD_W_NAME);
  for (auto& c : propa_list_by_bus_) {
    if (nullptr == c) continue;

    c->stop();
    c.reset();
  }
}

bool PropagateManager::init(int r_freq, int w_freq) {
  // LOG_WARNING << "SIZE: " << res_list_.size();
  for (auto& cc : res_list_) {
    auto c = Label::getHardwareByName<Propagate>(cc->getLabel());
    // LOG_WARNING << "GOT: " << cc << " " << cc->getLabel();
    propa_list_by_bus_[c->bus_id_] = c;
  }

  w_interval_ = std::chrono::microseconds((int)(1000000.0 / w_freq));
  r_interval_ = std::chrono::microseconds((int)(1000000.0 / r_freq));

  bool all_fail = true;
  for (auto& c : propa_list_by_bus_) {
    if (nullptr == c) continue;
    // for (auto c = begin(); c != end(); ++c) {
    LOG_DEBUG << c->propa_name_ << " is starting.";
    if (!c->start()) {
      LOG_ERROR << "The propagate '" << c->propa_name_ << "' starting FAIL.";
    } else {
      all_fail = false;
      LOG_DEBUG << "The propagate '" << c->propa_name_ << "' has started.";
    }
  }

  // LOG_WARNING << "RETURN: " << (!all_fail ? "true" : "false");
  return !all_fail;
}

void PropagateManager::print() {
  size_t N_max_label = 0;
  for (const auto& res : res_list_) {
    if (N_max_label < res->propa_name_.size())
      N_max_label = res->propa_name_.size();
  }

  char format[128] = {0};
//printf("No. NAME       BUS-ID  ADDR   LABEL\n");
//printf("1   fake_propa  0x01 0xdbe4f0 leg.propa.fake_propa\n");
//printf("%3d %Xs  0x%02X %p %s\n");
  printf("\n");
  LOG_WARNING << "\nPropagate's table, size = " << res_list_.size()
      << "\n-------------------------------------------------------------";
  sprintf(format, "No. %%-%lds BUS-ID   ADDR   LABEL\n", N_max_label);
  printf(format, "NAME");

  memset(format, 0x00, 128);
  sprintf(format, "%%-3d %%-%lds  0x%%02X %%p %%s\n", N_max_label);

  int count = 0;
  for (const auto& res : res_list_) {
    printf(format, count++, res->propa_name_.c_str(), (int)res->bus_id_,
        res, res->label_.c_str());
  }
  LOG_WARNING;
  printf("\n");
}

bool PropagateManager::run() {
  // LOG_DEBUG << "<<==========PropagateManager::run==========";
  ThreadPool::instance()->add("propa-r", &PropagateManager::updateRead,  this);
  ThreadPool::instance()->add("propa-w", &PropagateManager::updateWrite, this);
  return true;
}

void PropagateManager::updateRead() {
  TICKER_INIT(std::chrono::microseconds);

  while (thread_alive_) {
    MUTEX_TRY_LOCK(lock_4_recv_)
    for (auto& c : res_list_) {
      Packet pkt;
      if (c->read(pkt)) {
//        printf("[propagate_manager.cpp: %d] NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//          __LINE__,         (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
//          (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
//          (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);

        // pkts_queue_4_recv_->push(pkt);
        pkts_queue_4_recv_.push_back(pkt);
      }
    }
    MUTEX_UNLOCK(lock_4_recv_)

    TICKER_CONTROL(r_interval_, std::chrono::microseconds);
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

    TICKER_CONTROL(w_interval_, std::chrono::microseconds);
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
