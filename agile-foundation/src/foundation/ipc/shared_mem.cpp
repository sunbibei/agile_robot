/*
 * mii_shm.cpp
 *
 *  Created on: Feb 1, 2018
 *      Author: bibei
 */

#include "foundation/ipc/shared_mem.h"
#include "foundation/internal/sync.h"

using namespace internal;

SINGLETON_IMPL(SharedMem)

void SharedMem::clear() {
  __clear(IPC_TYPE::SHM);
}

void SharedMem::printAllSharedMem() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "Label's table, address " << &key_map_
        << ", size " << key_map_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "NAME\t\tKEY\t\tUSING\t\tADDR";
    auto _addr = get_addr_from_shm(SYNC_IDENTIFY);
    if (nullptr == _addr) return;
    _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

    for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
      if (IPC_PRIVATE != _key_map[offset].key) {
        auto _addr = addr_list_.find(_key_map[offset].name);
        LOG_INFO << _key_map[offset].name << "\t\t" << _key_map[offset].key
            << "\t\t" << _key_map[offset].count
            << "\t\t" << ((addr_list_.end() == _addr) ? "nullptr" : _addr->second);
      }
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
}

SharedMem::SharedMem() {
  __init_key_map();
}

SharedMem::~SharedMem() {
  ///! detach the memory from the current process.
  for (auto& mem : addr_list_)
    shmdt(mem.second);
  addr_list_.clear();

  for (const auto& itr : key_map_)
      __sub_count_key_map(itr.first);

  key_map_.clear();
}

bool SharedMem::create_shm(const std::string& _n, size_t _s) {
  if (key_map_.end() != key_map_.find(_n)) return true;

  key_t _key = __find_ava_key(_n, IPC_TYPE::SHM);
  ///! First, try to find the memory.
  int   _shm_id = shmget(_key, 0, 0);
  if (-1 == _shm_id) { // Could not found.
    _shm_id = shmget(_key, _s, IPC_CREAT | 0666);
    if (-1 == _shm_id) {
      LOG_ERROR << "Can't create the named shared memory '" << _n << "'";
      return false;
    } else {
      LOG_INFO << "Create the named shared memory '" << _n << "'";
      __add_key_map(_n, _key, _shm_id, IPC_TYPE::SHM);
    }
  } else { // Found
    ; // Nothing to do here.
    LOG_INFO << "Found the named shared memory '" << _n << "'";
  }

  key_map_.insert(std::make_pair(_n, _key));
  get_addr_from_shm(_n);
  return true;
}

void* SharedMem::get_addr_from_shm(const std::string& _n) {
  if (key_map_.end()   == key_map_.find(_n)) return nullptr;
  if (addr_list_.end() == addr_list_.find(_n)) {
    addr_list_[_n] = shmat(shmget(key_map_[_n], 0, 0), NULL, 0);

    __add_count_key_map(_n);
  }

  return addr_list_[_n];
}

#ifdef TEST_SHM

#include <iostream>

#include "shared_mem.h"

#include <thread>
#include <chrono>
#include <atomic>

// #define S

typedef struct {
  double pos;
  double vel;
  double tor;
} JntData;

void __printJntData(const JntData& d) {
  printf("pos -> %+04.02f, vel -> %+04.02f, tor -> %+04.02f\n",
      d.pos, d.vel, d.tor);
}

const std::string SHM_CMD_NAME = "jnt_cmd";
const std::string SHM_JNT_NAME = "jnt_sts";

int main() {
#ifdef CLEAR
  LOG_INFO << "Starting to clear shm";
  SharedMem::clear();
  return 0;
#endif
  LOG_INFO << "Starting to test shm";
#ifdef S
  LOG_INFO << "I'm SERVER";
#else
  LOG_INFO << "I'm CLIENT";
#endif

  auto shm = SharedMem::create_instance();
  if (nullptr == shm)
    LOG_FATAL << "What fucking... ...";

  char c = 'y';
  double cmd    = 0.0;
  JntData stats = {0, 0, 0};
  shm->create_shm(SHM_JNT_NAME, sizeof(JntData));
  shm->create_shm(SHM_CMD_NAME, sizeof(double));

  shm->printAllSharedMem();
#ifndef S
  TIMER_INIT
#endif
  while ('y' == c || 'Y' == c) {
#ifdef S
    shm->read_from_shm(SHM_JNT_NAME, stats);
     LOG_INFO << "The current joint states:";
     __printJntData(stats);

    LOG_INFO << "Input the command:";
    std::cin >> cmd;

    shm->write_to_shm(SHM_CMD_NAME,  cmd);
    LOG_INFO << "Continue?(y/Y)";
    std::cin >> c;
#else
    shm->read_from_shm(SHM_CMD_NAME, cmd);
    LOG_INFO << "The current command -> " << cmd;
    if (1111 == cmd) c = 'n';

    stats.pos += 0.1;
    stats.vel += 0.01;
    stats.tor += 0.001;
    shm->write_to_shm(SHM_JNT_NAME, stats);
    LOG_INFO << "The current joint states:";
    __printJntData(stats);
    TIMER_CONTROL(1000)
#endif
  }

  SharedMem::destroy_instance();
  return 0;
}
#endif
