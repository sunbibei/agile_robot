/*
 * sync.cpp
 *
 *  Created on: Feb 7, 2018
 *      Author: bibei
 */

#include "foundation/internal/sync.h"
#include <iostream>
#include <string.h>

#define __LOG(str) \
  std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":" << __LINE__ \
            << " -> " << str << std::endl;

namespace internal {
///////////////////////////////////////////////////////////////////////////////
//////////////           The helper method define first   /////////////////////
///////////////////////////////////////////////////////////////////////////////
#define IPC_TYPE_TOSTRING(ipc) \
    ( ( (const char*[]){"UNKNOWN", "SHM", "MSG_QUEUE", "N_IPC"} )[(ipc) + 1] )

bool __init_key_map() {
  ///! First, try to find the memory.
  int _shm_id = shmget(KEY_MAP_OF_KEY, 0, 0);
  if (-1 == _shm_id) { // Could not found.
    _shm_id = shmget(KEY_MAP_OF_KEY, MAX_SHM_SIZE * sizeof(_PrivateKeyMap), IPC_CREAT | 0666);
    if (-1 == _shm_id) {
      __LOG("Can't initialize the sync shared memory '" << SYNC_IDENTIFY << "'");
      return false;
    } else {
      __LOG("Create the sync shared memory '" << SYNC_IDENTIFY << "'");
      __add_key_map(SYNC_IDENTIFY, KEY_MAP_OF_KEY, _shm_id, IpcType::IPC_SHM);
    }
  } else { // Found
    ; // Nothing to do here.
    LOG_INFO << "Found the sync shared memory '" << SYNC_IDENTIFY << "'";
  }

  return true;
}

bool __add_key_map(const std::string& _n, key_t _key, int _id, IpcType _type) {
  auto _addr = shmat(shmget(KEY_MAP_OF_KEY, 0, 0), NULL, 0);
  if (nullptr == _addr) return false;
  _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

  ///! initialize the structure.
  if (0 == SYNC_IDENTIFY.compare(_n)) {
    LOG_DEBUG << "Initialize the all of key_map";
    size_t off = 0;
    strcpy(_key_map[off].name, _n.c_str());
    _key_map[off].type   = IPC_SHM;
    _key_map[off].key    = _key;
    _key_map[off].id     = _id;
    _key_map[off].count  = 0;

    ++off;
    for (; off < MAX_SHM_SIZE; ++off) {
      memset(_key_map[off].name, 0x00, MAX_NAME_SIZE * sizeof(char));
      _key_map[off].key    = IPC_PRIVATE;
      _key_map[off].type   = UNKNOWN_IPC;
      _key_map[off].id     = -1;
      _key_map[off].count  = 0;
    }
  } else { ///! Add a new block.
    LOG_DEBUG << "Add a new key_map";
    for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
      if (IPC_PRIVATE == _key_map[offset].key) {
        LOG_DEBUG << "OK";
        strcpy(_key_map[offset].name, _n.c_str());
        _key_map[offset].key    = _key;
        _key_map[offset].type   = _type;
        _key_map[offset].id     = _id;
        _key_map[offset].count  = 0;
        break;
      }
    }
  }

  return true;
}

int  __get_count_key_map(const std::string& _n) {
  auto _addr = shmat(shmget(KEY_MAP_OF_KEY, 0, 0), NULL, 0);
  if (nullptr == _addr) return -1;
  _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

  for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
    if (0 == _n.compare(_key_map[offset].name)) {
      return _key_map[offset].count;
    }
  }

  ///! NO FOUND
  return -1;
}

bool __add_count_key_map(const std::string& _n) {
  auto _addr = shmat(shmget(KEY_MAP_OF_KEY, 0, 0), NULL, 0);
  if (nullptr == _addr) return false;
  _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

  for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
    if (0 == _n.compare(_key_map[offset].name)) {
      ++_key_map[offset].count;
      LOG_DEBUG << "The count of '" << _n << "' is " << _key_map[offset].count;
      return true;
    }
  }

  return false;
}

bool __sub_count_key_map(const std::string& _n) {
  auto _addr = shmat(shmget(KEY_MAP_OF_KEY, 0, 0), NULL, 0);
  if (nullptr == _addr) return false;
  _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

  for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
    if (0 == _n.compare(_key_map[offset].name)) {
      --_key_map[offset].count;
      LOG_DEBUG << "The count of '" << _n << "' is " << _key_map[offset].count;

      if (0 == _key_map[offset].count) {
        LOG_DEBUG << "remove the shared memory -> '" << _n << "'";
        if (-1 != _key_map[offset].id) {
          if (IPC_SHM == _key_map[offset].type)
            shmctl(_key_map[offset].id, IPC_RMID, NULL);
          else if (IPC_MSGQ == _key_map[offset].type)
            msgctl(_key_map[offset].id, IPC_RMID, NULL);
          else
            LOG_ERROR << "What fucking code."; // unreached code
        }
        memset(_key_map[offset].name, 0x00, MAX_NAME_SIZE * sizeof(char));
        _key_map[offset].key    = IPC_PRIVATE;
        _key_map[offset].type   = UNKNOWN_IPC;
        _key_map[offset].id     = -1;
        _key_map[offset].count  = 0;
      }
      for (size_t off = 1; off < MAX_SHM_SIZE; ++off)
        if (IPC_PRIVATE != _key_map[off].key) return true;

      LOG_DEBUG << "remove the shared memory -> '" << SYNC_IDENTIFY << "'";
      shmctl(shmget(KEY_MAP_OF_KEY, 0, 0), IPC_RMID, NULL);
      return true;
    }
  }

  return false;
}

key_t __find_ava_key(const std::string& _n, IpcType type) {
  auto _addr = shmat(shmget(KEY_MAP_OF_KEY, 0, 0), NULL, 0);
  if (nullptr == _addr) return false;
  _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

  for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
    if (0 == _n.compare(_key_map[offset].name)) {
      return _key_map[offset].key;
    }
  }

  for (key_t key = KEY_MAP_OF_KEY + 0x01; key < KEY_MAP_OF_KEY + MAX_SHM_SIZE; key += 0x01) {
    switch (type) {
    case IpcType::IPC_SHM:
      if (-1 == shmget(key, 0, 0))
        return key;

      break;
    case IpcType::IPC_MSGQ:
      if (-1 == msgget(key, IPC_EXCL))
        return key;

      break;
    default:
      LOG_ERROR << "What fucking code."; // unreached code
    }
  }

  return IPC_PRIVATE;
}

void __clear(IpcType type) {
  if ((IpcType::IPC_SHM == type) || (IpcType::N_IPC == type)) {
    for (key_t key = KEY_MAP_OF_KEY; key < KEY_MAP_OF_KEY + MAX_SHM_SIZE; key += 0x01) {
      int _shm_id = shmget(key, 0, 0);
      if (-1 != _shm_id) {
        LOG_INFO << "remove the shared memory: " << _shm_id;
        shmctl(_shm_id, IPC_RMID, NULL);
      }
    }
  }

  if ((IpcType::IPC_MSGQ == type) || (IpcType::N_IPC == type)) {
    int _shm_id = shmget(KEY_MAP_OF_KEY, 0, 0);
    if (-1 != _shm_id) {
      LOG_INFO << "remove the shared memory: " << _shm_id;
      shmctl(_shm_id, IPC_RMID, NULL);
    }

    for (key_t key = KEY_MAP_OF_KEY + 0x01; key < KEY_MAP_OF_KEY + MAX_SHM_SIZE; key += 0x01) {
      int _msq_id = msgget(key, IPC_EXCL);
      if (-1 != _msq_id) {
        LOG_INFO << "remove the message queue: " << _msq_id;
        msgctl(_msq_id, IPC_RMID, 0);
      }
    }
  }
}

void __print_all_ipc() {
  LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  LOG_WARNING << "NAME\t\tTYPE\t\tKEY\t\tUSING";
  auto _addr = shmat(shmget(KEY_MAP_OF_KEY, 0, 0), NULL, 0);
  if (nullptr == _addr) return;
  _PrivateKeyMap* _key_map = (_PrivateKeyMap*) _addr;

  for (size_t offset = 0; offset < MAX_SHM_SIZE; ++offset) {
    if (IPC_PRIVATE != _key_map[offset].key) {
      LOG_INFO << _key_map[offset].name << "\t\t" << IPC_TYPE_TOSTRING(_key_map[offset].type)
          << "\t\t" << _key_map[offset].key << "\t\t" << _key_map[offset].count;
    }
  }
  LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
}

} /*end namespace internal */
