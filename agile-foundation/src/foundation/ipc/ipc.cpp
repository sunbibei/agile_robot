/*
 * ipc.cpp
 *
 *  Created on: Apr 16, 2018
 *      Author: bibei
 */

#include <foundation/ipc/ipc.h>
#include "foundation/internal/sync.h"

std::map<const std::string&, MiiIPC*> MiiIPC::s_ipcs_;

MiiIPC::MiiIPC(const std::string& _n)
  : ipc_name_(_n) {
  if (s_ipcs_.empty()) {
    internal::__init_key_map();
  }

  if (s_ipcs_.end() != s_ipcs_.find(ipc_name_))
    return;

  s_ipcs_.insert(std::make_pair(ipc_name_, this));
  internal::__add_count_key_map(ipc_name_);
}

MiiIPC::~MiiIPC() {
  ///! If register in the map of IPCs, erase it.
  auto itr = s_ipcs_.find(ipc_name_);
  if (s_ipcs_.end() != itr) {
    s_ipcs_.erase(itr);
  }
  internal::__sub_count_key_map(ipc_name_);
}

MiiIPC* MiiIPC::ipc_channel(const std::string& _n) {
  auto itr = s_ipcs_.find(_n);
  if (s_ipcs_.end() == itr) return nullptr;

  return itr->second;
}
