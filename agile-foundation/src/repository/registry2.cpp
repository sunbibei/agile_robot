/*
 * registry.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#include "repository/registry2.h"
#include "foundation/thread/threadpool.h"
#include "foundation/utf.h"

#include <chrono>
#include <iostream>

namespace agile_robot {

///! The id of type, easy to swap in shared memory.
enum DataType : char {
  DT_UNKONWN,
  DT_SHORT,
  DT_INT,
  DT_DOUBLE,
  DT_VEC_I,
  DT_VEC_D,
};

///! The information of registry, The structure must not be malloc by NEW,
///! and it is at the header of shared memory.
struct __RegInfo {
  DataType data_type;
  char     data_name[N_MAX_NAME];
  size_t   n_res; // in byte
  char     addr[0];
};

///! The information of resource, it is difference in the different process.
///! The @addr is the head address of the content of handle.
struct __ResStu {
  ResType    handle;
  void*      addr;
  __RegInfo* reg_info;
};

///! The tool function.
bool __get_res(std::map<std::string, __ResStu*>& res,
    const std::string& _n, ResType& vars) {
  LOG_INFO << "RES: " << (res.end() == res.find(_n));
  if (res.end() != res.find(_n)) {
    vars = res[_n]->handle;
    return true;
  }

  return false;
}

///! The tool function.
void __parseResType(const ResType& res, DataType& type, size_t& size, void*& addr) {
  if ( typeid(const short*) == res.type() ) {
    type = DataType::DT_SHORT;
    size = sizeof(short);
    addr = (void* ) boost::get<const short*>(res);
  } else if (typeid(const int*) == res.type()) {
    type = DataType::DT_INT;
    size = sizeof(int);
    addr = (void* ) boost::get<const int*>(res);
  } else if (typeid(const double*) == res.type()) {
    type = DataType::DT_DOUBLE;
    size = sizeof(double);
    addr = (void* ) boost::get<const double*>(res);
  } else if (typeid(const Eigen::VectorXi*) == res.type()) {
    type = DataType::DT_VEC_I;
    auto _res = boost::get<const Eigen::VectorXi*>(res);
    size = _res->size() * sizeof(int);
    addr = (void* ) _res->data();
  } else if (typeid(const Eigen::VectorXd*) == res.type()) {
    type = DataType::DT_VEC_D;
    auto _res = boost::get<const Eigen::VectorXd*>(res);
    size = _res->size() * sizeof(double);
    addr = (void* ) _res->data();
  } else {
    type = DataType::DT_UNKONWN;
    size = 0;
  }
}

SINGLETON_IMPL(Registry2)

const std::string RegDataName = "registry2-data";
const std::string SyncName    = "registry2-sync";
const char VALID_FLAG         = 0x88;
Registry2::Registry2()
  : shm_buffer_(nullptr), buff_top_(nullptr),
    thread_alive_(true) {
  if (nullptr == SharedMem::instance()) {
    SharedMem::create_instance();
  }

  auto shm_manager = SharedMem::instance();
  // Create the buffer for data
  shm_manager->create_shm(RegDataName, N_INIT_BUF);
  shm_buffer_ = (char* )shm_manager->get_addr_from_shm(RegDataName);
  buff_top_   = shm_buffer_;

  syncRegInfo();

  ///! Add the register support thread.
  ThreadPool::instance()->add("registry2", &Registry2::support, this);
}

Registry2::~Registry2() {
  thread_alive_ = false;
  SharedMem::destroy_instance();
}

void Registry2::syncRegInfo() {
  lock_.lock(); // For thread safety

  __RegInfo* info = (__RegInfo*) buff_top_;
  while (DataType::DT_UNKONWN != info->data_type && info->n_res > 0) {
    // record the reg_info
    reg_infos_.push_back(info);

    buff_top_ += (sizeof(__RegInfo) + info->n_res);
    info = (__RegInfo*) buff_top_;
  } // search and location the tail of buffer.

  lock_.unlock();
}

bool Registry2::publish(const std::string& _n, ResType _handle) {
  if (pub_origin_.end() != pub_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table.";
    return true;
  }
  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _n << " is fail!";
    return false;
  }

  insertRegInfo(pub_origin_, _n, _handle);
  return true;
}

bool Registry2::subscribe(const std::string& _n, ResType _handle) {
  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _n << " is fail!";
    return false;
  }

  if (sub_origin_.end() == sub_origin_.find(_n)) {
    insertRegInfo(sub_origin_, _n, _handle);
  } else {
    _handle = sub_origin_[_n]->handle;
  }

  return true;
}

void Registry2::insertRegInfo(std::map<std::string, class __ResStu*>& _o,
    const std::string& _n, ResType& _h) {
  // update the buff_top_;
  syncRegInfo();

  __RegInfo* info = nullptr;
  for ( const auto& _i : reg_infos_) {
    if (0 == strcmp(_i->data_name, _n.c_str())) {
      info = _i;
      break;
    }
  }

  __ResStu* res = addPuber(_n, _h, info);
  if (nullptr == res) {
    LOG_ERROR << "The information of registry does not match!";
    return;
  }

  _o[_n] = res;
}

__ResStu* Registry2::addPuber(const std::string& _n, ResType& _h, __RegInfo* _i) {
  DataType type = DataType::DT_UNKONWN;
  size_t   size = 0;
  void*    addr = nullptr;
  __parseResType(_h, type, size, addr);

  if (nullptr == _i) {
    lock_.lock();
    _i = (__RegInfo*) buff_top_;
    memcpy(_i->data_name, _n.c_str(), _n.size());
    _i->data_name[_n.size()] = '\0';
    _i->data_type = type;
    _i->n_res     = size;
    ///! Add into the registry list.
    reg_infos_.push_back(_i);
    // update the new top of buffer with the specify data.
    buff_top_ += (sizeof(__RegInfo) + _i->n_res);
    ((__RegInfo*) buff_top_)->data_type = DataType::DT_UNKONWN;
    ((__RegInfo*) buff_top_)->n_res     = 0;

    lock_.unlock();
  }
  if (type != _i->data_type || size != _i->n_res)
    return nullptr;

  __ResStu* res = new __ResStu;
  res->handle   = _h;
  res->addr     = addr;
  res->reg_info = _i;

  return res;
}

void Registry2::support() {
  TICKER_INIT(std::chrono::microseconds);

  thread_alive_ = true;
  while (thread_alive_) {
    for (const auto& pair : pub_origin_) {
      auto res = pair.second;
      if (nullptr != res->addr)
        memcpy(res->reg_info->addr, res->addr, res->reg_info->n_res);
    }

    for (const auto& pair : sub_origin_) {
      auto res = pair.second;
      if (nullptr != res->addr)
        memcpy(res->addr, res->reg_info->addr, res->reg_info->n_res);
    }

    TICKER_CONTROL(100, std::chrono::microseconds);
  }

}

std::string __getTypeName(const ResType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(const int*) == t.type()) {
    return "int     ";
  } else if (typeid(const double*) == t.type()) {
    return "double  ";
  } else if (typeid(const Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(const Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else {
    return t.type().name();
  }
}

///! print the all of registry.
void Registry2::print() {
  if (_DEBUG_INFO_FLAG) {
    syncRegInfo();

    LOG_WARNING;
    printf("The list of PUBLISH in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT   TYPE      ADDR   NAME\n");
 // printf("    0 vectorXd 0x1c65f10 test-res-d\n");
    int count = 0;
    for (const auto& l : pub_origin_) {
      ++count;
      printf("%5d %8s %8p %-31s\n", count,
          __getTypeName(l.second->handle).c_str(), l.second, l.first.c_str());
    }
    printf("___________________________________________\n");

    printf("The list of SUBSCRIBE in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT   TYPE      ADDR   NAME\n");
 // printf("    0 vectorXd 0x1c65f10 test-res-d\n");
    count = 0;
    for (const auto& l : sub_origin_) {
      ++count;
      printf("%5d %8s %8p %-31s\n", count,
          __getTypeName(l.second->handle).c_str(), l.second, l.first.c_str());
    }
    printf("___________________________________________\n");

    printf("The list of REGISTRY INFO in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT TYPE SIZE       ADDR     NAME\n");
  //printf("    1 0000 0000 0x7fc53c2af028 nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn\n");
    count = 0;
    for (const auto& i : reg_infos_) {
      ++count;
      printf("%5d %4d %4ld %p %-31s\n", count, i->data_type, i->n_res, i->addr, i->data_name);
    }
    printf("___________________________________________\n");
    LOG_WARNING;
  }
}

} /* namespace middleware */
