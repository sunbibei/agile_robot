/*
 * registry.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#include "foundation/registry/registry2.h"
#include "foundation/thread/threadpool.h"
#include "foundation/utf.h"

#include <chrono>
#include <iostream>

#define N_MAX_TNAME     (15)
#define OCCUPY_FLAG     (0x88)

///! cancel the namespaces
// namespace agile_robot {

///! The information of registry, The structure must not be malloc by NEW,
///! and it is at the header of shared memory.
struct __RegInfo {
  char     occupy;
  char     data_type[N_MAX_TNAME];
  char     data_name[N_MAX_NAME];

  size_t   n_res; // in byte
  char     addr[0];
};

///! The information of resource, it is difference in the different process.
///! The @addr is the head address of the content of handle.
struct __PubResStu {
  const void* addr;
  __RegInfo*  reg_info;
};

struct __SubResStu {
  void*       addr;
  __RegInfo*  reg_info;
};

SINGLETON_IMPL(Registry2)

const std::string RegDataName = "registry2-data";
const std::string SyncName    = "registry2-sync";
const char VALID_FLAG         = 0x88;
Registry2::Registry2()
  : shm_buffer_(nullptr), buff_top_(nullptr),
    thread_alive_(true) {
  if (nullptr == SharedMem::instance())
    LOG_FATAL << "YOU NEED CREATE THE instance of SharedMem firstly!";

  auto shm_manager = SharedMem::instance();
  // Create the buffer for data
  shm_manager->create_shm(RegDataName, N_INIT_BUF);
  shm_buffer_ = (char* )shm_manager->get_addr_from_shm(RegDataName);
  buff_top_   = shm_buffer_;

  ///! sync the reg_info
  syncRegInfo();

  ///! Add the register support thread.
  if (nullptr != ThreadPool::instance())
    ThreadPool::instance()->add("registry2", &Registry2::support, this);
}

Registry2::~Registry2() {
  thread_alive_ = false;

  for (auto& pub : pub_origin_) {
    delete pub.second;
    pub.second = nullptr;
  }
  pub_origin_.clear();
}

void Registry2::syncRegInfo() {
  lock_.lock(); // For thread safety

  __RegInfo* info = (__RegInfo*) buff_top_;
  while (OCCUPY_FLAG == info->occupy) {
    LOG_ERROR << info->n_res;
    // record the reg_info
    reg_infos_.push_back(info);

    buff_top_ += (sizeof(__RegInfo) + info->n_res);
    info = (__RegInfo*) buff_top_;
  } // search and location the tail of buffer.

  lock_.unlock();
}

bool Registry2::publish(const std::string& _n, const Eigen::VectorXd* ptr) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "vectorxd");
}

bool Registry2::publish(const std::string& _n, const Eigen::VectorXi* ptr) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "vectorxi");
}

bool Registry2::publish(const std::string& _n, const Eigen::MatrixXd* ptr) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "matrixxd");
}

bool Registry2::publish(const std::string& _n, const Eigen::MatrixXi* ptr) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "matrixxi");
}

bool Registry2::pub_helper(const std::string& _n, const void* addr, size_t size, const std::string& _tn) {
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
  LOG_ERROR << _n << ": " << size << ", " << _tn << ", " << addr;

  // update the buff_top_;
  syncRegInfo();

  __RegInfo* info = nullptr;
  for ( const auto& _i : reg_infos_) {
    if (0 == strcmp(_i->data_name, _n.c_str())) {
      info = _i;
      break;
    }
  }

  std::string tn = _tn.substr(0, std::min(_tn.size(), (size_t)(N_MAX_TNAME - 1)));
  LOG_ERROR << "tname: " << tn;
  if (nullptr == info) {
    lock_.lock();
    info = (__RegInfo*) buff_top_;
    memset(info, 0x00, (sizeof(__RegInfo) + size));
    memcpy(info->data_type, tn.c_str(), tn.size());
    memcpy(info->data_name, _n.c_str(), _n.size());
    info->n_res  = size;
    info->occupy = OCCUPY_FLAG;
    ///! Add into the registry list.
    reg_infos_.push_back(info);
    // update the new top of buffer with the specify data.
    buff_top_ += (sizeof(__RegInfo) + info->n_res);

    lock_.unlock();
  }

  if (0 != tn.compare(info->data_type) || size != info->n_res)
    return false;

  __PubResStu* res = new __PubResStu;
  res->addr        = addr;
  res->reg_info    = info;

  pub_origin_[_n] = res;
  return true;
}

bool Registry2::subscribe(const std::string& _n, Eigen::VectorXd* ptr) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "vectorxd");
}

bool Registry2::subscribe(const std::string& _n, Eigen::VectorXi* ptr) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "vectorxi");
}

bool Registry2::subscribe(const std::string& _n, Eigen::MatrixXd* ptr) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "matrixxd");
}

bool Registry2::subscribe(const std::string& _n, Eigen::MatrixXi* ptr) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "matrixxi");
}

bool Registry2::sub_helper(const std::string& _n, void* addr, size_t size, const std::string& _tn) {
  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _n << " is fail!";
    return false;
  }

  LOG_ERROR << _n << ": " << size << ", " << _tn << ", " << addr;

  LOG_ERROR << "1";
  // update the buff_top_;
  syncRegInfo();
  LOG_ERROR << "2";
  __RegInfo* info = nullptr;
  for ( const auto& _i : reg_infos_) {
    if (0 == strcmp(_i->data_name, _n.c_str())) {
      info = _i;
      break;
    }
  }

  LOG_ERROR << "3";
  std::string tn = _tn.substr(0, std::min(_tn.size(), (size_t)(N_MAX_TNAME - 1)));
  if (nullptr == info) {
    lock_.lock();
    info = (__RegInfo*) buff_top_;
    memset(info, 0x00, (sizeof(__RegInfo) + size));
    memcpy(info->data_type, tn.c_str(), tn.size());
    memcpy(info->data_name, _n.c_str(), _n.size());
    info->n_res  = size;
    info->occupy = OCCUPY_FLAG;

    ///! Add into the registry list.
    reg_infos_.push_back(info);
    // update the new top of buffer with the specify data.
    buff_top_ += (sizeof(__RegInfo) + info->n_res);
    ((__RegInfo*) buff_top_)->n_res = -1;
    LOG_ERROR << "4";
    lock_.unlock();
  }

  if (0 != tn.compare(info->data_type) || size != info->n_res)
    return false;

  __SubResStu* res = new __SubResStu;
  res->addr        = addr;
  res->reg_info    = info;

  sub_origin_[_n] = res;
  LOG_ERROR << "5";
  return true;
}

//void Registry2::insertRegInfo(std::map<std::string, class __ResStu2*>& _o,
//    const std::string& _n, ResType2& _h) {
//  // update the buff_top_;
//  syncRegInfo();
//
//  __RegInfo2* info = nullptr;
//  for ( const auto& _i : reg_infos2_) {
//    if (0 == strcmp(_i->data_name, _n.c_str())) {
//      info = _i;
//      break;
//    }
//  }
//
//  __ResStu2* res = addPuber(_n, _h, info);
//  if (nullptr == res) {
//    LOG_ERROR << "The information of registry does not match!";
//    return;
//  }
//
//  _o[_n] = res;
//}
//
//__ResStu2* Registry2::addPuber(const std::string& _n, ResType2& _h, __RegInfo2* _i) {
//  DataType type = DataType::DT_UNKONWN;
//  size_t   size = 0;
//  void*    addr = nullptr;
//  __parseResType(_h, type, size, addr);
//
//  if (nullptr == _i) {
//    lock_.lock();
//    _i = (__RegInfo2*) buff_top_;
//    memcpy(_i->data_name, _n.c_str(), _n.size());
//    _i->data_name[_n.size()] = '\0';
//    _i->data_type = type;
//    _i->n_res     = size;
//    ///! Add into the registry list.
//    reg_infos2_.push_back(_i);
//    // update the new top of buffer with the specify data.
//    buff_top_ += (sizeof(__RegInfo2) + _i->n_res);
//    ((__RegInfo2*) buff_top_)->data_type = DataType::DT_UNKONWN;
//    ((__RegInfo2*) buff_top_)->n_res     = 0;
//
//    lock_.unlock();
//  }
//  if (type != _i->data_type || size != _i->n_res)
//    return nullptr;
//
//  __ResStu2* res = new __ResStu2;
//  res->handle   = _h;
//  res->addr     = addr;
//  res->reg_info = _i;
//
//  return res;
//}

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

///! print the all of registry.
void Registry2::print() {
  syncRegInfo();

  printf("\n");
  LOG_WARNING;
  if (!pub_origin_.empty()) {
    printf("The list of PUBLISH in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT       TYPE         ADDR           NAME\n");
 // printf("    0 aaaaaaaaaaaaaaaa 0x7fc53c2af028ab test-res-d\n");
    int count = 0;
    for (const auto& l : pub_origin_) {
      ++count;
      printf("%5d %16s %16p %-31s\n", count,
          l.second->reg_info->data_type, l.second, l.first.c_str());
    }
    printf("___________________________________________\n");
  }

  if (!sub_origin_.empty()) {
    printf("The list of SUBSCRIBE in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT       TYPE         ADDR           NAME\n");
 // printf("    0 aaaaaaaaaaaaaaaa 0x7fc53c2af028ab test-res-d\n");
    int count = 0;
    for (const auto& l : sub_origin_) {
      ++count;
      printf("%5d %16s %16p %-31s\n", count,
          l.second->reg_info->data_type, l.second, l.first.c_str());
    }
    printf("___________________________________________\n");
  }

  if (!reg_infos_.empty()) {
    printf("The list of REGISTRY INFO in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT         TYPE     SIZE   ADDR           NAME\n");
  //printf("    1 aaaaaaaaaaaaaaaa 0000 0x7fc53c2af028   nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn\n");
    int count = 0;
    for (const auto& i : reg_infos_) {
      ++count;
      printf("%5d %16s %4ld %16p %-31s\n", count, i->data_type, i->n_res, i->addr, i->data_name);
    }
    printf("___________________________________________\n");
  }
  LOG_WARNING;
  printf("\n");
}

// } /* namespace middleware */
