/*
 * registry.cpp
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#include "foundation/registry/registry2.h"
#include "foundation/thread/threadpool.h"
#include "foundation/ipc/msg_queue.h"
#include "foundation/ipc/shared_mem.h"

#include "foundation/utf.h"

#include <chrono>
#include <iostream>

#define N_MAX_TNAME     (14)
#define OCCUPY_FLAG     ((char)0x88)

///! The 1st style
#define STYLE_1ST      ((char)0x01)
///! The 2nd style
#define STYLE_2ND      ((char)0x02)
///! The id of the 2nd style
#define MSG_ID_2ND     ((char)0x02)

///! cancel the namespaces
// namespace agile_robot {

///! The information of registry, The structure must not be malloc by NEW,
///! and it is at the header of shared memory.
struct __RegInfo {
  char     occupy;
  char     comm_type;
  char     data_type[N_MAX_TNAME];
  char     data_name[N_MAX_NAME];

  size_t   n_res; // in byte
  char     addr[0];
};

///! The information of resource, it is difference in the different process.
///! The @addr is the head address of the content of handle.
struct __PubResStu1 {
  const void* addr;
  __RegInfo*  reg_info;
};

struct __SubResStu1 {
  void*       addr;
  __RegInfo*  reg_info;
};

///! The sub-structure was inherited from MsgQueue::MsgBase
struct __PubMsg : public MsgBase {
  char     addr[0];
};

typedef __PubMsg __SubMsg;


struct __PubResStu2 {
  std::atomic_bool* flag;
  size_t            n_res;
  const void*       addr;
  __PubMsg*         msg;

  __RegInfo*        reg_info;
};

struct __SubResStu2 {
  std::atomic_bool* flag;
  size_t            n_res;
  void*             addr;
  __SubMsg*         msg;

  __RegInfo*        reg_info;
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
  if (nullptr == MsgQueue::instance())
    LOG_FATAL << "YOU NEED CREATE THE instance of MsgQueue  firstly!";

  msgqs_           = MsgQueue ::instance();
  auto shm_manager = SharedMem::instance();
  // Create the buffer for data
  shm_manager->create_shm(RegDataName, N_INIT_BUF);
  shm_buffer_ = (char* )shm_manager->get_addr_from_shm(RegDataName);
  buff_top_   = shm_buffer_;

  ///! sync the reg_info
  __syncRegInfo();

  ///! Add the register support thread.
  if (nullptr != ThreadPool::instance())
    ThreadPool::instance()->add("registry2", &Registry2::support, this);
}

Registry2::~Registry2() {
  thread_alive_ = false;

  for (auto& pub : pub1_origin_) {
    delete pub.second;
    pub.second = nullptr;
  }
  pub1_origin_.clear();
}

void Registry2::__syncRegInfo() {
  lock_.lock(); // For thread safety

  __RegInfo* info = (__RegInfo*) buff_top_;
  while (OCCUPY_FLAG == info->occupy) {
    // LOG_ERROR << info->n_res;
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

bool Registry2::publish(const std::string& _n, const Eigen::VectorXd* ptr, std::atomic_bool* flag) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "vectorxd", flag);
}

bool Registry2::publish(const std::string& _n, const Eigen::VectorXi* ptr, std::atomic_bool* flag) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "vectorxi", flag);
}

bool Registry2::publish(const std::string& _n, const Eigen::MatrixXd* ptr, std::atomic_bool* flag) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "matrixxd", flag);
}

bool Registry2::publish(const std::string& _n, const Eigen::MatrixXi* ptr, std::atomic_bool* flag) {
  return pub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "matrixxi", flag);
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

bool Registry2::subscribe(const std::string& _n, Eigen::VectorXd* ptr, std::atomic_bool* flag) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "vectorxd", flag);
}

bool Registry2::subscribe(const std::string& _n, Eigen::VectorXi* ptr, std::atomic_bool* flag) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "vectorxi", flag);
}

bool Registry2::subscribe(const std::string& _n, Eigen::MatrixXd* ptr, std::atomic_bool* flag) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(double), "matrixxd", flag);
}

bool Registry2::subscribe(const std::string& _n, Eigen::MatrixXi* ptr, std::atomic_bool* flag) {
  return sub_helper(_n, ptr->data(), ptr->size()*sizeof(int), "matrixxi", flag);
}

__RegInfo* Registry2::__checkRegInfo(const std::string& _n,
    size_t size, char style, const std::string& _tn) {
  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _n << " is fail!";
    return nullptr;
  }
  // update the buff_top_;
  __syncRegInfo();

  __RegInfo* info = nullptr;
  for ( const auto& _i : reg_infos_) {
    if (0 == strcmp(_i->data_name, _n.c_str())) {
      info = _i;
      break;
    }
  }

  std::string tn = _tn.substr(0, std::min(_tn.size(), (size_t)(N_MAX_TNAME - 1)));
  if (nullptr == info) {
    lock_.lock();
    info = (__RegInfo*) buff_top_;
    // memset(info, 0x00, (sizeof(__RegInfo) + size));
    info->occupy    = OCCUPY_FLAG;
    info->n_res     = size;
    info->comm_type = style;
    memcpy(info->data_type, tn.c_str(), tn.size());
    memcpy(info->data_name, _n.c_str(), _n.size());

    ///! Add into the registry list.
    reg_infos_.push_back(info);
    // update the new top of buffer with the specify data.
    buff_top_ += (sizeof(__RegInfo) + info->n_res);

    lock_.unlock();
  }

  if ((0 != tn.compare(info->data_type)) || (style != info->comm_type)
      || ((STYLE_1ST == style) && (size != info->n_res)))
    return nullptr;

  return info;
}

bool Registry2::pub_helper(const std::string& _n, const void* addr, size_t size, const std::string& _tn) {
  if (pub1_origin_.end() != pub1_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table.";
    return true;
  }
  auto info = __checkRegInfo(_n, size, STYLE_1ST, _tn);

  __PubResStu1* res = new __PubResStu1;
  res->addr        = addr;
  res->reg_info    = info;

  pub1_origin_[_n] = res;
  return true;
}

bool Registry2::sub_helper(const std::string& _n, void* addr, size_t size, const std::string& _tn) {
  auto info = __checkRegInfo(_n, size, STYLE_1ST, _tn);

  __SubResStu1* res = new __SubResStu1;
  res->addr        = addr;
  res->reg_info    = info;

  sub1_origin_[_n]  = res;
  return true;
}

bool Registry2::pub_helper(const std::string& _n, const void* addr, size_t size,
    const std::string& _tn, std::atomic_bool* flag) {
  if (pub2_origin_.end() != pub2_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table.";
    return true;
  }
  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _n << " is fail!";
    return false;
  }

  LOG_ERROR << "1";
  // update the buff_top_;
  __syncRegInfo();

  __RegInfo* info = nullptr;
  for ( const auto& _i : reg_infos_) {
    if (0 == strcmp(_i->data_name, _n.c_str())) {
      info = _i;
      break;
    }
  }

  LOG_ERROR << "2";
  std::string tn = _tn.substr(0, std::min(_tn.size(), (size_t)(N_MAX_TNAME - 1)));
  if (nullptr == info) {
    lock_.lock();
    info = (__RegInfo*) buff_top_;
    memset(info, 0x00, (sizeof(__RegInfo) + size));
    memcpy(info->data_type, tn.c_str(), tn.size());
    memcpy(info->data_name, _n.c_str(), _n.size());
    info->n_res     = 0; // NOTE that the size of 2nd style is zero.
    info->occupy    = OCCUPY_FLAG;
    info->comm_type = STYLE_2ND;
    ///! Add into the registry list.
    reg_infos_.push_back(info);
    // update the new top of buffer with the specify data.
    buff_top_ += (sizeof(__RegInfo) + info->n_res);

    lock_.unlock();
  }

  LOG_ERROR << "3";
  if( (0 != tn.compare(info->data_type)) || STYLE_2ND != info->comm_type )
    return false;

  LOG_ERROR << "4";
  __PubResStu2* res = new __PubResStu2;
  res->flag         = flag;
  res->n_res        = size;
  res->addr         = addr;
  res->reg_info     = info;
  res->msg          = (__PubMsg*) malloc(sizeof(__PubMsg) + res->n_res);
  res->msg->msg_id  = MSG_ID_2ND;

  if (!msgqs_->create_msgq(_n)) {
    LOG_ERROR << "Create the MsgQueue fail with the given name[" << _n << "]";
    LOG_ERROR << "5";
    return false;
  }

  pub2_origin_[_n]  = res;
  return true;
}

bool Registry2::sub_helper(const std::string& _n, void* addr, size_t size,
    const std::string& _tn, std::atomic_bool* flag) {
  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _n << " is fail!";
    return false;
  }

  // update the buff_top_;
  __syncRegInfo();

  __RegInfo* info = nullptr;
  for ( const auto& _i : reg_infos_) {
    if (0 == strcmp(_i->data_name, _n.c_str())) {
      info = _i;
      break;
    }
  }

  std::string tn = _tn.substr(0, std::min(_tn.size(), (size_t)(N_MAX_TNAME - 1)));
  if (nullptr == info) {
    lock_.lock();
    info = (__RegInfo*) buff_top_;
    memset(info, 0x00, (sizeof(__RegInfo) + size));
    memcpy(info->data_type, tn.c_str(), tn.size());
    memcpy(info->data_name, _n.c_str(), _n.size());
    info->n_res     = 0; // NOTE that the size of 2nd style is zero.
    info->occupy    = OCCUPY_FLAG;
    info->comm_type = STYLE_2ND;
    ///! Add into the registry list.
    reg_infos_.push_back(info);
    // update the new top of buffer with the specify data.
    buff_top_ += (sizeof(__RegInfo) + info->n_res);
    lock_.unlock();
  }

  if( (0 != tn.compare(info->data_type)) || STYLE_2ND != info->comm_type )
    return false;

  __SubResStu2* res = new __SubResStu2;
  res->addr         = addr;
  res->flag         = flag;
  res->n_res        = size;
  res->reg_info     = info;
  res->msg          = (__SubMsg*) malloc(sizeof(__SubMsg) + res->n_res);
  res->msg->msg_id  = MSG_ID_2ND;

  if (!msgqs_->create_msgq(_n)) {
    LOG_ERROR << "Create the MsgQueue fail with the given name[" << _n << "]";
    return false;
  }

  sub2_origin_[_n]  = res;
  return true;
}

void Registry2::support() {
  TICKER_INIT(std::chrono::microseconds);

  thread_alive_ = true;
  while (thread_alive_) {
    for (const auto& pair : pub1_origin_) {
      auto res = pair.second;
      if (nullptr != res->addr)
        memcpy(res->reg_info->addr, res->addr, res->reg_info->n_res);
    }

    for (const auto& pair : sub1_origin_) {
      auto res = pair.second;
      if (nullptr != res->addr)
        memcpy(res->addr, res->reg_info->addr, res->reg_info->n_res);
    }

    for (const auto& pair : pub2_origin_) {
      auto res = pair.second;
      if (res->flag->load()) {
        LOG_ERROR << "ENTER";
        memcpy(res->msg->addr, res->addr, res->n_res);
        msgqs_->write_to_msgq(pair.first, res->msg, res->n_res + sizeof(__PubMsg));

        /// change the flag.
        res->flag->store(false);
      }
    }

    for (const auto& pair : sub2_origin_) {
      auto res = pair.second;
      if (msgqs_->read_from_msgq(pair.first, res->msg, res->n_res + sizeof(__PubMsg))) {
        memcpy(res->addr, res->msg->addr, res->n_res);

        int64_t timestamp = std::chrono::time_point_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now()).time_since_epoch().count();
        LOG_ERROR << "LAPES: " << (timestamp - res->msg->timestamp);
        /// change the flag.
        res->flag->store(true);
      }
    }

    TICKER_CONTROL(100, std::chrono::microseconds);
  }

}

///! print the all of registry.
void Registry2::print() {
  __syncRegInfo();

  printf("\n");
  LOG_WARNING;
  if (!pub1_origin_.empty()) {
    printf("The list of PUBLISH in 1st style\n");
    printf("-------------------------------------------\n");
    printf("COUNT       TYPE             ADDR       NAME\n");
 // printf("    0 aaaaaaaaaaaaaaaa 0x7fc53c2af028ab test-res-d\n");
    int count = 0;
    for (const auto& l : pub1_origin_) {
      ++count;
      printf("%5d %16s %16p %-31s\n", count,
          l.second->reg_info->data_type, l.second, l.first.c_str());
    }
    printf("___________________________________________\n");
  }

  if (!sub1_origin_.empty()) {
    printf("The list of SUBSCRIBE in 1st style\n");
    printf("-------------------------------------------\n");
    printf("COUNT       TYPE             ADDR       NAME\n");
 // printf("    0 aaaaaaaaaaaaaaaa 0x7fc53c2af028ab test-res-d\n");
    int count = 0;
    for (const auto& l : sub1_origin_) {
      ++count;
      printf("%5d %16s %16p %-31s\n", count,
          l.second->reg_info->data_type, l.second, l.first.c_str());
    }
    printf("___________________________________________\n");
  }

  if (!pub2_origin_.empty()) {
    printf("The list of PUBLISH in 2nd style\n");
    printf("-------------------------------------------\n");
    printf("COUNT       TYPE             ADDR       NAME\n");
 // printf("    0 aaaaaaaaaaaaaaaa 0x7fc53c2af028ab test-res-d\n");
    int count = 0;
    for (const auto& l : pub2_origin_) {
      ++count;
      printf("%5d %16s %16p %-31s\n", count,
          l.second->reg_info->data_type, l.second, l.first.c_str());
    }
    printf("___________________________________________\n");
  }

  if (!sub2_origin_.empty()) {
    printf("The list of SUBSCRIBE in 2nd style\n");
    printf("-------------------------------------------\n");
    printf("COUNT       TYPE             ADDR       NAME\n");
 // printf("    0 aaaaaaaaaaaaaaaa 0x7fc53c2af028ab test-res-d\n");
    int count = 0;
    for (const auto& l : sub2_origin_) {
      ++count;
      printf("%5d %16s %16p %-31s\n", count,
          l.second->reg_info->data_type, l.second, l.first.c_str());
    }
    printf("___________________________________________\n");
  }

  if (!reg_infos_.empty()) {
    printf("The list of REGISTRY INFO in this process\n");
    printf("-------------------------------------------\n");
    printf("COUNT         TYPE     SIZE       ADDR       NAME\n");
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
