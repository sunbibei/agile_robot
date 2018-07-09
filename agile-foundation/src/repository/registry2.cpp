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

enum DataType : char {
  DT_UNKONWN,
  DT_SHORT,
  DT_INT,
  DT_DOUBLE,
  DT_VEC_I,
  DT_VEC_D,
  DT_MAT_I,
  DT_MAT_D
};

struct __RegInfo {
  DataType data_type;
  char     data_name[N_MAX_NAME];
  size_t   n_res; // in byte
  char     addr[0];
};

struct __ResStu {
  ResType    handle;
  ///! For the publisher,  the @from is NOT null.
  ///! For the subscriber, the @from is null.
  void*      from;
  ///! For the publisher,  the @to is null.
  ///! For the subscriber, the @to is NOT null.
  void*      to;
  __RegInfo* reg_info;
};

struct __CmdStu {
  CmdType    handle;
  void*      addr;
  __RegInfo* reg_info;
};

ResType __get_res(std::map<std::string, __ResStu*>& res,
    const std::string& _n) {

  LOG_INFO << "RES: " << (res.end() == res.find(_n));
  return (res.end() == res.find(_n)) ? ResType() : res[_n]->handle;
}

void __cvtResStu(const ResType& res, __RegInfo* stu, void*& addr) {
  DataType type = DataType::DT_UNKONWN;
  size_t   size = 0;
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
  } else if (typeid(const Eigen::MatrixXi*) == res.type()) {
    type = DataType::DT_MAT_I;
    auto _res = boost::get<const Eigen::MatrixXi*>(res);
    size = _res->size() * sizeof(int);
    addr = (void* ) _res->data();
  } else if (typeid(const Eigen::VectorXd*) == res.type()) {
    type = DataType::DT_VEC_D;
    auto _res = boost::get<const Eigen::VectorXd*>(res);
    size = _res->size() * sizeof(double);
    addr = (void* ) _res->data();
  } else if (typeid(const Eigen::MatrixXd*) == res.type()) {
    type = DataType::DT_MAT_D;
    auto _res = boost::get<const Eigen::MatrixXd*>(res);
    size = _res->size() * sizeof(double);
    addr = (void* ) _res->data();
  } else {
    type = DataType::DT_UNKONWN;
    size = 0;
  }

  stu->data_type = type;
  stu->n_res     = size;
}

DataType __cvtDataType(const CmdType& res) {
  if ( typeid(short*) == res.type() ) {

    return DataType::DT_SHORT;
  } else if (typeid(int*) == res.type()) {

    return DataType::DT_INT;
  } else if (typeid(double*) == res.type()) {

    return DataType::DT_DOUBLE;
  } else if (typeid(Eigen::VectorXi*) == res.type()) {

    return DataType::DT_VEC_I;
  } else if (typeid(Eigen::MatrixXi*) == res.type()) {

    return DataType::DT_MAT_I;
  } else if (typeid(Eigen::VectorXd*) == res.type()) {

    return DataType::DT_VEC_D;
  } else if (typeid(Eigen::MatrixXd*) == res.type()) {

    return DataType::DT_MAT_D;
  } else {
    return DataType::DT_UNKONWN;
  }

}

SINGLETON_IMPL(Registry2)

const std::string RegDataName = "registry2-data";
const std::string SyncName    = "registry2-sync";
Registry2::Registry2()
  : shm_buffer_(nullptr), buff_top_(nullptr),
    thread_alive_(true) {
  ; // Nothing to do here.
}

Registry2::~Registry2() {
  SharedMem::destroy_instance();
}

void Registry2::syncRegInfo() {
  __RegInfo* info = (__RegInfo*) buff_top_;
  while ( DataType::DT_UNKONWN != info->data_type && 0 != info->n_res ) {
    // record the reg_info
    reg_infos_.push_back(info);
    if (res_origin_.end() == res_origin_.find(info->data_name)) {
      __ResStu* res = new __ResStu;
      void* addr = nullptr;
      switch (info->data_type) {
      case DataType::DT_SHORT: {
        addr = new short;
        res->handle = (const short*) addr;
        break;
      }
      case DataType::DT_INT: {
        addr = new int;
        res->handle = (const int*) addr;
        break;
      }
      case DataType::DT_DOUBLE: {
        addr = new double;
        res->handle = (const double*) addr;
        break;
      }
      case DataType::DT_VEC_I: {
        auto vec_i = new Eigen::VectorXi;
        vec_i->resize( info->n_res / sizeof(int) );
        addr = vec_i->data();
        res->handle = vec_i;
        break;
      }
      case DataType::DT_VEC_D: {
        LOG_INFO << "DT_VEC_D";
        auto vec_d = new Eigen::VectorXd;
        vec_d->resize( info->n_res / sizeof(double) );
        addr = vec_d->data();
        res->handle = (const Eigen::VectorXd*)vec_d;
        break;
      }
//      case DataType::DT_MAT_I: {
//        auto mat_i = new Eigen::MatrixXi;
//        mat_i->resize( info->n_res / sizeof(int) );
//        addr = mat_i->data();
//        res->handle = mat_i;
//        break;
//      }
//      case DataType::DT_MAT_D: {
//        auto mat_d = new Eigen::MatrixXd;
//        mat_d->resize( info->n_res / sizeof(int) );
//        addr = mat_d->data();
//        res->handle = mat_d;
//        break;
//      }
      default:
        break;
      }

      res->reg_info = info;
      res->to       = addr;
      res->from     = nullptr;
      memcpy(addr, info->addr, info->n_res);
      res_origin_[info->data_name] = res;
    }

    buff_top_ += (sizeof(__RegInfo) + info->n_res);
    info = (__RegInfo*) buff_top_;
  } // search and location the tail of buffer.
}

bool Registry2::init() {
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
  return true;
}

bool Registry2::registerResource(const std::string& _n, ResType _handle) {
  if (res_origin_.end() != res_origin_.find(_n)) {
    LOG_WARNING << "The named resource '" << _n << "' has registered in the resource table.";
    return true;
  }

  if ( _n.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named " << _n << " is fail!";
    return false;
  }
  ///! For thread safety
  lock_.lock();

  __ResStu* res   = new __ResStu;
  __RegInfo* info = (__RegInfo*) buff_top_;
  __cvtResStu(_handle, info, res->from);
  res->to = nullptr;

  if (DataType::DT_UNKONWN == info->data_type || 0 == info->n_res) {
    LOG_ERROR << "What fucking the type of data! " << _handle.type().name();

    info->data_type = DataType::DT_UNKONWN;
    info->n_res     = 0;
    delete res;

    lock_.unlock();
    return false;
  }

  // fill the name
  memcpy(info->data_name, _n.c_str(), _n.size());
  info->data_name[_n.size()] = '\0';
  memset(info->addr, 0x00, info->n_res);

  res->handle     = _handle;
  res->reg_info   = info;
  res_origin_[_n] = res;
  // update the new top of buffer with the specify data.
  buff_top_ += (sizeof(__RegInfo) + info->n_res);
  ((__RegInfo*) buff_top_)->data_type = DataType::DT_UNKONWN;
  ((__RegInfo*) buff_top_)->n_res     = 0;

  lock_.unlock();
  return true;
}

bool Registry2::registerCommand(const std::string& _n, CmdType _handle) {
  if (cmd_origin_.end() != cmd_origin_.find(_n)) {
    LOG_WARNING << "The named command '" << _n << "' has registered in the command table."
        << ", now it will be replaced.";
  }

  // cmd_origin_[_n] = _handle;
  return true;
}

void Registry2::support() {
  TICKER_INIT(std::chrono::milliseconds);

  for (const auto& pair : res_origin_) {
    printf("%s\n", pair.first.c_str());
    auto res = pair.second;
    printf("  FROM: %p, TO: %p, INFO: %p\n", res->from, res->to, res->reg_info);
    if (nullptr != res->reg_info) {
      printf("  %-31s, %d, %d\n", res->reg_info->data_name,
          res->reg_info->data_type, res->reg_info->n_res);
    }
  }

  thread_alive_ = true;
  while (thread_alive_) {
    for (const auto& pair : res_origin_) {
      auto res = pair.second;
      if (nullptr != res->from)
        memcpy(res->reg_info->addr, res->from, res->reg_info->n_res);
      if (nullptr != res->to)
        memcpy(res->to, res->reg_info->addr,   res->reg_info->n_res);
    }

    TICKER_CONTROL(10, std::chrono::milliseconds);
  }

}

inline std::string __getTypeName(const ResType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(const int*) == t.type()) {
    return "int     ";
  } else if (typeid(const double*) == t.type()) {
    return "double  ";
  } else if (typeid(const Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(const Eigen::MatrixXi*) == t.type()) {
    return "matrixXi";
  } else if (typeid(const Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else if (typeid(const Eigen::MatrixXd*) == t.type()) {
    return "matrixXd";
  } else {
    return t.type().name();
  }
}

inline std::string __getTypeName(const CmdType& t) {
  if (typeid(const short*) == t.type()) {
    return "short   ";
  } else if (typeid(int*) == t.type()) {
    return "int     ";
  } else if (typeid(double*) == t.type()) {
    return "double  ";
  } else if (typeid(Eigen::VectorXi*) == t.type()) {
    return "vectorXi";
  } else if (typeid(Eigen::MatrixXi*) == t.type()) {
    return "matrixXi";
  } else if (typeid(Eigen::VectorXd*) == t.type()) {
    return "vectorXd";
  } else if (typeid(Eigen::MatrixXd*) == t.type()) {
    return "matrixXd";
  } else {
    return t.type().name();
  }
}

///! print the all of registry.
void Registry2::print() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "The size of resource Registry: " << res_origin_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "COUNT\tNAME\t\tTYPE\t\tADDR";
    int count = 0;
    for (const auto& l : res_origin_) {
      LOG_INFO << count++ << "\t" << l.first << "\t" << __getTypeName(l.second->handle)
          << "  " << l.second;
    }
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "The size of command Registry: " << cmd_origin_.size();
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_WARNING << "COUNT\tNAME\t\tTYPE\t\tADDR";
    count = 0;
    for (const auto& l : cmd_origin_) {
      LOG_INFO << count++ << "\t" << l.first << "\t" << __getTypeName(l.second->handle)
          << "  " << l.second;
    }
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
}

} /* namespace middleware */
