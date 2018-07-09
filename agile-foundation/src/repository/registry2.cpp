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
  DT_BUILDIN,
  DT_VEC_I,
  DT_VEC_D,
  DT_MAT_I,
  DT_MAT_D
};

struct __RegInfo {
  DataType data_type;
  char     data_name[N_MAX_NAME];
  size_t   n_res;
  char     addr[0];
};

struct __ResStu {
  ResType    handle;
  __RegInfo* reg_info;
};

struct __CmdStu {
  CmdType    handle;
  __RegInfo* reg_info;
};

void __cvtResStu(const ResType& res, __RegInfo* stu) {
  DataType type = DataType::DT_UNKONWN;
  size_t   size = 0;
  if ( typeid(const short*) == res.type() ) {
    type = DataType::DT_BUILDIN;
    size = sizeof(short);
  } else if (typeid(const int*) == res.type()) {
    type = DataType::DT_VEC_I;
    size = sizeof(int);
  } else if (typeid(const double*) == res.type()) {
    type = DataType::DT_VEC_I;
    size = sizeof(double);
  } else if (typeid(const Eigen::VectorXi*) == res.type()) {
    type = DataType::DT_VEC_I;
    auto _res = boost::get<const Eigen::VectorXi*>(res);
    size = _res->size();
  } else if (typeid(const Eigen::MatrixXi*) == res.type()) {
    type = DataType::DT_MAT_I;
    auto _res = boost::get<const Eigen::MatrixXi*>(res);
    size = _res->size();
  } else if (typeid(const Eigen::VectorXd*) == res.type()) {
    type = DataType::DT_VEC_D;
    auto _res = boost::get<const Eigen::VectorXd*>(res);
    size = _res->size();
  } else if (typeid(const Eigen::MatrixXd*) == res.type()) {
    type = DataType::DT_MAT_D;
    auto _res = boost::get<const Eigen::MatrixXd*>(res);
    size = _res->size();
  } else {
    type = DataType::DT_UNKONWN;
    size = 0;
  }

  stu->data_type = type;
  stu->n_res     = size;
}

DataType __cvtDataType(const CmdType& res) {
  if ( typeid(short*) == res.type()
      || (typeid(int*) == res.type())
      || (typeid(double*) == res.type()) ) {

    return DataType::DT_BUILDIN;
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
  : shm_buffer_(nullptr), buff_top_(nullptr) {
  if (nullptr == SharedMem::instance()) {
    SharedMem::create_instance();
  }

  auto shm_manager = SharedMem::instance();
  // Create the buffer for data
  shm_manager->create_shm(RegDataName, N_INIT_BUF);
  shm_buffer_ = shm_manager->get_addr_from_shm(RegDataName);
  memset(shm_buffer_, 0x00, N_INIT_BUF);
  buff_top_ = shm_buffer_;

  ///! Add the register support thread.
  ThreadPool::instance()->add("registry2", &Registry2::support, this);
}

Registry2::~Registry2() {
  SharedMem::destroy_instance();
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

  __RegInfo* info   = (__RegInfo*) buff_top_;
  __cvtResStu(_handle, info);
  if (DataType::DT_UNKONWN == info->data_type || 0 == info->n_res) {
    LOG_ERROR << "What fucking the type of data! " << _handle.type().name();
    lock_.unlock();
    return false;
  }
  // fill the name
  memcpy(info->data_name, _n.c_str(), _n.size());
  info->data_name[_n.size()] = "\0";

  __ResStu* res   = new __ResStu;
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

  cmd_origin_[_n] = _handle;
  return true;
}

void Registry2::support() {
  TICKER_INIT(std::chrono::milliseconds);


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
