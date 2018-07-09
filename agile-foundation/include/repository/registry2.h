/*
 * registry.h
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_REGISTRY_H_
#define INCLUDE_REPOSITORY_REGISTRY_H_

#include "foundation/utf.h"
#include "foundation/ipc/shared_mem.h"

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <atomic>
#include <mutex>
#include <list>
#include <map>

#define N_INIT_BUF (1024*1024)
#define N_SNYC     (1024)
#define N_MAX_NAME (31)

namespace agile_robot {

/*
// #define REG_RESOURCE(_n, _var)  \
//  ( agile_robot::Registry::instance()->registerResource((_n), (_var)) )
//
// #define REG_COMMAND(_n, _var, _flag)   \
//  ( agile_robot::Registry::instance()->registerCommand ((_n), (_var), (_flag)) )
//
// #define REG_COMMAND_NO_FLAG(_n, _var)   \
//  ( agile_robot::Registry::instance()->registerCommand ((_n), (_var)) )
//
// #define GET_RESOURCE(_n, _type) \
//  ( agile_robot::Registry::instance()->resource< _type >(_n) )
//
// #define GET_COMMAND(_n, _flag, _type) \
//  ( agile_robot::Registry::instance()->command< _type >(_n, _flag) )
//
// #define GET_COMMAND_NO_FLAG(_n, _type) \
//  ( agile_robot::Registry::instance()->command< _type >(_n) )
*/

template <typename _T>
using MiiPtr = boost::shared_ptr<_T>;

typedef boost::variant<const short*, const int*, const double*,
    const Eigen::VectorXi*, const Eigen::MatrixXi*,
    const Eigen::VectorXd*, const Eigen::MatrixXd*> ResType;

typedef boost::variant<short*, int*, double*,
    Eigen::VectorXi*, Eigen::MatrixXi*,
    Eigen::VectorXd*, Eigen::MatrixXd*> CmdType;

/*!
 * @brief \class Registry2, support the data exchange under multi-process.
 */
class Registry2 {
  SINGLETON_DECLARE(Registry2)

public:
  /*!
   * @brief initialize something.
   */
  bool init();
  /*!
   * @brief register a resource into the registry2.
   *        Note, The name of resource must be unique, or it will be replaced.
   *        The resource must be a reference to the actual value, The registry
   *        thread will be visit the reference.
   */
  bool registerResource(const std::string&, ResType);

  /*!
   * @brief Register a command into the registry2.
   *        Note, the name of command must be unique, or it will be replaced.
   *        The resource must be a reference to the actual value, The registry
   *        thread will be visit the reference.
   */
  bool registerCommand (const std::string&, CmdType);

  ///! The boost static assert fail! so we need split into two methods.
  template<typename _DataType>
  _DataType resource(const std::string&);

  template<typename _DataType>
  _DataType command(const std::string&);

public:
  ///! Query the given name whether register in the registry.
  // bool query(const std::string&);

  ///! Search by the key words and return the results.
  // std::string search(const std::string&);

  ///! print the all of registry.
  void print();

protected:
  void support();

  void syncRegInfo();

protected:
  ///! The buffer for the all of data.
  char* shm_buffer_;
  ///! The available buffer header.
  char* buff_top_;
  ///! For thread safety
  std::mutex lock_;
  ///! The list of resources or command
  std::map<std::string, class __ResStu*>  res_origin_;
  std::map<std::string, class __CmdStu*>  cmd_origin_;
  std::list<class __RegInfo*> reg_infos_;
  ///! Whether is the thread alive.
  bool thread_alive_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
ResType __get_res(std::map<std::string, class __ResStu*>& res, const std::string& _n);
CmdType __get_cmd(std::map<std::string, class __CmdStu*>& cmd, const std::string& _n);

template <typename _DataType>
_DataType Registry2::resource(const std::string& _res_name) {
  ResType var_data = __get_res(res_origin_, _res_name);
  if (var_data.empty()) {
    return _DataType(nullptr);
  }

  LOG_INFO << var_data.type().name() << " v.s. " << typeid(_DataType).name();
  assert(var_data.type() == typeid(_DataType));
  return boost::get<_DataType>(var_data);
}

template <typename _DataType>
_DataType Registry2::command(const std::string& _res_name) {
  return nullptr;
//  if (cmd_origin_.end() == cmd_origin_.find(_res_name)) {
//    return _DataType(nullptr);
//  }
//
//  auto var_cmd = cmd_origin_[_res_name]->handle;
//  assert(var_cmd.type() == typeid(_DataType));
//
//  return boost::get<_DataType>(var_cmd);
}

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_REGISTRY_H_ */
