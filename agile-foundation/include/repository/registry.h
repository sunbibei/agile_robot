/*
 * registry.h
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_REGISTRY_H_
#define INCLUDE_REPOSITORY_REGISTRY_H_

#include "foundation/utf.h"

#include <boost/variant.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <atomic>

namespace middleware {

#define REG_RESOURCE(_n, _var)  \
  ( middleware::Registry::instance()->registerResource((_n), (_var)) )

#define REG_COMMAND(_n, _var, _flag)   \
  ( middleware::Registry::instance()->registerCommand ((_n), (_var), (_flag)) )

#define REG_COMMAND_NO_FLAG(_n, _var)   \
  ( middleware::Registry::instance()->registerCommand ((_n), (_var)) )

#define GET_RESOURCE(_n, _type) \
  ( middleware::Registry::instance()->resource< _type >(_n) )

#define GET_COMMAND(_n, _flag, _type) \
  ( middleware::Registry::instance()->command< _type >(_n, _flag) )

#define GET_COMMAND_NO_FLAG(_n, _type) \
  ( middleware::Registry::instance()->command< _type >(_n) )

template <typename _T>
using MiiPtr = boost::shared_ptr<_T>;

typedef boost::variant<const short*, const int*, const double*,
    const Eigen::VectorXi*, const Eigen::MatrixXi*,
    const Eigen::VectorXd*, const Eigen::MatrixXd*> ResType;

typedef boost::variant<short*, int*, double*,
    Eigen::VectorXi*, Eigen::MatrixXi*,
    Eigen::VectorXd*, Eigen::MatrixXd*> CmdType;

class Registry {
  SINGLETON_DECLARE(Registry)

public:
  bool registerResource(const std::string&, ResType);
  bool registerCommand (const std::string&, CmdType, std::atomic_bool** flag = nullptr);

  ///! The boost static assert fail! so we need split into two methods.
  template<typename _DataType>
  _DataType resource(const std::string&);
  template<typename _DataType>
  _DataType command(const std::string&, std::atomic_bool** flag = nullptr);

public:
  ///! Query the given name whether register in the registry.
  // bool query(const std::string&);

  ///! Search by the key words and return the results.
  // std::string search(const std::string&);

  ///! print the all of registry.
  void print();

protected:
  std::map<std::string, ResType>    res_origin_;

  typedef struct {
    CmdType          handle;
    std::atomic_bool* flag;
  } CmdStruct;
  std::map<std::string, CmdStruct>  cmd_origin_;
};

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template <typename _DataType>
_DataType Registry::resource(const std::string& _res_name) {
  if (res_origin_.end() == res_origin_.find(_res_name)) {
    return _DataType(nullptr);
  }
  auto var_data = res_origin_[_res_name];
  assert(var_data.type() == typeid(_DataType));

  return boost::get<_DataType>(var_data);
}

template <typename _DataType>
_DataType Registry::command(const std::string& _res_name, std::atomic_bool** flag) {
  if (cmd_origin_.end() == cmd_origin_.find(_res_name)) {
    return _DataType(nullptr);
  }

  auto var_cmd = cmd_origin_[_res_name];
  assert(var_cmd.handle.type() == typeid(_DataType));

  if (flag) *flag = var_cmd.flag;
  return boost::get<_DataType>(var_cmd.handle);
}

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_REGISTRY_H_ */
