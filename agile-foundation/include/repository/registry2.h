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
    const Eigen::VectorXi*, const Eigen::VectorXd*> ResType;

/*!
 * @brief \class Registry2, support the data exchange under Multi-process.
 */
class Registry2 {
  SINGLETON_DECLARE(Registry2)

public:
  /*!
   * @brief register a resource into the registry2.
   *        Note, The name of resource must be unique, or it will be replaced.
   *        The resource must be a reference to the actual value, The registry
   *        thread will be visit the reference.
   */
  bool publish(const std::string&, ResType);

  ///! The boost static assert fail! so we need split into two methods.
  template<typename _DataType>
  const _DataType* subscribe(const std::string&, size_t size);

public:
  ///! print the all of registry.
  void print();

protected:
  void support();

  ///! called when try to obtain the resource or the constructor.
  void syncRegInfo();
  ///! insert a reg info into the buffer(NO PUBLISHER).
  void insertRegInfo(std::map<std::string, class __ResStu*>&,
      const std::string&, ResType&);

  ///! Add a new publisher
  class __ResStu* addPuber(const std::string&, ResType&, class __RegInfo*);

protected:
  ///! The buffer for the all of data.
  char* shm_buffer_;
  ///! The available buffer header.
  char* buff_top_;
  ///! For thread safety
  std::mutex lock_;
  ///! The list of resources or command
  std::map<std::string, class __ResStu*>  pub_origin_;
  std::map<std::string, class __ResStu*>  sub_origin_;
  std::list<class __RegInfo*> reg_infos_;
  ///! Whether is the thread alive.
  bool thread_alive_;
};



///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
bool __get_res(std::map<std::string, __ResStu*>&,
    const std::string&, ResType&);

template <typename _DataType>
const _DataType* Registry2::subscribe(const std::string& _res_name, size_t size) {
  if ( _res_name.size() >= N_MAX_NAME ) {
    LOG_ERROR << "The max name of resource or command is " << N_MAX_NAME
        << ", Registry the resource or command with named "
        << _res_name << " is fail!";
    return nullptr;
  }
  // update the list of registry.
  syncRegInfo();

  ResType var_data;
  if (sub_origin_.end() == sub_origin_.find(_res_name)) {
    auto data_type = new _DataType;
    if (typeid(_DataType) == typeid(Eigen::VectorXd)
        || typeid(_DataType) == typeid(Eigen::VectorXi)) {
      if (size <= 0) {
        LOG_ERROR << "YOU NEED TO GIVE THE SIZE!";
        delete data_type;
        return nullptr;
      }

      data_type->resize(size);
      data_type->fill(0.0);
    }

    var_data = data_type;
    insertRegInfo(sub_origin_, _res_name, var_data);
  } else {
    __get_res(sub_origin_, _res_name, var_data);
  }

  LOG_INFO << var_data.type().name() << " v.s. " << typeid(const _DataType*).name();
  assert(var_data.type() == typeid(const _DataType*));
  return boost::get<const _DataType*>(var_data);
}

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_REGISTRY_H_ */
