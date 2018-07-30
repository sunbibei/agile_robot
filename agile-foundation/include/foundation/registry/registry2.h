/*
 * registry.h
 *
 *  Created on: Nov 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_REGISTRY2_H_
#define INCLUDE_REPOSITORY_REGISTRY2_H_

#include "foundation/utf.h"

#include <Eigen/Dense>
#include <atomic>
#include <mutex>
#include <list>
#include <map>

#define N_INIT_BUF (1024*1024)
#define N_SNYC     (1024)
#define N_MAX_NAME (32)

///! cancel the namespaces
// namespace agile_robot {

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

/*!
 * @brief \class Registry2, support the data exchange under Multi-process.
 */
class Registry2 {
  SINGLETON_DECLARE(Registry2)

  friend class Registry2Monitor;

/*!
 *  The 1st style for communication between multi-process. The 1st style SUPPORT
 *  that multi-publisher or multi-subscriber.
 *  Process A send some data into the shared memory, Process B read this data.
 *  You(Publisher) should MUST BE give a pointer point to anywhere you save
 *  data. We should read this data in each tick, and then write into the shared
 *  memory on the Process A. Process B will read the data from shared memory
 *  And then write into the pointer which given by the subscriber.
 */
public:
  /*!
   * @brief register a resource into the registry2.
   *        Note, The name of resource must be unique, or it will be ignored.
   *        The resource must be a reference to the actual value, The registry
   *        thread will be visit the reference.
   */
  template<typename _T>
  bool publish(const std::string&, const _T*);
  ////////////////// Template Specialization
  bool publish(const std::string&, const Eigen::VectorXd*);
  bool publish(const std::string&, const Eigen::VectorXi*);
  bool publish(const std::string&, const Eigen::MatrixXd*);
  bool publish(const std::string&, const Eigen::MatrixXi*);

  /*!
   * @brief subscribe a resource from the registry2. The name of resource maybe
   *        not contains in the registry.
   *        NOTE: The resource MUST BE malloced before, If this resource is not
   *        publish by others, it will reserve memory with the given size.
   */
  template<typename _T>
  bool subscribe(const std::string&, _T*);
  ////////////////// Template Specialization
  bool subscribe(const std::string&, Eigen::VectorXd*);
  bool subscribe(const std::string&, Eigen::VectorXi*);
  bool subscribe(const std::string&, Eigen::MatrixXd*);
  bool subscribe(const std::string&, Eigen::MatrixXi*);

/*!
 *  The 2nd style is designed for communication between TWO different process,
 *  of course, it is can running between more than two process normally, BUT
 *  the any message got by process A, this message CAN NOT got by the other process.
 *  NOTE that this style is INAPPROPRIATE to exchange great data between two process.
 */
public:
  /*!
   * @brief register a resource into the registry2.
   *        Note, The name of resource must be unique, or it will be ignored.
   *        The resource must be a reference to the actual value, The registry
   *        thread will be visit the reference. The atomic_bool is a pointer,
   *        Only when the variable is true, the resource will be read and write
   *        to the others.
   */
  template<typename _T>
  bool publish(const std::string&, const _T*, std::atomic_bool*);
  ////////////////// Template Specialization
  bool publish(const std::string&, const Eigen::VectorXd*, std::atomic_bool*);
  bool publish(const std::string&, const Eigen::VectorXi*, std::atomic_bool*);
  bool publish(const std::string&, const Eigen::MatrixXd*, std::atomic_bool*);
  bool publish(const std::string&, const Eigen::MatrixXi*, std::atomic_bool*);

  /*!
   * @brief subscribe a resource from the registry2. The name of resource maybe
   *        not contains in the registry.
   *        NOTE: The resource MUST BE malloced before, If this resource is not
   *        publish by others, it will reserve memory with the given size.
   */
  template<typename _T>
  bool subscribe(const std::string&, _T*, std::atomic_bool*);
  ////////////////// Template Specialization
  bool subscribe(const std::string&, Eigen::VectorXd*, std::atomic_bool*);
  bool subscribe(const std::string&, Eigen::VectorXi*, std::atomic_bool*);
  bool subscribe(const std::string&, Eigen::MatrixXd*, std::atomic_bool*);
  bool subscribe(const std::string&, Eigen::MatrixXi*, std::atomic_bool*);

public:
  ///! print the all of registry.
  void print();

protected:
  void support();

private:
  bool pub_helper(const std::string&, const void*, size_t, const std::string&);
  bool sub_helper(const std::string&,       void*, size_t, const std::string&);

  bool pub_helper(const std::string&, const void*, size_t, const std::string&, std::atomic_bool*);
  bool sub_helper(const std::string&,       void*, size_t, const std::string&, std::atomic_bool*);
  ///! called when try to obtain the resource or the constructor.
  void syncRegInfo();

protected:
  ///! The buffer for the all of data.
  char* shm_buffer_;
  ///! The available buffer header.
  char* buff_top_;
  ///! For thread safety
  std::mutex lock_;
  ///! The list of resources or command using the 1st style
  std::map<std::string, class __PubResStu1*> pub1_origin_;
  std::map<std::string, class __SubResStu1*> sub1_origin_;

  ///! The list of resources or command using the 2nd style
  std::map<std::string, class __PubResStu2*> pub2_origin_;
  std::map<std::string, class __SubResStu2*> sub2_origin_;
  class MsgQueue*                            msgqs_;

  ///! Record the all of resource or command.
  std::list<class __RegInfo*>                reg_infos_;
  ///! Whether is the thread alive.
  // std::atomic_bool thread_alive_;
};

// } /* namespace middleware */

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<typename _T>
bool Registry2::publish(const std::string& _n, const _T* ptr) {
  return pub_helper(_n, ptr, sizeof(_T), typeid(_T).name());
}

template<typename _T>
bool Registry2::publish(const std::string& _n, const _T* ptr, std::atomic_bool* flag) {
  return pub_helper(_n, ptr, sizeof(_T), typeid(_T).name(), flag);
}

template<typename _T>
bool Registry2::subscribe(const std::string& _n, _T* ptr) {
  return sub_helper(_n, ptr, sizeof(_T), typeid(_T).name());
}

template<typename _T>
bool Registry2::subscribe(const std::string& _n, _T* ptr, std::atomic_bool* flag) {
  return sub_helper(_n, ptr, sizeof(_T), typeid(_T).name(), flag);
}

#endif /* INCLUDE_REPOSITORY_REGISTRY_H_ */
