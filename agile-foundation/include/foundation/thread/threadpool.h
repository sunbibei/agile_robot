/*
 * ThreadManager.h
 *
 *  Created on: Sep 5, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_UTILS_THREADMANAGER_H_
#define INCLUDE_SYSTEM_UTILS_THREADMANAGER_H_

#include "foundation/utf.h"

#include <functional>
#include <thread>
#include <map>

///! cancel the namespace
// namespace agile_robot {

class ThreadPool {
  SINGLETON_DECLARE(ThreadPool)

public:
  /**
   * @brief This method allow you to add many functions associate with arguments
   *        into the internal vector. This class will start a thread for each
   *        function. An example as follow,
   *        bool debug() { };       void debug1() { }
   *        void print(double) { }; void print(int,int) { }
   *        class SomeClass {
   *          public:
   *          void method() { };
   *          void some_method() { }; void some_method(int) { };
   *        } *obj;
   *
   *        MiiThread t;
   *        t.add(debug); t.add(debug1); t.add(&A::method, obj);
   *        t.add(static_cast<void (&)(double)>(print), 0.123);
   *        t.add(static_cast<void (&)(int,int)>(print), 10, 100);
   *        t.add(static_cast<void (SomeClass::*)()>(&A::some_method), obj);
   *        t.add(static_cast<void (SomeClass::*)(int)>(&A::some_method). obj, 100);
   * @param __n        The name of thread
   * @param __f        The address of function
   * @param __args     The variadic templates offer the list of arguments.
   */
  template<typename _Func, typename... _BoundArgs>
  void add(const std::string& __n, _Func&& __f, _BoundArgs&&... __args);

  bool init();

  bool start();
  bool start(const std::string& __n);
  void stop();
  void stop(const std::string& __n);

  bool is_running(const std::string& __n);

protected:
  void __internal_thread_task(class __PrivateThreadVar*);
  void __register_thread_task(const std::string&, std::function<void()>&);
  /**
   * The list of thread function and variate.
   */
  std::map<std::string, class __PrivateThreadVar*>  thread_vars_;
};




///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of template methods         ////////////
///////////////////////////////////////////////////////////////////////////////
template<typename _Func, typename... _BoundArgs>
void ThreadPool::add(const std::string& __n, _Func&& __f, _BoundArgs&&... __args) {
  std::function<void()> f = std::bind(__f, __args...);
  __register_thread_task(__n, f);
}

// } /* namespace middleware */

#endif /* INCLUDE_SYSTEM_UTILS_THREADMANAGER_H_ */
