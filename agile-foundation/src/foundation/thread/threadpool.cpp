/*
 * thread.cpp
 *
 *  Created on: Sep 5, 2017
 *      Author: bibei
 */

#include "foundation/thread/threadpool.h"

#include <boost/bind.hpp>
#include <chrono>
#include <thread>

namespace middleware {

struct __PrivateThreadVar {
  std::string    thread_name_;
  bool         thread_alive_;
  std::thread* thread_handle_;
  std::function<void()> thread_func_;
  __PrivateThreadVar(const std::string& _n, std::function<void()>& _f)
  : thread_name_(_n),        thread_alive_(false),
    thread_handle_(nullptr), thread_func_(_f) { }
};

SINGLETON_IMPL(ThreadPool)

ThreadPool::ThreadPool() {
}

ThreadPool::~ThreadPool() {
  stop();
}

void ThreadPool::__register_thread_task(const std::string& __n, std::function<void()>& __f) {
  if (thread_vars_.end() != thread_vars_.find(__n))
    LOG_WARNING << "The named thread(" << __n << ") task function "
      << "has inserted into the function list. It will be replaced.";

  __PrivateThreadVar* var = new __PrivateThreadVar(__n, __f);
  // LOG_DEBUG << "ThreadPool has received a thread task, named '" << __n << "'.";
  thread_vars_.insert(std::make_pair(__n, var));
}

void ThreadPool::__internal_thread_task(__PrivateThreadVar* var) {
  if (nullptr == var) {
    LOG_ERROR << "One of threads you want to launch does not exist!";
    return;
  }

  LOG_DEBUG << "The thread has started -- " << var->thread_name_;
  var->thread_alive_ = true;
  var->thread_func_();
  var->thread_alive_ = false;
  LOG_DEBUG << "The thread has  exited -- " << var->thread_name_;
}

bool ThreadPool::init() {
  return true;
}

bool ThreadPool::start() {
  bool ret = true;
  for (auto& f : thread_vars_) {
    ret  = ret && start(f.first);
  }
  return ret;
}

bool ThreadPool::start(const std::string& __n) {
  // LOG_DEBUG << "This thread is starting -- " << __n;
  // start the specific named threads
  auto iter = thread_vars_.find(__n);
  if (thread_vars_.end() == iter) {
    LOG_WARNING << "The task function of the specific named thread "
        << "isn't exist in function list.";
    return false;
  }
  auto var = iter->second;
  if ((nullptr == var) || (0 != __n.compare(var->thread_name_))
      || (nullptr != var->thread_handle_) || (var->thread_alive_)) {
    LOG_WARNING << "The " << __n << " thread launchs fail, something is wrong!"
        << ((((nullptr == var) || (0 != __n.compare(var->thread_name_)))) ?
            ("Register Wrong!")
            : (((nullptr != var->thread_handle_) || (var->thread_alive_)) ?
                ("Thread has launched ago!")
                : ("What fucking wrong!")));
    return false;
  }

  var->thread_handle_  = new std::thread(
      std::bind(&ThreadPool::__internal_thread_task, this, var));
  return true;
}

void ThreadPool::stop() {
  while (!thread_vars_.empty())
    stop(thread_vars_.begin()->first);

  thread_vars_.clear();
}

void ThreadPool::stop(const std::string& __n) {
  auto var = thread_vars_.find(__n);
  if (thread_vars_.end() == var) {
    LOG_WARNING << "These is not exist the named thread('" << __n
        << "') in the thread pool.";
    return;
  }

  if (nullptr != var->second) {
    if (nullptr != var->second->thread_handle_) {
      var->second->thread_handle_->join();
      delete var->second->thread_handle_;
      var->second->thread_handle_ = nullptr;
    }
    delete var->second;
    var->second = nullptr;
  }

  thread_vars_.erase(var);
}

bool ThreadPool::is_running(const std::string& __n) {
  return ((thread_vars_.end() != thread_vars_.find(__n))
      && (thread_vars_.find(__n)->second->thread_alive_));
}

} /* namespace middleware */
