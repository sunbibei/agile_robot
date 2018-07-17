/*
 * registry_monitor.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: bibei
 */

#include "foundation/registry/registry2.h"

#include <thread>
#include <chrono>
#include <stdio.h>
#include <sys/wait.h>

static bool is_alive = true;

void termial(int signo) {
  printf("Ready to shutdown the monitor of registry...\n");
  is_alive = false;
}

class Registry2Monitor {
public:
  Registry2Monitor(Registry2* reg)
    : reg_(reg), n_last_reg_(0) {
    print();
  }

public:
  void print() {
    ///! update the list of registry.
    reg_->syncRegInfo();
    if (n_last_reg_ != reg_->reg_infos_.size()) {
      reg_->print();

      n_last_reg_ = reg_->reg_infos_.size();
    }
  }

private:
  Registry2* reg_;
  size_t     n_last_reg_;
};

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("ipc_clear");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::GLOG_INFO);

  if (nullptr == SharedMem::create_instance()) {
    LOG_ERROR << "ERROR Create SharedMem";
    return -1;
  }
  if ( nullptr == Registry2::create_instance()) {
    LOG_ERROR << "ERROR Create Registry2";
    return -1;
  }
  signal(SIGINT, termial);

  auto monitor = new Registry2Monitor(Registry2::instance());
  LOG_WARNING << "Ready to monitoring the Registry2...\n";
  TICKER_INIT(std::chrono::seconds);
  while (is_alive) {
    monitor->print();
    // printf("\n\n\n");

    TICKER_CONTROL(1, std::chrono::seconds);
  }

  delete monitor;
  monitor = nullptr;

  Registry2::destroy_instance();
  SharedMem::destroy_instance();

  ///! Clear the all of resource.
  pid_t pid = fork();
  if ( 0 == pid) {
    system("rosrun agile_apps ipc_clear");
    exit(0);
  }

  int status = 0;
  wait(&status);

  LOG_INFO << "Exit the Registry2Monitor";
  google::ShutdownGoogleLogging();
  return 0;
}
