/*
 * test_registry2.cpp
 *
 *  Created on: Jul 9, 2018
 *      Author: bibei
 */

#include "foundation/thread/threadpool.h"
#include "repository/registry2.h"
#include "foundation/utf.h"

#include <Eigen/Dense>
#include <iostream>
#include <chrono>

using namespace agile_robot;

// #define PUB
#define SUB


int main(int argc, char* argv[]) {
  google::InitGoogleLogging("test-registry2");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;
  google::SetStderrLogging(google::GLOG_INFO);

  if ( nullptr == ThreadPool::create_instance()) {
    std::cout << "ERROR Create ThreadPool" << std::endl;
    return -1;
  }
  if ( nullptr == Registry2::create_instance()) {
    std::cout << "ERROR Create Registry2" << std::endl;
    return -1;
  }

  auto registry = Registry2::instance();

#ifdef PUB
  // create the resource.
  Eigen::VectorXd vec_d;
  vec_d.resize(100);
  vec_d.fill(0.0);

  Eigen::VectorXd delta;
  delta.resize(100);
  delta.fill(0.01);

  std::cout << "The init of resource: \n" << vec_d.transpose() << std::endl;
  registry->publish("test-vec-d", &vec_d);
#endif

#ifdef SUB
  const Eigen::VectorXd& vec_d = *(registry->subscribe<Eigen::VectorXd>("test-vec-d", 100));
#endif

  ThreadPool::instance()->start();
  TICKER_INIT(std::chrono::milliseconds);
  while (true) {
#ifdef PUB
    vec_d += delta;
#endif
    LOG_INFO << "\n";
    for (int i = 0; i < vec_d.size(); ++i) {
      printf("%5.02f ", vec_d(i));
    }
    printf("\n");

    TICKER_CONTROL(500, std::chrono::milliseconds);
  }

  Registry2::destroy_instance();
  return 0;
}

