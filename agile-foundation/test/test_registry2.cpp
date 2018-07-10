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

// using namespace agile_robot;

// #define PUB
#define SUB


int main1(int argc, char* argv[]) {
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

  Eigen::MatrixXd mat_d;
  mat_d.resize(20, 20);
  mat_d.fill(0.0);

  Eigen::MatrixXd mat_d_delta;
  mat_d_delta.resize(20, 20);
  mat_d_delta.fill(0.1);

  Eigen::MatrixXi mat_i;
  mat_i.resize(20, 20);
  mat_i.fill(0);

  Eigen::MatrixXi mat_i_delta;
  mat_i_delta.resize(20, 20);
  mat_i_delta.fill(1);

  // create the resource.
  Eigen::VectorXd vec_d; // test for vectorxd
  vec_d.resize(3);
  vec_d.fill(0.0);

  Eigen::VectorXd delta;
  delta.resize(3);
  delta.fill(0.01);

  Eigen::VectorXi vec_i; // test for vectorxi
  vec_i.resize(3);
  vec_i.fill(0.0);

  Eigen::VectorXi delta_i;
  delta_i.resize(3);
  delta_i.fill(1);

  double res_d = 0.0; // test for double
  int    res_i = 100; // test for int

#ifdef PUB
  registry->publish("test-mat-d", &mat_d);
  registry->publish("test-vec-d", &vec_d);
  registry->publish("test-res-d", &res_d);
  registry->subscribe("test-mat-i", &mat_i);
  registry->subscribe("test-vec-i", &vec_i);
  registry->subscribe("test-res-i", &res_i);
#endif

#ifdef SUB
  registry->subscribe("test-mat-d", &mat_d);
  registry->subscribe("test-vec-d", &vec_d);
  registry->subscribe("test-res-d", &res_d);
  registry->publish("test-mat-i", &mat_i);
  registry->publish("test-vec-i", &vec_i);
  registry->publish("test-res-i", &res_i);
#endif

  registry->print();

  ThreadPool::instance()->start();
  TICKER_INIT(std::chrono::milliseconds);
  while (true) {
#ifdef PUB
    vec_d += delta;
    res_d += 0.001;
    mat_d += mat_d_delta;
#endif

#ifdef SUB
    vec_i += delta_i;
    res_i += 100;
    mat_i += mat_i_delta;
#endif

    LOG_WARNING << "\n" << mat_d;
    LOG_ERROR << "\n" << mat_i;
//    LOG_INFO << "test-vec-d:";
//    for (int i = 0; i < vec_d.size(); ++i) {
//      printf("%6.02f ", vec_d(i));
//    }
//    printf("\n");
//
//    LOG_WARNING << "test-vec-i:";
//    for (int i = 0; i < vec_i.size(); ++i) {
//      printf("%6d ", vec_i(i));
//    }
//    printf("\n");
//
//    LOG_INFO << "test-res-d:";
//    printf("%6.03f\n", res_d);
//
//    LOG_WARNING << "test-res-i:";
//    printf("%6d\n", res_i);

    if (1000 == res_i) {
      registry->print();
    }

    TICKER_CONTROL(500, std::chrono::milliseconds);
  }

  Registry2::destroy_instance();
  return 0;
}

