/*
 * test_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

// #define CHECK_INST_
#ifdef  CHECK_INST_
#include <foundation/label.h>
#endif

#include <apps/ros_wrapper.h>
#include <iostream>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("qr_driver");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "mii_qr");
  ros::NodeHandle nh("~");

  if (nullptr == RosWrapper::create_instance("qr.wrapper"))
    LOG_FATAL << "Can't get the instance of RosWrapper!";
  RosWrapper::instance()->start();

  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Waiting for shutdown by user
  ros::waitForShutdown();

  RosWrapper::destroy_instance();
#ifdef  CHECK_INST_
  Label::printfEveryInstance();
#endif
  LOG_INFO << "The shutdown of qr_driver has finished... ...";
  google::ShutdownGoogleLogging();
  return 0;
}
