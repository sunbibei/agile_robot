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

#include <apps/control_wrapper.h>

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("agile_control");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "agile_control");

  MiiApp* application = ControlWrapper::create_instance("ctrl");

  if (nullptr == application)
    LOG_FATAL << "Can't get the instance of ControlWrapper!";

  application->start();

  // Waiting for shutdown by user
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  ControlWrapper::destroy_instance();

#ifdef  CHECK_INST_
  Label::printfEveryInstance();
#endif

  LOG_INFO << "The shutdown of agile-control has finished... ...";
  google::ShutdownGoogleLogging();
  return 0;
}
