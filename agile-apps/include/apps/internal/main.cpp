/*
 * test_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

// #define CHECK_INST
#ifdef  CHECK_INST
#include <foundation/label.h>
#endif

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("agile_apps");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "agile_apps");

  if (nullptr == WRAPPER::create_instance())
    LOG_FATAL << "Can't get the instance of RosWrapper!";
  WRAPPER::instance()->start();

  // Waiting for shutdown by user
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  WRAPPER::destroy_instance();
#ifdef  CHECK_INST
  Label::printfEveryInstance();
#endif

  // clear the shared memory
  system("rosrun agile_apps ipc_clear");
  LOG_INFO << "The shutdown of agile-apps has finished... ...";
  google::ShutdownGoogleLogging();
  return 0;
}
