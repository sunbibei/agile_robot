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

#include <sys/wait.h>
#include <apps/robot_wrapper.h>

static bool is_alive = true;

void termial(int signo) {
  LOG_WARNING << "Ready to shutdown the agile-robot...";
  is_alive = false;
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("agile_robot");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "agile_robot");

  MiiApp* application = RobotWrapper::create_instance("leg");

  if (nullptr == application)
    LOG_FATAL << "Can't get the instance of RobotWrapper!";

  application->start();

  // Waiting for shutdown by user
  // signal(SIGINT, termial);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  ros::waitForShutdown();

  RobotWrapper::destroy_instance();

#ifdef  CHECK_INST_
  Label::printfEveryInstance();
#endif

  LOG_INFO << "The shutdown of agile-apps has finished... ...";
  google::ShutdownGoogleLogging();
  return 0;
}
