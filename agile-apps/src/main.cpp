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

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("agile_apps");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "mii_agile");
  ros::NodeHandle nh("~");
  std::string prefix;
  if (!ros::param::get("~prefix", prefix)) {
    LOG_FATAL << "Could not found the 'prefix' parameter, Did you forget point this parameter.";
    return -1;
  }

  if (nullptr == RosWrapper::create_instance(prefix))
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
  LOG_INFO << "The shutdown of agile-apps has finished... ...";
  google::ShutdownGoogleLogging();
  return 0;
}
