/*
 * pd.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

// #define CHECK_INST_
#ifdef  CHECK_INST_
#include <foundation/label.h>
#endif

#include "apps/pd_wrapper.h"

int main(int argc, char* argv[]) {
  google::InitGoogleLogging("mii_apps");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "mii_apps");
  ros::NodeHandle nh("~");

  std::cout << "NS: " << nh.getNamespace() << std::endl;

  if (nullptr == PdWrapper::create_instance("pd"))
    LOG_FATAL << "Can't get the instance of RosWrapper!";
  PdWrapper::instance()->start();

  ros::AsyncSpinner spinner(3);
  spinner.start();

  // Waiting for shutdown by user
  ros::waitForShutdown();

  PdWrapper::destroy_instance();
#ifdef  CHECK_INST_
  Label::printfEveryInstance();
#endif

  // clear the shared memory
  system("rosrun agile_apps ipc_clear");
  LOG_INFO << "The shutdown of agile-apps has finished... ...";
  google::ShutdownGoogleLogging();
  return 0;
}
