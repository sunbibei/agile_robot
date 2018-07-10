/*
 * main.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: bibei
 */

#include <string>
#include <ros/ros.h>
#include <sys/wait.h>

#include "foundation/cfg_reader.h"

void apps_launcher() {
  auto cfg = MiiCfgReader::instance();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "agile-apps");

  std::string prefix;
  std::string configure;
  if (!ros::param::get("~configure", configure)
      || !ros::param::get("~prefix", prefix) ) {
    printf("\033[0;31mNo parameter with named configure or prefix!\033[0m\n");
    return -1;
  }

  if (nullptr == MiiCfgReader::create_instance(configure)) {
    printf("\033[0;31mCreate the CfgReader fail!\033[0m\n");
    return -1;
  }

  // launch the each APPS
  apps_launcher();

  // waiting for the all of process exiting.
  int status = 0;
  wait(&status);

  // destroy the CfgReader.
  MiiCfgReader::destroy_instance();
  return 0;
}


