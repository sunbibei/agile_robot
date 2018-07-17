/*
 * main.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */
#include "gait/gait_manager.h"
#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "mii_control.h"

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>

void __auto_inst(const std::string& __p, const std::string& __type) {
  LOG_INFO << "Create instance(" << __type << " " << __p;
  if (!AutoInstor::instance()->make_instance(__p, __type)) {
    LOG_WARNING << "Create instance(" << __type << " " << __p << ") fail!";
  }
}

void create_system_instance() {
  // Just for test
  auto auto_inst = AutoInstor::create_instance();
  if (nullptr == auto_inst) {
    LOG_FATAL << "Create the singleton 'AutoInstor' has failed.";
  }
  auto cfg = CfgReader::create_instance();
  if (!cfg) {
    LOG_FATAL << "The CfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before GaitManager::init()";
  }

  auto_inst->add_library("/home/bibei/Workspaces/qr_ws/devel/lib/libqr_control_repository.so");
  cfg->add_config("/home/bibei/Workspaces/qr_ws/src/qr-control/config/control_config.xml");

  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", __auto_inst);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "agile-control");
  ros::NodeHandle nh;

  agile_control::GaitManager* _manager
    = agile_control::GaitManager::create_instance();

  create_system_instance();

  if (!_manager) {
    LOG_ERROR << "Something is wrong!";
    return -1;
  }

  ros::Rate loop_rate(50);
  _manager->activate("creep");
  while (ros::ok()) {
    _manager->tick();
    loop_rate.sleep();
  }
  return 0;
}

