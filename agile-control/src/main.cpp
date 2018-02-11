/*
 * main.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */
#include <iostream>
#include <ros/ros.h>
#include "gait/gait_manager.h"
#include <foundation/cfg_reader.h>
#include <foundation/auto_instanceor.h>

#include <Eigen/Dense>

void __auto_inst(const MiiString& __p, const MiiString& __type) {
  LOG_INFO << "Create instance(" << __type << " " << __p;
  if (!AutoInstanceor::instance()->make_instance(__p, __type)) {
    LOG_WARNING << "Create instance(" << __type << " " << __p << ") fail!";
  }
}

void create_system_instance() {
  // Just for test
  auto auto_inst = AutoInstanceor::create_instance("/home/bibei/Workspaces/qr_ws/devel/lib/libqr_control_repository.so");
  if (nullptr == auto_inst) {
    LOG_FATAL << "Create the singleton 'AutoInstanceor' has failed.";
  }
  auto cfg = MiiCfgReader::create_instance("/home/bibei/Workspaces/qr_ws/src/qr-control/config/control_config.xml");
  if (!cfg) {
    LOG_FATAL << "The MiiCfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before GaitManager::init()";
  }
  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", __auto_inst);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test_qr");
  ros::NodeHandle nh;

  qr_control::GaitManager* _manager
    = qr_control::GaitManager::create_instance();

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

