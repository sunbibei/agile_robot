/*
 * mii_control.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#include <foundation/cfg_reader.h>
#include <foundation/auto_instanceor.h>
#include <foundation/thread/threadpool.h>
#include <robot/agile_robot.h>

#include "mii_control.h"
#include "gait/gait_manager.h"

#include <thread>

namespace agile_control {

// #define MII_CTRL ("mii-control")

SINGLETON_IMPL_NO_CREATE(MiiControl)

void __auto_inst(const MiiString& _p, const MiiString& _type) {
  if (!AutoInstanceor::instance()->make_instance(_p, _type))
    LOG_ERROR << "Create instance(" << _type << " " << _p << ") fail!";
}

MiiControl* MiiControl::create_instance(const MiiString& prefix) {
  if (nullptr != instance_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    instance_ = new MiiControl(prefix);
  }
  return instance_;
}

MiiControl::MiiControl(const MiiString& _prefix)
  : prefix_tag_(_prefix), alive_(true),
    tick_interval_(20) {
}

MiiControl::~MiiControl() {
  alive_ = false; // exit the thread
  GaitManager::instance()->destroy_instance();
}

bool MiiControl::init() {
  create_system_instance();

  auto cfg = MiiCfgReader::instance();
  if (nullptr == cfg) {
    LOG_FATAL << "The MiiCfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before MiiRobot::init()";
  }
  double hz = 50;
  cfg->get_value(prefix_tag_, "frequency", hz);
  tick_interval_ = std::chrono::milliseconds(int(1000/hz));

  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", __auto_inst, prefix_tag_);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();

  GaitManager::instance()->init();

  MiiString act_gait;
   if (cfg->get_value(Label::make_label(prefix_tag_, "gait"), "activate", act_gait))
     activate(act_gait);

  GaitManager::instance()->print();
  return true;
}

void MiiControl::activate(const MiiString& _n) {
  GaitManager::instance()->activate(_n);
}

void MiiControl::create_system_instance() {
  if (nullptr == GaitManager::create_instance())
    LOG_FATAL << "Create the singleton 'GaitManager' has failed.";
  if (nullptr == AgileRobot::create_instance())
    LOG_FATAL << "Create the singleton 'QrRobot' has failed.";
}

void MiiControl::tick() {
  TIMER_INIT

  while (alive_) {
    if (GaitManager::instance()) GaitManager::instance()->tick();
    else break;

    TIMER_CONTROL(tick_interval_)
  }
}

} /* namespace qr_control */
