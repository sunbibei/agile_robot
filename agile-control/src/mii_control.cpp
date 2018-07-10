/*
 * mii_control.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#include "foundation/cfg_reader.h"
#include "foundation/auto_instanceor.h"
#include "foundation/thread/threadpool.h"
#include "repository/registry2.h"

#include "mii_control.h"
#include "robot/agile_robot.h"
#include "gait/gait_manager.h"

namespace agile_control {

void __auto_inst(const std::string& _p, const std::string& _type) {
  if (!AutoInstanceor::instance()->make_instance(_p, _type))
    LOG_ERROR << "Create instance(" << _type << " " << _p << ") fail!";
}

MiiControl::MiiControl(const std::string& _prefix)
  : prefix_tag_(_prefix), gait_manager_(nullptr),
    alive_(true), tick_interval_(1) {
}

MiiControl::~MiiControl() {
  alive_ = false; // exit the thread
  GaitManager::destroy_instance();
  AgileRobot::destroy_instance();
  Registry2::destroy_instance();
  SharedMem::destroy_instance();
  ThreadPool::destroy_instance();
}

bool MiiControl::init() {
  create_system_instance();

  auto cfg = MiiCfgReader::instance();
  if (nullptr == cfg) {
    LOG_FATAL << "The MiiCfgReader::create_instance(MiiStringConstRef) "
        << "method must to be called by subclass before MiiRobot::init()";
  }
  double hz = 1000;
  cfg->get_value(prefix_tag_, "frequency", hz);
  tick_interval_ = std::chrono::milliseconds(int(1000/hz));
  LOG_WARNING << "MII-CONTROL: " << hz;

  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", __auto_inst, prefix_tag_);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();

  GaitManager::instance()->init();

  std::string act_gait;
   if (cfg->get_value(Label::make_label(prefix_tag_, "gait"), "activate", act_gait))
     activate(act_gait);

  GaitManager::instance()->print();
  return true;
}

bool MiiControl::start() {
  return ThreadPool::instance()->start();
}

void MiiControl::activate(const std::string& _n) {
  if (GaitManager::instance()->query(_n) || 0 == _n.compare("null"))
    GaitManager::instance()->activate(_n);
  else
    LOG_ERROR << "No such named gait in the gait manager";
}

void MiiControl::create_system_instance() {
  if (nullptr == ThreadPool::create_instance())
    LOG_FATAL << "Create the singleton 'ThreadPool' has failed.";

  if (nullptr == SharedMem::create_instance())
    LOG_FATAL << "Create the singleton 'SharedMem' has failed.";

  if (nullptr == Registry2::create_instance())
    LOG_FATAL << "Create the singleton 'Registry' has failed.";

  if (nullptr == GaitManager::create_instance())
    LOG_FATAL << "Create the singleton 'GaitManager' has failed.";

  if (nullptr == AgileRobot::create_instance())
    LOG_FATAL << "Create the singleton 'QrRobot' has failed.";
}

void MiiControl::tick() {
  TICKER_INIT(std::chrono::milliseconds);

  while (alive_) {
    GaitManager::instance()->tick();

    TICKER_CONTROL(tick_interval_, std::chrono::milliseconds);
  }
}

} /* namespace qr_control */
