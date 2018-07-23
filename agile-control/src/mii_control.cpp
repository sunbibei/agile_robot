/*
 * mii_control.cpp
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/thread/threadpool.h"
#include "foundation/registry/registry.h"
#include "foundation/registry/registry2.h"

#include "robot/agile_robot.h"
#include "policy/policy_manager.h"
#include "mii_control.h"

namespace agile_control {

MiiControl::MiiControl()
  : root_control_(""), policy_manager_(nullptr),
    alive_(true), tick_interval_(1) {
  ; // Nothing to do here.
}

MiiControl::~MiiControl() {
  alive_ = false; // exit the thread
  PolicyManager::destroy_instance();
  AgileRobot::destroy_instance();
  Registry2::destroy_instance();
  SharedMem::destroy_instance();
}

void MiiControl::create_system_singleton() {
  MiiApp::create_system_singleton();

  if (nullptr == SharedMem::create_instance())
    LOG_FATAL << "Create the singleton 'SharedMem' has failed.";

  if (nullptr == Registry::create_instance())
    LOG_FATAL << "Create the singleton 'Registry' has failed.";

  if (nullptr == Registry2::create_instance())
    LOG_FATAL << "Create the singleton 'Registry' has failed.";

  if (nullptr == PolicyManager::create_instance())
    LOG_FATAL << "Create the singleton 'PolicyManager' has failed.";

  if (nullptr == AgileRobot::create_instance())
    LOG_FATAL << "Create the singleton 'QrRobot' has failed.";
}

bool MiiControl::init() {
  auto cfg = CfgReader::instance();

  double hz = 1000;
  cfg->get_value(root_control_, "frequency", hz);
  tick_interval_ = std::chrono::milliseconds(int(1000/hz));
  LOG_WARNING << "MII-CONTROL: " << hz << "Hz";

  policy_manager_ = PolicyManager::instance();
  policy_manager_->init();

  std::string policy;
  if (cfg->get_value(Label::make_label(root_control_, "gait"), "activate", policy))
    activate(policy);

  // registry the thread
  ThreadPool::instance()->add("mii-control", &MiiControl::tick, this);

  // Just for debug
  policy_manager_->print();
  Label::printfEveryInstance();
  return true;
}

//bool MiiControl::run() {
//  return ThreadPool::instance()->start();
//}

void MiiControl::activate(const std::string& _n) {
  if (policy_manager_->query(_n) || 0 == _n.compare("null"))
    policy_manager_->activate(_n);
  else
    LOG_ERROR << "No such named gait in the gait manager";
}

void MiiControl::tick() {
  TICKER_INIT(std::chrono::milliseconds);

  while (alive_) {
    policy_manager_->tick();

    TICKER_CONTROL(tick_interval_, std::chrono::milliseconds);
  }
}

} /* namespace qr_control */
