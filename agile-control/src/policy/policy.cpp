/*
 * gait_base.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include "foundation/cfg_reader.h"
#include "policy/policy.h"
#include "policy/policy_manager.h"


namespace agile_control {

StateMachineBase::StateMachineBase()  { }

StateMachineBase::~StateMachineBase() { }

Policy::Policy()
  : Label(""), state_machine_(nullptr) {
  // Register gait class pointer into manager.
  PolicyManager::instance()->add(this);
}

bool Policy::auto_init() {
  auto cfg = CfgReader::instance();
  cfg->get_value(getLabel(), "name", gait_name_);
  return true;
}

Policy::~Policy() {
  // Nothing to do here.
}

//void GaitBase::update() {
//  checkState();
//  if (nullptr == state_machine()) {
//    LOG_WARNING << "No StateMachine!";
//    return;
//  }
//  state_machine()->operator ()();
//}

bool Policy::canSwitch() {
  LOG_ERROR << "Call the base method 'canSwitch'";
  return true;
}

/*!
 * @brief This method will be called when the gait is activated and
 *        ready to run. If return false, it don't be running, instead
 *        of nothing to run, until the user activate the next gait.
 */
bool Policy::starting() /*= 0*/ {
  return true;
}

/*!
 * @brief This method will be called when the gait is stopping and switch
 *        to the other gait.
 */
void Policy::stopping() /*= 0*/ {
  ; // Nothing to do under the default action.
}

//void GaitBase::checkState() {
//  LOG_ERROR << "Call the base method 'checkState'";
//}

//StateMachineBase* GaitBase::state_machine() {
//  LOG_ERROR << "Call the base method 'checkState'";
//  return nullptr;
//}

/*!
 * @brief This method will be called before the callback of state.
 */
void Policy::prev_tick()  {
  ;
}

/*!
 * @brief This method will be called after the callback of state.
 */
void Policy::post_tick() {
  ;
}

} /* namespace qr_control */
