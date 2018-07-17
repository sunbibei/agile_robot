/*
 * gait_base.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include "gait/gait_base.h"
#include "gait/gait_manager.h"
#include <foundation/cfg_reader.h>


namespace agile_control {
StateMachineBase::StateMachineBase()  { }

StateMachineBase::~StateMachineBase() { }

GaitBase::GaitBase(const std::string& _l)
  : Label(_l), state_machine_(nullptr) {
  // Register gait class pointer into manager.
  GaitManager::instance()->add(this);
}

bool GaitBase::auto_init() {
  auto cfg = CfgReader::instance();
  cfg->get_value(getLabel(), "name", gait_name_);
  return true;
}

GaitBase::~GaitBase() {
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

bool GaitBase::canSwitch() {
  LOG_ERROR << "Call the base method 'canSwitch'";
  return true;
}

/*!
 * @brief This method will be called when the gait is activated and
 *        ready to run. If return false, it don't be running, instead
 *        of nothing to run, until the user activate the next gait.
 */
bool GaitBase::starting() /*= 0*/ {
  return true;
}

/*!
 * @brief This method will be called when the gait is stopping and switch
 *        to the other gait.
 */
void GaitBase::stopping() /*= 0*/ {
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
void GaitBase::prev_tick()  {
  ;
}

/*!
 * @brief This method will be called after the callback of state.
 */
void GaitBase::post_tick() {
  ;
}

} /* namespace qr_control */
