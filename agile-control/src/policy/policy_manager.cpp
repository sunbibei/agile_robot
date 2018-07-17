/*
 * gait_manager.cpp
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#include "foundation/cfg_reader.h"
#include "policy/policy_manager.h"

namespace agile_control {

SINGLETON_IMPL(PolicyManager)

PolicyManager::PolicyManager()
  : internal::ResourceManager<Policy>(),
    running_gait_(nullptr), actived_gait_(nullptr) {

}

PolicyManager::~PolicyManager() {
  // Nothing to do here.
}

/*void PolicyManager::add(Policy* gait) {
  middleware::internal::ResourceManager<GaitBase>::add(gait);

  gait_list_by_name_.insert(std::make_pair(gait->gaitName(), gait));
}*/

bool PolicyManager::init() {
  for (const auto& g : res_list_) {
    gait_list_by_name_.insert(std::make_pair(g->name(), g));
  }

  return true;
}

void PolicyManager::tick() {
  if ((!running_gait_) && (!actived_gait_)) {
    ///! No gait be running, and no active gait.
    return;
  } else if ((!running_gait_) && (actived_gait_)) {
    ///! No gait be running, and an active gait.
    if (actived_gait_->starting()) {
      running_gait_ = actived_gait_;
      LOG_INFO << "Active the gait -- " << running_gait_->name();
    } else {
      LOG_ERROR << "The named " << actived_gait_->name()
          << " gait is starting fail.";
      actived_gait_->stopping();
      actived_gait_ = nullptr;
      return;
    }
  } else if (running_gait_ != actived_gait_) {
    if (nullptr == actived_gait_) {
      LOG_INFO << "Stop the gait -- " << running_gait_->name();
      running_gait_->stopping();
      running_gait_ = nullptr;
      return;
    }

    ///! The running gait is not the active gait.
    if (running_gait_->canSwitch()) {
      if (actived_gait_->starting()) {
        LOG_INFO << "Switch the gait pattern from " << running_gait_->name()
            << " to " << actived_gait_->name();
        running_gait_->stopping();
        running_gait_ = actived_gait_;
      } else {
        LOG_ERROR << "The named " << actived_gait_->name()
            << " gait is starting fail, the " << running_gait_->name()
            << " continue to run.";
        actived_gait_->stopping();
        actived_gait_ = nullptr;
      }
    } else
      LOG_WARNING << "Waiting to switch from "
        << running_gait_->name() << " to " << actived_gait_->name();
  } else { // runing_gait_ == active_gait_
    ; // Unreachable code
  }

  ///! prev tick
  running_gait_->prev_tick();

  ///! check state change.
  running_gait_->checkState();
  if (nullptr == running_gait_->state_machine()) {
    LOG_WARNING << "No StateMachine!";
    return;
  }
  ///! call the callbacks
  running_gait_->state_machine()->operator ()();

  ///! post tick
  running_gait_->post_tick();
}

void PolicyManager::activate(const std::string& gait_name) {
  if ((0 == gait_name.compare("null")) || (0 == gait_name.compare("NULL"))) {
    actived_gait_ = nullptr;
    return;
  }

  auto new_gait = gait_list_by_name_.find(gait_name);
  if (gait_list_by_name_.end() == new_gait) {
    LOG_WARNING << "No gait '" << gait_name << "' register in the gait manager.";
    return;
  }

  LOG_INFO << "Switch the current gait to " << gait_name;
  actived_gait_ = new_gait->second;
}

bool PolicyManager::query(const std::string& gait_name) {
  for (auto hw : res_list_) {
    if (0 == gait_name.compare(hw->gait_name_))
      return true;
  }

  return false;
}

void PolicyManager::print() {
  if (_DEBUG_INFO_FLAG) {
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
    LOG_WARNING << "NAME\t\tLABEL\t\tADDR";
    for (auto hw : res_list_) {
      LOG_INFO << hw->gait_name_ << "\t" << hw->getLabel() << "\t" << hw;
    }
    LOG_WARNING << "-------------------------------------------------------------";
    LOG_INFO << "The current running gait: " << (running_gait_ ? running_gait_->gait_name_ : "NULL");
    LOG_INFO << "The current active  gait: " << (actived_gait_  ? actived_gait_->gait_name_  : "NULL");
    LOG_WARNING << "+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+";
  }
}

} /* namespace qr_control */
