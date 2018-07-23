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
    running_policy_(nullptr), actived_policy_(nullptr) {

}

PolicyManager::~PolicyManager() {
  // Nothing to do here.
}

/*void PolicyManager::add(Policy* gait) {
  middleware::internal::ResourceManager<GaitBase>::add(gait);

  gait_list_by_name_.insert(std::make_pair(gait->gaitName(), gait));
}*/

bool PolicyManager::init() {
  for (const auto& cg : res_list_) {
    auto g = Label::getHardwareByName<Policy>(cg->getLabel());
    policys_by_name_.insert(std::make_pair(g->name(), g));
  }

  return true;
}

void PolicyManager::tick() {
  if ((!running_policy_) && (!actived_policy_)) {
    ///! No gait be running, and no active gait.
    return;
  } else if ((!running_policy_) && (actived_policy_)) {
    ///! No gait be running, and an active gait.
    if (actived_policy_->starting()) {
      running_policy_ = actived_policy_;
      LOG_INFO << "Active the gait -- " << running_policy_->name();
    } else {
      LOG_ERROR << "The named " << actived_policy_->name()
          << " gait is starting fail.";
      actived_policy_->stopping();
      actived_policy_ = nullptr;
      return;
    }
  } else if (running_policy_ != actived_policy_) {
    if (nullptr == actived_policy_) {
      LOG_INFO << "Stop the gait -- " << running_policy_->name();
      running_policy_->stopping();
      running_policy_ = nullptr;
      return;
    }

    ///! The running gait is not the active gait.
    if (running_policy_->canSwitch()) {
      if (actived_policy_->starting()) {
        LOG_INFO << "Switch the gait pattern from " << running_policy_->name()
            << " to " << actived_policy_->name();
        running_policy_->stopping();
        running_policy_ = actived_policy_;
      } else {
        LOG_ERROR << "The named " << actived_policy_->name()
            << " gait is starting fail, the " << running_policy_->name()
            << " continue to run.";
        actived_policy_->stopping();
        actived_policy_ = nullptr;
      }
    } else
      LOG_WARNING << "Waiting to switch from "
        << running_policy_->name() << " to " << actived_policy_->name();
  } else { // runing_gait_ == active_gait_
    ; // Unreachable code
  }

  ///! prev tick
  running_policy_->prev_tick();

  ///! check state change.
  running_policy_->checkState();
  if (nullptr == running_policy_->state_machine()) {
    LOG_WARNING << "No StateMachine!";
    return;
  }
  ///! call the callbacks
  running_policy_->state_machine()->operator ()();

  ///! post tick
  running_policy_->post_tick();
}

void PolicyManager::activate(const std::string& _n) {
  if ((0 == _n.compare("null")) || (0 == _n.compare("NULL"))) {
    actived_policy_ = nullptr;
    return;
  }

  auto new_gait = policys_by_name_.find(_n);
  if (policys_by_name_.end() == new_gait) {
    LOG_WARNING << "No gait '" << _n << "' register in the gait manager.";
    return;
  }

  LOG_INFO << "Switch the current gait to " << _n;
  actived_policy_ = new_gait->second;
}

bool PolicyManager::query(const std::string& _n) {
  return (policys_by_name_.end() != policys_by_name_.find(_n));
}

void PolicyManager::print() {
  size_t N_max_name  = 0;
  for (const auto& res : res_list_) {
    if (N_max_name < res->name().size())
      N_max_name = res->name().size();
  }

  char format[128] = {0};
//printf("No. NAME     ADDR             LABEL        \n");
//printf("1   sl_test  0x55636bc68da0 ctrl.gait.walk \n");
  printf("\n");
  LOG_WARNING << "\nPolicy's table, size = " << res_list_.size()
      << "\n-------------------------------------------------------------";
  sprintf(format, "No. %%-%lds %%-18s LABEL\n", N_max_name);
  printf( format, "NAME", "ADDR");

  memset( format, 0x00, 128);
  sprintf(format, "%%-3d %%-%lds %%p %%s\n", N_max_name);

  int count = 0;
  for (const auto& res : res_list_) {
    printf(format, count++, res->name().c_str(), res, res->label_.c_str());
  }
  printf("-------------------------------------------------------------\n");
  printf("The current running gait: %s\n",
      (running_policy_ ? running_policy_->gait_name_.c_str() : "NULL"));
  printf("The current active  gait: %s\n",
      (actived_policy_ ? actived_policy_->gait_name_.c_str() : "NULL"));
  printf("-------------------------------------------------------------\n");

  LOG_WARNING;
  printf("\n");

}

} /* namespace qr_control */
