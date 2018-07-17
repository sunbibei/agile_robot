/*
 * gait_manager.h
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_POLICY_POLICY_MANAGER_H_
#define INCLUDE_POLICY_POLICY_MANAGER_H_

#include "foundation/internal/resource_manager.h"
#include "policy.h"
#include <mutex>

namespace agile_control {

class PolicyManager
    : public internal::ResourceManager<Policy> {
  SINGLETON_DECLARE(PolicyManager)

public:
  /*!
   * @brief Initialization
   */
  bool init();
  /*!
   * The main function what call the active gait to control robot.
   */
  void tick();

  /*!
   * Change the active gait.
   * The string of 'null' means stop the running gait.
   */
  void activate(const std::string& gait_name);

  /*!
   * @brief Query whether is contains the given named gait.
   */
  bool query(const std::string& gait_name);

  // virtual void add(GaitBase*) override;

  // for Debug
  void print();

protected:
  Policy*                     running_gait_;
  Policy*                     actived_gait_;
  std::map<std::string, Policy*>  gait_list_by_name_;

  std::string                     prefix_tag_;
};

} /* namespace qr_control */

#endif /* INCLUDE_POLICY_POLICY_MANAGER_H_ */
