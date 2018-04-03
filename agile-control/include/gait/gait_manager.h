/*
 * gait_manager.h
 *
 *  Created on: Nov 20, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_GAIT_MANAGER_H_
#define INCLUDE_GAIT_GAIT_MANAGER_H_

#include <foundation/internal/resource_manager.h>
#include "gait_base.h"

#include <mutex>

namespace agile_control {

class GaitManager
    : public internal::ResourceManager<GaitBase> {
  SINGLETON_DECLARE(GaitManager)

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

  // virtual void add(GaitBase*) override;

  // for Debug
  void print();

protected:
  GaitBase*                     running_gait_;
  GaitBase*                     actived_gait_;
  std::map<std::string, GaitBase*>  gait_list_by_name_;

  std::string                     prefix_tag_;
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_GAIT_MANAGER_H_ */
