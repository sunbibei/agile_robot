/*
 * mii_control.h
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MII_CONTROL_H_
#define INCLUDE_MII_CONTROL_H_

#include <chrono>
#include <foundation/utf.h>

namespace agile_control {

class MiiControl {
  SINGLETON_DECLARE(MiiControl, const MiiString&)

public:
  /*!
   * @brief This method will completed the initialization for MII Control.
   */
  virtual bool init();

  /*!
   * @brief Starting to work
   */
  virtual void tick();

  /*!
   * @brief Switch to the different gait mode. This action is sync, the method
   *        is not change the gait right now, but add the gait mode into the
   *        ready queue, and switch will be completed in a appropriate opportunity.
   * @param The default value "null" mean to stop the running gait.
   */
  virtual void activate(const MiiString& _gn = "null");

protected:
  /*!
   * @brief The all of singleton has created.
   */
  virtual void create_system_instance();

protected:
  MiiString prefix_tag_;

  bool alive_;
  std::chrono::milliseconds tick_interval_;

};

} /* namespace qr_control */

#endif /* INCLUDE_MII_CONTROL_H_ */
