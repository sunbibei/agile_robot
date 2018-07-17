/*
 * mii_control.h
 *
 *  Created on: Nov 30, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MII_CONTROL_H_
#define INCLUDE_MII_CONTROL_H_

#include <chrono>
#include "foundation/utf.h"
#include "foundation/app.h"

namespace agile_control {

class MiiControl : virtual public MiiApp {
public:
  /*!
   * @brief Switch to the different gait mode. This action is sync, the method
   *        is not change the gait right now, but add the gait mode into the
   *        ready queue, and switch will be completed in a appropriate opportunity.
   * @param The default value "null" mean to stop the running gait.
   */
  virtual void activate(const std::string& _gn = "null");

protected:
  /**
   * @brief Constructed function.
   * @param _tag        Every necessary parameters will be found in this __tag
   */
  MiiControl(const std::string& __tag);
  virtual bool init() override;
  /**
   * @brief The pure virtual function is asked to implemented by subclass.
   *        These task should be completed in the function what include but not
   *        limited to: instantiate JointManger, PropagateManager, HwManager, and
   *        especially CfgReader. Throwing a fatal exception if something is wrong.
   */
  virtual void create_system_instance() override;

  /*!
   * @brief The running stage, ONLY this method, returned immediately.
   */
  // using the default implemented.
  // virtual bool run() override;

  /*!
   * @brief The destroying stage
   */
  virtual ~MiiControl();

  /*!
   * @brief Starting to work
   */
  virtual void tick();

protected:
  std::string        prefix_tag_;
  class GaitManager* gait_manager_;

  bool                      alive_;
  std::chrono::milliseconds tick_interval_;

};

} /* namespace qr_control */

#endif /* INCLUDE_MII_CONTROL_H_ */
