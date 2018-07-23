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
  MiiControl();
  /*!
   * @brief STEP 2:
   *        Create the all of singleton in our system, this method will be
   *        called before the @init() after the @prev_init(),
   *        If something was wrong, return NOTHING, SHUTDOWN the process directly.
   */
  virtual void create_system_singleton() override;
  /*!
   * @brief STEP 3:
   *        Create the all of instance in the configure file using the given
   *        Callback method or the default Callback @auto_inst_cb(), If you
   *        want to move the default callback to a self-define callback, calling
   *        the @mv_auto_inst_cb()
   */
  // virtual void auto_inst() override;
  /*!
   * @brief STEP 4:
   *        This function will be called lastly after the @init(), it must
   *        be completed the process of AutoInst and register the all of
   *        thread what our system need.
   */
  virtual bool init() override;

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
  ///! NOTE: This variable MUST BE initialize in the constructor of sub-class
  std::string          root_control_;
  class PolicyManager* policy_manager_;

  bool                      alive_;
  std::chrono::milliseconds tick_interval_;

};

} /* namespace qr_control */

#endif /* INCLUDE_MII_CONTROL_H_ */
