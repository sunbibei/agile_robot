/*
 * mii_robot.h
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_
#define INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_

#include <map>
#include <vector>
#include <chrono>

#include "foundation/app.h"
#include "foundation/utf.h"
#include "foundation/cfg_reader.h"

namespace agile_robot {

class MiiRobot : virtual public MiiApp {
public:
  /**
   * @brief Instead of the upper methods, you also use the JointManager directly.
   */
//  class JointManager*       joint_manager()     { return jnt_manager_;  }
//  const class JointManager& joint_manager_ref() { return *jnt_manager_; }

protected:
  /**
   * @brief Constructed function.
   * @param _tag         Every necessary parameters will be found in this __tag
   */
  MiiRobot();
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
  virtual bool run() override;

  /*!
   * @brief The destroying stage
   */
  virtual ~MiiRobot();

protected:
  /**
   * Given by subclass in the parameters list of the constructed function.
   * Tell MiiRobot what necessary parameters are found in @prefix_tag_.
   */
  std::string                     root_robot_;
// class Master*                   master_;
  ///! The resource
//  class JointManager*             jnt_manager_;
//  std::vector<class ForceSensor*> td_list_by_type_;
//  class ImuSensor*                imu_sensor_;

//  std::chrono::microseconds       tick_interval_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_ */
