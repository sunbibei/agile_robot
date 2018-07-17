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

namespace agile_robot {

class MiiRobot : virtual public MiiApp {
public:
  /**
   * @brief Instead of the upper methods, you also use the JointManager directly.
   */
  class JointManager* joint_manager()           { return jnt_manager_; }
  const class JointManager& joint_manager_ref() { return *jnt_manager_; }

protected:
  /**
   * @brief Constructed function.
   * @param _tag         Every necessary parameters will be found in this __tag
   */
  MiiRobot(const std::string& __tag);
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
  virtual bool run() override;

  /*!
   * @brief The destroying stage
   */
  virtual ~MiiRobot();

protected:
  /**
   * @brief Support the Registry2
   */
  virtual void supportRegistry2();

private:
  void __reg_resource_and_command();

protected:
  /**
   * Given by subclass in the parameters list of the constructed function.
   * Tell MiiRobot what necessary parameters are found in @prefix_tag_.
   */
  std::string                     prefix_tag_;
  // class Master*                 master_;
  class JointManager*             jnt_manager_;

  std::vector<class ForceSensor*>            td_list_;
  std::vector<class ForceSensor*>            td_list_by_type_; // type: leg
  std::map<std::string, class ForceSensor*>  td_list_by_name_;

  class ImuSensor*   imu_sensor_;
  
  std::chrono::microseconds tick_interval_;


private:
  bool is_alive;
  class __RegJntRes*   jnt_reg_res_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_ */
