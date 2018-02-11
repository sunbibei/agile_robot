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

#include "foundation/utf.h"

namespace middleware {

class MiiRobot {
public:
  /**
   * @brief The pure virtual function is asked to implemented by subclass.
   *        These task should be completed in the function what include but not
   *        limited to: instantiate JointManger, PropagateManager, HwManager, and
   *        especially MiiCfgReader. Throwing a fatal exception if something is wrong.
   */
  virtual void create_system_instance();

  virtual bool init(bool use_mii_control);

  virtual bool start();

  virtual void supportRegistry();
public:
  /**
   * @brief Instead of the upper methods, you also use the JointManager directly.
   */
  class JointManager* joint_manager()           { return jnt_manager_; }
  const class JointManager& joint_manager_ref() { return *jnt_manager_; }

public:
  static void auto_inst(const MiiString& __p, const MiiString& __type);

protected:
  /**
   * @brief Constructed function.
   * @param _tag         Every necessary parameters will be found in this __tag
   * @param _mii_control Is use the Mii Control
   */
  MiiRobot(const MiiString& __tag);
  virtual ~MiiRobot();

protected:
  /**
   * Given by subclass in the parameters list of the constructed function.
   * Tell MiiRobot what necessary parameters are found in @prefix_tag_.
   */
  MiiString                     prefix_tag_;
  // class Master*                 master_;
  class JointManager*           jnt_manager_;

  MiiVector<class ForceSensor*>          td_list_;
  MiiVector<class ForceSensor*>          td_list_by_type_; // type: leg
  MiiMap<MiiString, class ForceSensor*>  td_list_by_name_;

  class ImuSensor*   imu_sensor_;

  bool                      mii_ctrl_alive_;
  
  std::chrono::milliseconds tick_interval_;

private:
  bool                 use_mii_control_;
  class __RegJntRes*   jnt_reg_res_;
  class __RegForceRes* td_reg_res_;
  class __RegImuRes*   imu_reg_res_;
  ///! The helper method
  void __reg_resource_and_command(const MiiString&);

///! These methods has been deleted.
// public:
  /**
   * @brief This methods add the joint command to Joint object.
   * @param _name    The name of controlled joint
   * @param _command The read data of command
   */
  // void addJntCmd(const MiiString& _name, double _command);
  /**
   * @brief This methods add the joint command to Joint object.
   * @param _owner  The owner who owns the specific _jnt want to control
   * @param _jnt    The controlled Joint type
   * @param command The real data of command
   */
  // void addJntCmd(LegType _owner, JntType _jnt, double _command);
  /**
   * 获取Joint的名称, 位置, 速度, 力矩及JointState等数据
   */
  // void getJointNames(MiiVector<MiiString>&);
  // void getJointPositions(MiiVector<double>&);
  // void getJointVelocities(MiiVector<double>&);
  // void getJointTorques(MiiVector<double>&);
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_MII_ROBOT_H_ */
