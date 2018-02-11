/*
 * joint.h
 *
 *  Created on: Jan 11, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_RESOURCES_JOINT_H_
#define INCLUDE_SYSTEM_RESOURCES_JOINT_H_

#include "foundation/label.h"
#include <atomic>

namespace middleware {

class Joint : public Label {
  friend class LegNode;
  friend class Motor;
public:
  Joint(const MiiString& l = Label::null);
  // 妥协方案
  virtual bool auto_init() override;
  ~Joint();

  const MiiString&   joint_name() const;
  const JntType&     joint_type() const;
  const LegType&     leg_type()   const;
  /**
   * Interface for user layer.
   */
  // About joint state
  double        joint_position()               const;
  const double& joint_position_const_ref()     const;
  const double* joint_position_const_pointer() const;

  double        joint_position_min()           const;
  double        joint_position_max()           const;

  double        joint_velocity()               const;
  const double& joint_velocity_const_ref()     const;
  const double* joint_velocity_const_pointer() const;

  double        joint_velocity_min()           const;
  double        joint_velocity_max()           const;

  double        joint_torque()               const;
  const double& joint_torque_const_ref()     const;
  const double* joint_torque_const_pointer() const;

  double        joint_torque_min()           const;
  double        joint_torque_max()           const;

  ///! About joint command, This is only way that the user update the joint command.
  void updateJointCommand(double);
  ///! The first value is position, the second value is velocity.
  void updateJointCommand(double, double);
  ///! Stop the current action.
  void stop();

  /*!
   * The argument index is represent the command type.
   * idx = 0 means the position command;
   * idx = 1 means the velocity command, Only when under pos-vel mode is valid;
   */
  double            joint_command(size_t idx = 0)           const;
  const double&     joint_command_const_ref(size_t idx = 0) const;
  const double*     joint_command_const_pointer()           const;

protected:
  /**
   * Interface for communication layer, friend class LegNode.
   */
  void updateJointPosition(double pos);
  // if has new command, return true and fill the Packet pointer
  std::atomic_bool new_command_; // the flag indicate whether has new command

protected:
  JntType             jnt_type_;
  LegType             leg_type_;
  MiiString           jnt_name_;
  class Motor*        joint_motor_;
  // The private data structure
  class JointState*   joint_state_;
  class JointCommand* joint_command_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_RESOURCES_JOINT_H_ */
