/*
 * data_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_DATA_LEG_H_
#define INCLUDE_ROBOT_LEG_DATA_LEG_H_

#include <foundation/label.h>
#include <foundation/utf.h>

#include <Eigen/Dense>
#include <atomic>

namespace agile_control {

class DataLeg: public Label {
public:
  DataLeg();
  virtual bool auto_init() override;
  virtual ~DataLeg();

public:
  LegType leg_type();

  double         foot_force()               const;
  const double&  foot_force_const_ref()     const;
  const double*  foot_force_const_pointer() const;

  EVX        joint_position()               const;
  const EVX& joint_position_const_ref()     const;
  const EVX* joint_position_const_pointer() const;

  EVX        joint_velocity()               const;
  const EVX& joint_velocity_const_ref()     const;
  const EVX* joint_velocity_const_pointer() const;

  EVX        joint_torque()               const;
  const EVX& joint_torque_const_ref()     const;
  const EVX* joint_torque_const_pointer() const;

  // Only get the last command.
  EVX        joint_command()               const;
  EVX&       joint_command_const_ref()     const;
  EVX*       joint_command_const_pointer() const;

  JntCmdType  joint_mode()               const;
  JntCmdType& joint_mode_const_ref()     const;
  JntCmdType* joint_mode_const_pointer() const;

///! The interfaces for command leg.
public:
  /*!
   * @brief Set the joint command for specialy joint.
   * @param jnt  The target joint type, If is JntType::N_JNTS, then setting all
   *             joint the same value.
   * @param val  The command value
   * @param vals The command values for the all of joints.
   */
  void joint_command(JntType jnt, double val);
  void joint_command(const EVX& vals);

  /*!
   * @brief change the command mode of joint.
   */
  void joint_mode(JntCmdType);

protected:
  ///! The type of this leg, reference to utf.h
  LegType                leg_type_;
  ///! The data of the foot's force sensor.
  const double*          foot_force_;
  ///! The vector of joint position which size is 3.
  const Eigen::VectorXd* jnt_pos_[JntDataType::N_JNT_DATA_TYPES];
  ///! The vector of joint command.
  JntCmdType*       jnt_mode_;
  Eigen::VectorXd*  jnt_cmd_;
  std::atomic_bool* jnt_cmd_flag_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_DATA_LEG_H_ */
