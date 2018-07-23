/*
 * motor.h
 *
 *  Created on: Dec 15, 2017
 *      Author: robot
 */

#ifndef INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_
#define INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_

#include "foundation/label.h"
#include <atomic>

namespace agile_robot {

class Motor: public Label {
  ///! The list of the information offers.
  friend class MotorNode;
public:
  Motor();
  virtual bool auto_init() override;

  virtual ~Motor();

public:
  void updateMotorCommand(double);

public:
  const std::string&  motor_name() const;
  const JntCmdType&   cmd_mode()   const;

public:
  ///! About the state of motor
  const short  motor_position()               const;
  const short& motor_position_const_ref()     const;
  const short* motor_position_const_pointer() const;

  const short  motor_velocity()               const;
  const short& motor_velocity_const_ref()     const;
  const short* motor_velocity_const_pointer() const;

  const short  motor_torque()                 const;
  const short& motor_torque_const_ref()       const;
  const short* motor_torque_const_pointer()   const;

  ///! About the command of motor
  const short  motor_command()               const;
  const short& motor_command_const_ref()     const;
  const short* motor_command_const_pointer() const;

protected:
  /*!
   * Interface for communication layer, friend class LegNode.
   */
  void updateMotorPosition(short);
  void updateMotorVelocity(short);
  void updateMotorTorque  (short);
  std::atomic_bool    new_command_;

protected:
  std::string         name_;
  class MotorState*   state_;
  class MotorCommand* cmd_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_RESOURCE_MOTOR_H_ */
