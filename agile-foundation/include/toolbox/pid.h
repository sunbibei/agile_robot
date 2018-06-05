/*
 * pid.h
 *
 *  Created on: Mar 8, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_TOOLBOX_PID_H_
#define INCLUDE_TOOLBOX_PID_H_

#include <string>

class Pid {
public:
  Pid(const std::string& _name);
  virtual ~Pid();

public:
  /*!
   * @brief This method offer the interface to change the gains in the runtime.
   * @param Kp    The proportional gain.
   * @param Ki    The integral gain.
   * @param Kd    The derivative gain.
   */
  void gains(double Kp, double Ki, double Kd, double i_min = -200, double i_max = 200);

  /*!
   * @brief Set the target.
   */
  void target(double);

  /*!
   * @brief Set the epsilon for stability.
   */
  void epsilon(double, size_t lapse = 10);
  /*!
   * @brief Set the limit for joints TODO
   */
  void limits(double x_min, double x_max, double u_min, double u_max);
  /*!
   * @brief Set the PID error and compute the PID command with nonuniform time
   * step size. The derivative error is computed from the change in the error
   * and the timestep \c dt.
   *
   * @param _x  State since last call
   * @param _u  Command, range from -5000 to 5000
   *
   * @returns whether is a valid command value or not.
   */
  bool control(double _x, double& _u);

protected:
  /*!
   * @brief This method will judge whether is stable or not.
   * @returns True if the system is stable, or return false.
   */
  bool stability(double _x, double& _u);

  /*!
   * @brief This method will compute the command through PID
   * @returns True if everything is OK, or return false.
   */
  void compute(double _x, double _e, double& _u);

  /*!
   * @brief This method should be sure the safety of the controlled system.
   */
  void safety_control(double _x, double& _u);

protected:
  ///! the name of PID controller
  std::string name_;
  ///! The gains.
  class Gains*  gains_;
  ///! This structure saves the variety of errors.
  class Errors* errors_;
  ///! This structure saves the variety of limit.
  class Limits*      limits_;
  ///! This structure saves the time control variety.
  class TimeControl* time_control_;
  ///! The stability classifier.
  class Stability*   stability_;
};

#endif /* INCLUDE_TOOLBOX_PID_H_ */
