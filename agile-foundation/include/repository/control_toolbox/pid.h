/*
 * pid.h
 *
 *  Created on: Nov 2, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_
#define INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_

#include "foundation/utf.h"

#include <fstream>
#include <chrono>

namespace agile_robot {

class Pid {
public:
  Pid(const std::string& prefix);
  virtual ~Pid();

public:
  /*!
   * @brief This method offer the interface to change the gains in the runtime.
   * @param Kp    The proportional gain.
   * @param Ki    The integral gain.
   * @param Kd    The derivative gain.
   */
  void setPid(double Kp, double Ki, double Kd);

  void setTarget(short);

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
  bool control(short _x, short& _u);

protected:
  /*!
   * @brief This method will judge whether is stable or not.
   * @returns True if the system is stable, or return false.
   */
  bool stability(short _x, short& _u);

  /*!
   * @brief This method will compute the command through PID
   * @returns True if everything is OK, or return false.
   */
  void compute(short _x, short _e, short& _u);

  /*!
   * @brief This method should be sure the safety of the controlled system.
   */
  void safety_control(short _x, short& _u);

protected:
  ///! the name of PID controller
  std::string name_;
  /*!
   * The gain of PID controller read from the configure file under the @prefix
   * tag when the constructor been called. The content of configure as follow:
   * <pid_0  gains="Kp Ki Kd" limits = "iMin iMax error_threshold" />
   * <pid_0  node_id="0x02" leg="fl" jnt="yaw"  gains="1.20 0.10 0.10" limits="0 200 10" />
   */
  class Gains*  gains_;
  ///! This structure saves the variety of errors.
  class Errors* errors_;
  ///! This structure saves the variety of limit.
  class Limits*      limits_;
  ///! This structure saves the time control variety.
  class TimeControl* time_control_;
  ///! The stability classifier.
  class Stability*   stability_;

  ///! Just for debug
  bool                   __d_debug_;
  std::ofstream          __d_ofd_;
  std::vector<short>     __d_u_buf_;
  std::vector<short>     __d_x_buf_;
  std::vector<short>     __d_mu_buf_;
  std::vector<double>    __d_t_buf_;
  std::vector<double>    __d_e_buf_;
  std::vector<double>    __d_p_term_;
  std::vector<double>    __d_i_term_;
  std::vector<double>    __d_d_term_;
  void __save_everything_to_file();
  void __save_data_into_buf(double, short, short, short, double, double, double);
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_CONTROL_TOOLBOX_PID_H_ */
