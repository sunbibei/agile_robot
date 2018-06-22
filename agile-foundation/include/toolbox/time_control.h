/*
 * timer.h
 *
 *  Created on: Dec 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_TOOLBOX_TIME_CONTROL_H_
#define INCLUDE_TOOLBOX_TIME_CONTROL_H_

#include <chrono>

///! cancel the namespace
// namespace middleware {

class TimeControl {
public:
  /*!
   * @brief The construct methods, where timer will call @start()
   *        when _s is true.
   */
  TimeControl(bool _s = false);

  /*!
   * @brief The timer whether is running.
   * @return return true if the timer is running.
   */
  bool running();

  /*!
   * @brief Start the timer.
   */
  void start();

  /*!
   * @brief The duration (in ms)
   */
  int64_t dt_us();

  /*!
   * @brief The duration (in ms)
   */
  int64_t dt();
  /*!
   * @brief The duration (in s)
   */
  double  dt_s();
  /*!
   * @brief Return the time span from @start (in ms)
   */
  int64_t span();

  /*!
   * @brief Stop the timer, and return timespan from start();
   * @param span The dufault is nullptr, if you don't care the timespan.
   */
  void stop(int64_t* span = nullptr);

private:
  void clear();

private:
  ///! time control (in ms)
  int64_t dt_;
  std::chrono::high_resolution_clock::time_point curr_update_t_;
  std::chrono::high_resolution_clock::time_point last_update_t_;

  ///! these variables for debug.
  std::chrono::high_resolution_clock::time_point t0_;
  std::chrono::high_resolution_clock::time_point t1_;

  const std::chrono::high_resolution_clock::time_point INVALID_TIME_POINT;
};

// } /* namespace middleware */

#endif /* INCLUDE_TOOLBOX_TIME_CONTROL_H_ */
