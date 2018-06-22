/*
 * timer.cpp
 *
 *  Created on: Dec 22, 2017
 *      Author: bibei
 */

#include <toolbox/time_control.h>

///! cancel the namespace
// namespace middleware {

TimeControl::TimeControl(bool _s)
  : INVALID_TIME_POINT(std::chrono::high_resolution_clock::time_point::max()) {
  clear();
  if (_s) start();
}

void TimeControl::clear() {
  curr_update_t_ = INVALID_TIME_POINT;
  last_update_t_ = INVALID_TIME_POINT;
  dt_ = 0;
  t0_ = INVALID_TIME_POINT;
  t1_ = INVALID_TIME_POINT;
}

bool TimeControl::running() {
  return (last_update_t_ != INVALID_TIME_POINT);
}

void TimeControl::start() {
  curr_update_t_ = std::chrono::high_resolution_clock::now();
  last_update_t_ = curr_update_t_;
  t0_            = curr_update_t_;
}

/*!
 * @brief The duration (in us)
 */
int64_t TimeControl::dt_us() {
  curr_update_t_ = std::chrono::high_resolution_clock::now();
  dt_            = std::chrono::duration_cast<std::chrono::microseconds>
      (curr_update_t_ - last_update_t_).count();
  last_update_t_ = curr_update_t_;
  return dt_;
}

/*!
 * @brief The duration (in ms)
 */
int64_t TimeControl::dt() {
//  curr_update_t_ = std::chrono::high_resolution_clock::now();
//  dt_            = std::chrono::duration_cast<std::chrono::milliseconds>
//      (curr_update_t_ - last_update_t_).count();
//  last_update_t_ = curr_update_t_;
  return dt_us()/1000.0;
}

double TimeControl::dt_s() {
  return dt()/1000.0;
}

int64_t TimeControl::span() {
  int64_t span;
  t1_ = std::chrono::high_resolution_clock::now();
  span= std::chrono::duration_cast<std::chrono::milliseconds>
                (t1_ - t0_).count();

  return span;
}

void TimeControl::stop(int64_t* span) {
  t1_ = std::chrono::high_resolution_clock::now();
  if (span) *span= std::chrono::duration_cast<std::chrono::milliseconds>
                      (t1_ - t0_).count();

  clear();
}

// } /* namespace middleware */
