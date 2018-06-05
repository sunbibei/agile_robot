/*
 * pid.cpp
 *
 *  Created on: Mar 8, 2018
 *      Author: bibei
 */

#include "toolbox/pid.h"
#include "toolbox/time_control.h"
#include "foundation/utf.h"

#include <cmath>
#include <vector>
#include <boost/algorithm/clamp.hpp>

const short INVALID_VALUE = 0x8888;

// Output file description for debug
const size_t BUF_RESERVE_SIZE = 1024;

struct Gains {
  // Optional constructor for passing in values
  Gains(double p, double i, double d)
    : p_gain_(p), i_gain_(i), d_gain_(d),
      i_max_(0), i_min_(0)
  {}
  // Default constructor
  Gains()
    : p_gain_(0.0), i_gain_(0.0), d_gain_(0.0),
      i_max_(0), i_min_(0)
  {}
  ///! Proportional gain.
  double p_gain_;
  ///! Integral gain.
  double i_gain_;
  ///! Derivative gain.
  double d_gain_;
  ///! Maximum allowable integral iterm.
  double i_max_;
  ///! Minimum allowable integral iterm.
  double i_min_;
};

struct Errors {
  Errors()
  : p_error_last_(0), p_error_(0), i_error_(0), d_error_(0),
    tmp_d_error_(0), first_compute_(true) { }
  ///! Save position state for derivative state calculation.
  double p_error_last_;
  ///! Position error.
  double p_error_;
  ///! Integral of position error.
  double i_error_;
  ///! Derivative of position error.
  double d_error_;
  ///! The update interface, e = target - state
  bool update(double e, double dt) {
    if (first_compute_) {
      p_error_last_ = e;
      p_error_      = e;
      d_error_      = 0;
      i_error_      = dt * p_error_;
      first_compute_= false;
      return true;
    }
    // Calculate the derivative error
    tmp_d_error_  = (e - p_error_last_) / dt;
    if (std::isnan(tmp_d_error_) || std::isinf(tmp_d_error_))
      return false;
    p_error_      = e;
    d_error_      = tmp_d_error_;
    p_error_last_ = p_error_;
    // Calculate the integral of the position error
    i_error_     += dt * p_error_;
    return true;
  }

  void clear() {
    p_error_last_ = 0;
    p_error_      = 0;
    i_error_      = 0;
    d_error_      = 0;
    first_compute_=true;
  }

private:
  double tmp_d_error_;
  bool   first_compute_;
};

inline double operator*(const Gains& k, const Errors& e) {

  return k.p_gain_ * e.p_error_
       + boost::algorithm::clamp(k.i_gain_ * e.i_error_, k.i_min_, k.i_max_)
       + k.d_gain_ * e.d_error_;
}

inline double operator*(const Errors& e, const Gains& k) {
  return k * e;
}

struct Limits {
  enum Type {
    MIN = 0,
    MAX,
    N_TYPE
  };
  double command[Type::N_TYPE];
  double angle[Type::N_TYPE];
};

class Stability {
private:
  ///! The stability margin;
  double  epsilon_;
  ///! The times waiting.
  size_t lapse_;
  ///! The target value
  double target_;
  bool  internal_flag_;
  ///! The stability array
  std::vector<double> x_buf_;
  std::vector<double> e_buf_;

  const size_t BUF_REV_SIZE = 64;

public:
  Stability(double epsilon, size_t lapse)
    : epsilon_(epsilon), lapse_(lapse),
      target_(INVALID_VALUE),
      internal_flag_(false) {
    x_buf_.reserve(BUF_REV_SIZE);
    e_buf_.reserve(BUF_REV_SIZE);
  }

  /*!
   * @brief Set the parameter
   */
  void epsilon(double _e, double _lapse) {
    epsilon_ = _e;
    lapse_   = _lapse;
  }

  /*!
   * @brief Get the target
   */
  double target() { return target_; }

  /*!
   * @brief This method set a new target.
   */
  void set_target(double target) {
    internal_flag_ = false;
    x_buf_.clear();
    e_buf_.clear();
    target_ = target;
  }

  /*!
   * @brief This method clear the current target.
   */
  void clear_target() {
    internal_flag_ = false;
    x_buf_.clear();
    e_buf_.clear();
    target_ = INVALID_VALUE;
  }

  /*!
   * @brief This method judges the current state whether is stable or not,
   *        return true if the system is stability, or return false.
   *        @_e is error.
   */
  bool classifier(double _x, double& _e) {
    if (INVALID_VALUE == target_) {
      _e = INVALID_VALUE;
      return true;
    } else {
      _e = target_ - _x;
    }

    if (BUF_REV_SIZE == x_buf_.size()) {
      x_buf_.clear();
      e_buf_.clear();
    }

    internal_flag_ = (std::abs(_e) <= epsilon_);
    if (internal_flag_) {
      // mean filter.
      _x += (x_buf_.empty()) ? 0 : x_buf_.back();
      x_buf_.push_back(_x);
      // printf("modified: %05d %04d %04d\n", _x, x_buf_.size(), _x / x_buf_.size());
      _x /= x_buf_.size();
      // re-calculate the error.
      _e = target_ - _x;
      e_buf_.push_back(std::abs(_e) + ((e_buf_.empty()) ? 0 : e_buf_.back()));
    } else {
      x_buf_.clear();
      e_buf_.clear();
    }

    return ((e_buf_.size() >= lapse_)
        && (std::abs(e_buf_.back()/e_buf_.size()) <= epsilon_));
  }

};

Pid::Pid(const std::string& _name)
  : name_(_name), gains_(new Gains), errors_(new Errors),
    limits_(new Limits), time_control_(new TimeControl),
    stability_(new Stability(0.1, 10)) {
  ;
}

Pid::~Pid() {
  delete gains_;
  gains_ = nullptr;

  delete errors_;
  errors_ = nullptr;

  delete limits_;
  limits_ = nullptr;

  delete time_control_;
  time_control_ = nullptr;

  delete stability_;
  stability_ = nullptr;
}

void Pid::gains(double Kp, double Ki, double Kd, double i_min, double i_max) {
  gains_->p_gain_ = Kp;
  gains_->i_gain_ = Ki;
  gains_->d_gain_ = Kd;
  gains_->i_min_  = i_min;
  gains_->i_max_  = i_max;
}

void Pid::target(double target) {
  if (target == stability_->target()) return;
  LOG_DEBUG << name_ << " - Setting Target: " << target;

  stability_->set_target(target);
  errors_->clear();
  time_control_->start();
}

void Pid::epsilon(double epsilon, size_t lapse) {
  stability_->epsilon(epsilon, lapse);
}

void Pid::limits(double x_min, double x_max, double u_min, double u_max) {
  ;
}

bool Pid::control(double _x, double& _u) {
  // if (__d_debug_) printf("update - %d\n", _x);
  if (!time_control_->running() || std::isnan(_x) || std::isinf(_x))
      return false;

  double error = 0;
  // The system has not went to stabilization.
  // if (stability(_x, error) || (INVALID_VALUE == error)) return false;
  stability_->classifier(_x, error);
  if (INVALID_VALUE == error) return false;

  // compute the command
  compute(_x, error, _u);

  // safety control
  // safety_control(_x, _u);
  return true;
}

bool Pid::stability(double _x, double& _e) {
  if (!stability_->classifier(_x, _e)) return false;

  int64_t time = 0;
  time_control_->stop(&time);
  LOG_DEBUG << name_ << " - This target is reached, using "
            << time << ", error " << _e;

  errors_->clear();
  stability_->clear_target();
  return true;
}

void Pid::compute(double _x, double _e, double& _u) {
  _u =  (errors_->update(_e, time_control_->dt_s())) ?
        ((*gains_) * (*errors_)) : 0.0;
}

void Pid::safety_control(double _x, double& _u) {
  _u = boost::algorithm::clamp(_u, limits_->command[Limits::MIN], limits_->command[Limits::MAX]);
  if (_x <= limits_->angle[Limits::MIN] || _x >= limits_->angle[Limits::MAX]) {
    LOG_DEBUG << name_ << " - Outside the range, " << _x << " should in \\in ["
        << limits_->angle[Limits::MIN] << ", " << limits_->angle[Limits::MAX]
        << "], change the command to zero.";
    _u = 0;
  }
}

