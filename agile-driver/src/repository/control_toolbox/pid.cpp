/*
 * pid.cpp
 *
 *  Created on: Nov 2, 2017
 *      Author: bibei
 */

#include <repository/control_toolbox/pid.h>
#include <foundation/cfg_reader.h>

#include <boost/algorithm/clamp.hpp>
#include <fstream>
#include <cmath>

namespace agile_robot {

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
  short command[Type::N_TYPE];
  short angle[Type::N_TYPE];
};

struct TimeControl {
private:
  const std::chrono::high_resolution_clock::time_point INVALID_TIME_POINT;

public:
  ///! the first compute
  bool  first_compute_;
  ///! time control (in ms)
  int64_t dt_;
  std::chrono::high_resolution_clock::time_point curr_update_t_;
  std::chrono::high_resolution_clock::time_point last_update_t_;

  ///! these variables for debug.
  std::chrono::high_resolution_clock::time_point t0_;
  std::chrono::high_resolution_clock::time_point t1_;

  TimeControl()
    : INVALID_TIME_POINT(std::chrono::high_resolution_clock::time_point::max()),
      first_compute_(true), dt_(0),
      curr_update_t_(INVALID_TIME_POINT),
      last_update_t_(INVALID_TIME_POINT),
      t0_(INVALID_TIME_POINT), t1_(INVALID_TIME_POINT) {
    ;
  }

  bool running() {
    return (last_update_t_ != INVALID_TIME_POINT);
  }

  void start() {
    curr_update_t_ = std::chrono::high_resolution_clock::now();
    last_update_t_ = curr_update_t_;
    t0_            = curr_update_t_;
  }

  /*!
   * @brief The duration (in ms)
   */
  int64_t dt() {
    curr_update_t_ = std::chrono::high_resolution_clock::now();
    dt_            = std::chrono::duration_cast<std::chrono::milliseconds>
        (curr_update_t_ - last_update_t_).count();
    last_update_t_ = curr_update_t_;
    return dt_;
  }

  void stop(int64_t* span = nullptr) {
    t1_ = std::chrono::high_resolution_clock::now();
    curr_update_t_ = INVALID_TIME_POINT;
    last_update_t_ = INVALID_TIME_POINT;
    if (span) *span= std::chrono::duration_cast<std::chrono::milliseconds>
                        (t1_ - t0_).count();
  }
};

class Stability {
private:
  ///! The stability margin;
  short  epsilon_;
  ///! The times waiting.
  size_t lapse_;
  ///! The target value
  short target_;
  bool  internal_flag_;
  ///! The stability array
  std::vector<short> x_buf_;
  std::vector<short> e_buf_;

  const size_t BUF_REV_SIZE = 64;

public:
  Stability(short epsilon, size_t lapse)
    : epsilon_(epsilon), lapse_(lapse),
      target_(INVALID_VALUE),
      internal_flag_(false) {
    x_buf_.reserve(BUF_REV_SIZE);
    e_buf_.reserve(BUF_REV_SIZE);
  }

  /*!
   * @brief Get the target
   */
  short target() { return target_; }

  /*!
   * @brief This method set a new target.
   */
  void set_target(short target) {
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
  bool classifier(short _x, short& _e) {
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

Pid::Pid(const std::string& prefix)
  : gains_(new Gains), errors_(new Errors),
    limits_(new Limits), time_control_(new TimeControl),
    stability_(nullptr) {
  auto cfg = MiiCfgReader::instance();

  std::vector<double> tmp_vec_d;
  cfg->get_value_fatal(prefix, "gains", tmp_vec_d);
  if (5 == tmp_vec_d.size()) {
    gains_->p_gain_ = tmp_vec_d[0];
    gains_->i_gain_ = tmp_vec_d[1];
    gains_->d_gain_ = tmp_vec_d[2];
    gains_->i_min_  = tmp_vec_d[3];
    gains_->i_max_  = tmp_vec_d[4];
  }

  std::vector<short> tmp_vec_s;
  cfg->get_value(prefix, "limits", tmp_vec_s);
  if (4 == tmp_vec_s.size()) {
    limits_->command[Limits::MIN] = tmp_vec_s[0];
    limits_->command[Limits::MAX] = tmp_vec_s[1];
    limits_->angle[Limits::MIN]   = tmp_vec_s[2];
    limits_->angle[Limits::MAX]   = tmp_vec_s[3];
  }

  tmp_vec_s.clear();
  cfg->get_value(prefix, "criterion", tmp_vec_s);
  if (2 == tmp_vec_s.size())
    stability_ = new Stability(tmp_vec_s[0], tmp_vec_s[1]);

  std::string tmp;
  Label::split_label(prefix, tmp, name_);

  // For debug function
  __d_debug_ = false;
  if (cfg->get_value(prefix, "debug", __d_debug_) && __d_debug_) {
    std::string path = ".";
    cfg->get_value(prefix, "path", path);
    __d_ofd_.open(path + "/" + name_
        /*+ "_" + std::to_string((short)gains_->p_gain_)
        + "_" + std::to_string((short)gains_->i_gain_)
        + "_" + std::to_string((short)gains_->d_gain_)*/);

    __d_x_buf_.reserve(BUF_RESERVE_SIZE);
    __d_u_buf_.reserve(BUF_RESERVE_SIZE);
    __d_t_buf_.reserve(BUF_RESERVE_SIZE);
    __d_e_buf_.reserve(BUF_RESERVE_SIZE);
    __d_mu_buf_.reserve(BUF_RESERVE_SIZE);
    __d_p_term_.reserve(BUF_RESERVE_SIZE);
    __d_i_term_.reserve(BUF_RESERVE_SIZE);
    __d_d_term_.reserve(BUF_RESERVE_SIZE);
  }
}

Pid::~Pid() {
  __save_everything_to_file();
  if (nullptr != gains_) {
    delete gains_;
    gains_ = nullptr;
  }
  if (nullptr != errors_) {
    delete errors_;
    errors_ = nullptr;
  }
}

void Pid::setTarget(short target) {
  if (target == stability_->target()) return;
  if (__d_debug_) LOG_DEBUG << name_ << " - Setting Target: " << target;

  stability_->set_target(target);
  errors_->clear();
  time_control_->start();
}

bool Pid::control(short _x, short& _u) {
  // if (!__d_debug_) return false;

  // if (__d_debug_) printf("update - %d\n", _x);
  if (!time_control_->running() || std::isnan(_x) || std::isinf(_x))
      return false;

  short error = 0;
  // The system has not went to stabilization.
  if (stability(_x, error) || (INVALID_VALUE == error)) return false;

  // compute the command
  compute(_x, error, _u);

  // safety control
  safety_control(_x, _u);

  if (/*_u && */true && __d_debug_)
    printf("%s - %02ld %+04d %+4.2f %+4.2f %+1.2f %04d %04d %04d\n",
        name_.c_str(), time_control_->dt_, error,
        errors_->p_error_, errors_->i_error_, errors_->d_error_,
        stability_->target(), _x, _u);

  __save_data_into_buf(time_control_->dt_, _x, _u, error,
      errors_->p_error_, errors_->i_error_, errors_->d_error_);
  return true;
}

bool Pid::stability(short _x, short& _e) {
  if (!stability_->classifier(_x, _e)) return false;

  int64_t time = 0;
  time_control_->stop(&time);
  if (__d_debug_) LOG_DEBUG << name_ << " - This target is reached, using "
                   << time << ", error " << _e;

  errors_->clear();
  stability_->clear_target();
  __save_everything_to_file();
  return true;
}

void Pid::compute(short _x, short _e, short& _u) {
  _u =  (errors_->update(_e, time_control_->dt()/1000.0)) ?
        ((*gains_) * (*errors_)) : 0.0;
}

void Pid::safety_control(short _x, short& _u) {
  _u = boost::algorithm::clamp(_u, limits_->command[Limits::MIN], limits_->command[Limits::MAX]);
  if (_x <= limits_->angle[Limits::MIN] || _x >= limits_->angle[Limits::MAX]) {
    LOG_DEBUG << name_ << " - Outside the range, " << _x << " should in \\in ["
        << limits_->angle[Limits::MIN] << ", " << limits_->angle[Limits::MAX]
        << "], change the command to zero.";
    _u = 0;
  }
}

inline void Pid::__save_everything_to_file() {
  if ((!__d_debug_) || !__d_ofd_.is_open()) return;
  for (size_t i = 0; i < __d_t_buf_.size(); ++i)
    __d_ofd_ << __d_t_buf_[i]  << " " << __d_x_buf_[i] << " "
             << __d_u_buf_[i]  << " " << __d_e_buf_[i] << " "
             << __d_p_term_[i] << " " << __d_i_term_[i] << " "
             << __d_d_term_[i] << " " << __d_mu_buf_[i] << std::endl;

  LOG_DEBUG << "It has saved everything into the local file.";
  __d_u_buf_.clear();
  __d_x_buf_.clear();
  __d_t_buf_.clear();
  __d_e_buf_.clear();
  __d_mu_buf_.clear();
  __d_p_term_.clear();
  __d_i_term_.clear();
  __d_d_term_.clear();
}

inline void Pid::__save_data_into_buf(double dt, short _x, short _u, short _err,
    double _p_term, double _i_term, double _d_term) {
  if ((!__d_debug_) || !__d_ofd_.is_open()) return;
  __d_t_buf_ .push_back(dt);
  __d_x_buf_ .push_back(_x);
  __d_u_buf_ .push_back(_u);
  __d_e_buf_ .push_back(_err);
  __d_mu_buf_.push_back(stability_->target());
  __d_p_term_.push_back(_p_term);
  __d_i_term_.push_back(_i_term);
  __d_d_term_.push_back(_d_term);
}

} /* namespace middleware */
