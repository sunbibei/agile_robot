/*
 * trot.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#include "gait/sl/hopping.h"
#include <toolbox/time_control.h>
#include <foundation/cfg_reader.h>
#include <robot/agile_robot.h>

namespace agile_control {

///! The private parameters for trot.
struct __SLHPParams {
  ///! The stance height, x-direction and y-direction translation,
  ///! namely the balance position of spring.
  Eigen::Vector3d STC_POS;
  ///! The spring constant
  double SPR_CST;
  ///! The damping coefficient
  double DMP_COF;

  __SLHPParams(const std::string& _tag)
    : STC_POS(0, 0, 46), SPR_CST(1.0), DMP_COF(0.5) {
    auto cfg = MiiCfgReader::instance();
    // TODO
    // cfg->get_value(_tag, "stc_len",             STC_POS);
    cfg->get_value(_tag, "spring_constant",     SPR_CST);
    cfg->get_value(_tag, "damping_coefficient", DMP_COF);
  }
};

SLHopping::SLHopping()
  : current_state_(SLHPState::INVALID_SL_HP_STATE),
    params_(nullptr), leg_iface_(nullptr),
    post_tick_interval_(20), stc_test_timer_(nullptr),
    leg_cmd_(nullptr) {
  ;
}

bool SLHopping::auto_init() {
  if (!GaitBase::auto_init()) return false;

  auto ifaces = LegRobot::instance();
  if (!ifaces) {
    LOG_FATAL << "The interface for robot is null!";
    return false;
  }

  ///! TODO
  auto cfg = MiiCfgReader::instance();
  LegType leg = LegType::FL;
  if (!cfg->get_value(getLabel(), "leg_type", leg))
    LOG_WARNING << "No point leg type, using the default type of leg -- FL";

  leg_iface_ = ifaces->robot_leg(leg);
  return true;
}

SLHopping::~SLHopping() {
  stopping();
}

bool SLHopping::starting() {
  state_machine_ = new StateMachine<SLHPState>(current_state_);
  auto _sm       = (StateMachine<SLHPState>*)state_machine_;
  _sm->registerStateCallback(SLHPState::SL_HP_TRU_STATE, &SLHopping::sl_hp_thrust,   this);
  _sm->registerStateCallback(SLHPState::SL_HP_FGT_STATE, &SLHopping::sl_hp_flight,   this);
  _sm->registerStateCallback(SLHPState::SL_HP_STP_STATE, &SLHopping::sl_hp_stopping, this);
  _sm->registerStateCallback(SLHPState::SL_STC_TEST_STATE, &SLHopping::sl_static_test, this);
  // TODO
  params_ = new __SLHPParams(Label::make_label(getLabel(), "parameters"));

  leg_cmd_    = new LegTarget;
  current_state_ = SLHPState::SL_STC_TEST_STATE;
  return true;
}

void SLHopping::stopping() {
  current_state_ = SLHPState::INVALID_SL_HP_STATE;

  delete state_machine_;
  state_machine_ = nullptr;

  delete params_;
  params_   = nullptr;
}

void SLHopping::checkState() {
  switch(current_state_) {
  case SLHPState::SL_HP_TRU_STATE:
    break;
  case SLHPState::SL_HP_FGT_STATE:
    break;
  case SLHPState::SL_HP_STP_STATE:
    break;
  case SLHPState::SL_STC_TEST_STATE:
  {
    if (nullptr == stc_test_timer_) {
      stc_test_timer_    = new TimeControl(true);
      leg_cmd_->cmd_type = JntCmdType::CMD_TOR;
      leg_cmd_->target.resize(LegType::N_LEGS);
      leg_cmd_->target.fill(0.0);
      last_eef_delta_.fill(0.0);
    }

    break;
  }
  default:
    LOG_ERROR << "What fucking State!";
  }
}

///! call it after the state callback
void SLHopping::post_tick() {
  ///! The command frequency control
  static TimeControl _s_post_tick(true);
  static int64_t     _s_sum_interval = 0;
  _s_sum_interval += _s_post_tick.dt();
  if (_s_sum_interval < post_tick_interval_) return;
  _s_sum_interval = 0;

  ///! execute the command.
  leg_iface_->legTarget(*leg_cmd_);
  leg_iface_->move();
}

///! callback for SL_TRU_STATE
void SLHopping::sl_hp_thrust() {
  ;
}

///! callback for SL_FGT_STATE
void SLHopping::sl_hp_flight() {
  ;
}

///! callback for SL_STP_STATE
void SLHopping::sl_hp_stopping() {
  ;
}

///! callback for SL_STC_TEST_STATE
void SLHopping::sl_static_test() {
  if ((nullptr == stc_test_timer_) || (!stc_test_timer_->running()))
    return;

  leg_iface_->fk(last_leg_eef_);
  Eigen::Vector3d _diff  = params_->STC_POS - last_leg_eef_;
  Eigen::Vector3d _delta = _diff - last_eef_delta_;
  double _dt = stc_test_timer_->dt();
  _dt /= 1000.0; // Unit of Translation, from mic-second to second.
  if (std::abs(_dt) < 1e-5) _delta.fill(0.0);
  else _delta /= _dt;

  ///! get the virtual force which the leg output.
  Eigen::Vector3d _f = params_->SPR_CST * _diff + params_->DMP_COF * _delta;
  ///! convert the force origin from the end-effector to the torque of each joint.
  leg_iface_->is(_f, leg_cmd_->target);

  ///! advance the variables.
  last_eef_delta_ = _diff;
}

} /* namespace qr_control */
