/*
 * creep.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: bibei
 */

#include "gait/creep/creep.h"
#include "adt/segmented.h"
#include "adt/polynomial.h"
#include "adt/geometry.h"

#include <toolbox/time_control.h>
#include <foundation/cfg_reader.h>
#include <boost/algorithm/clamp.hpp>
#include <robot/agile_robot.h>

#ifdef TEST_TIME
#include <fstream>
#endif

namespace agile_control {

struct CreepParams {
  ///! The foot step in x-direction
  double FOOT_STEP;
  ///! The foot step in y-direction
  double FOOT_STEP_Y;
  ///! The height value when the robot stay stance.
  double STANCE_HEIGHT;
  ///! The height value when swing leg.
  double SWING_HEIGHT;

  ///! The time for moving COG(in ms)
  int64_t   COG_TIME;
  ///! The time for swing leg (in ms)
  int64_t   SWING_TIME;
  ///! The time for swing leg in rise phase (in s)
  double   SWING_RISE;
  ///! The time for swing leg in approach phase (in s)
  double   SWING_APPROACH;
  ///! The time for swing leg in drop phase (in s)
  double   SWING_DROP;
  ///! The minimum stability margin could to swing leg.
  double   MARGIN_THRES;
  ///! The maximum swing difference(cm)
  double   ALLOW_DIFF;

  CreepParams(const std::string& _tag)
    : FOOT_STEP(10),     FOOT_STEP_Y(0),
      STANCE_HEIGHT(46), SWING_HEIGHT(5),
      COG_TIME(2000),    SWING_TIME(2000),
      MARGIN_THRES(6),   ALLOW_DIFF(5) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value(_tag, "step",         FOOT_STEP);
    cfg->get_value(_tag, "step_y",       FOOT_STEP_Y);
    cfg->get_value(_tag, "stance_height",STANCE_HEIGHT);
    cfg->get_value(_tag, "swing_height", SWING_HEIGHT);

    cfg->get_value(_tag, "cog_time",     COG_TIME);
    cfg->get_value(_tag, "swing_time",   SWING_TIME);

    cfg->get_value(_tag, "swing_rise",     SWING_RISE);
    cfg->get_value(_tag, "swing_approach", SWING_APPROACH);
    cfg->get_value(_tag, "swing_drop",     SWING_DROP);

    cfg->get_value(_tag, "margin_threshold", MARGIN_THRES);
    cfg->get_value(_tag, "allow_diff",       ALLOW_DIFF);
  }
};

Creep::Creep(const std::string& _n)
  : GaitBase(_n), body_iface_(nullptr),
    current_state_(CreepState::UNKNOWN_CP_STATE),
    cp_params_(nullptr), swing_leg_(LegType::UNKNOWN_LEG),
    timer_(nullptr), swing_timer_(nullptr), cog_timer_(nullptr),
    post_tick_interval_(50), is_hang_(false) {

  for (auto& l : leg_ifaces_)
    l = nullptr;
}

bool Creep::auto_init() {
  if (!GaitBase::auto_init()) return false;

  auto ifaces = LegRobot::instance();
  if (!ifaces) {
    LOG_FATAL << "The interface for robot is null!";
    return false;
  }
  body_iface_ = ifaces->robot_body();
  if (!body_iface_)
    LOG_FATAL << "The interface of RobotBody is null!";

  FOREACH_LEG(l) {
    leg_ifaces_[l] = ifaces->robot_leg(l);
    if (!leg_ifaces_[l])
      LOG_FATAL << "The interface of RobotLeg" << LEGTYPE2STR(l) << " is null!";
  }

  if (false) {
    LOG_INFO << "get interface(LegType::FL): " << leg_ifaces_[LegType::FL];
    LOG_INFO << "get interface(LegType::FR): " << leg_ifaces_[LegType::FR];
    LOG_INFO << "get interface(LegType::HL): " << leg_ifaces_[LegType::HL];
    LOG_INFO << "get interface(LegType::HR): " << leg_ifaces_[LegType::HR];
  }

  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "hang",     is_hang_);
  cfg->get_value(getLabel(), "interval", post_tick_interval_);

  std::string _tag = Label::make_label(getLabel(), "coeffs");
  cp_params_ = new CreepParams(_tag);

  return true;
}

bool Creep::starting() {
  state_machine_ = new StateMachine<CreepState>(current_state_);
  auto _sm       = (StateMachine<CreepState>*)state_machine_;
  _sm->registerStateCallback(CreepState::CP_INIT_POSE,   &Creep::pose_init,   this);
  _sm->registerStateCallback(CreepState::CP_READY,       &Creep::ready,       this);
  _sm->registerStateCallback(CreepState::CP_SWING_FRONT, &Creep::swing_front, this);
  _sm->registerStateCallback(CreepState::CP_SWING_HIND,  &Creep::swing_hind,  this);

  current_state_ = CreepState::CP_INIT_POSE;

  ///! initialize the TimeControl
  timer_       = new TimeControl;
  swing_timer_ = new TimeControl;
  cog_timer_   = new TimeControl;

//  eef_traj_.reset(new SegTraj3d);
//  for (auto& t : cog2eef_traj_)
//    t.reset(new PolyTraj3d);

#ifdef TEST_TIME
  t_time_.reserve(24*4);
  t_translation_.reserve(24*4);
#endif

  LOG_INFO << "The Creep gait has STARTED!";
  return true;
}

void Creep::stopping() {
  delete cp_params_;
  cp_params_ = nullptr;

  delete state_machine_;
  state_machine_ = nullptr;

  current_state_ = CreepState::UNKNOWN_CP_STATE;

#ifdef TEST_TIME
  std::ofstream _ofd("/home/bibei/exper_time");
  LOG_WARNING << "STARTING TO SAVE TIME DATA... ...";
  for (size_t i = 0; i < t_time_.size(); ++i) {
    _ofd << t_time_[i] << " " << t_translation_[i] << std::endl;
  }
  _ofd.close();
#endif
}

Creep::~Creep() {
  stopping();
  if (state_machine_) {
    delete state_machine_;
    state_machine_ = nullptr;
  }
}

void Creep::checkState() {
  ///! the local static variable for every state time span.
  static int64_t _s_tmp_span = 0;

  switch(current_state_) {
  case CreepState::CP_INIT_POSE:
  {
    FOREACH_LEG(l) {
      auto diff = (leg_ifaces_[l]->eef() - eef_cmds_[l]).norm();
      if (diff > 0.3) return;
    }
    return;

    ///! the end of WK_INIT_POS
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----INIT POSE OK!(" << _s_tmp_span << "ms)----*******";
    print_eef_pos(eef_cmds_[LegType::FL], eef_cmds_[LegType::FR],
        eef_cmds_[LegType::HL], eef_cmds_[LegType::HR]);

    PRESS_THEN_GO
    if (is_hang_) {
      ///! updating the following swing leg
      swing_leg_     = next_leg(swing_leg_);
      current_state_ = (LEGTYPE_IS_FRONT(swing_leg_) ?
              CreepState::CP_SWING_FRONT : CreepState::CP_SWING_HIND);
    } else {
      current_state_ = CreepState::CP_READY;
    }

    break;
  }
  case CreepState::CP_READY:
  {
    if (!end_ready()) return;
    ///! the end of WK_INIT_POS
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----READY " << " (" << _s_tmp_span << "ms)----*******";

    FOREACH_LEG(l) {
      balance_fpts_(l) = leg_ifaces_[l]->eef().z();
    }

    LOG_WARNING << "ready: " << balance_fpts_.transpose();
    PRESS_THEN_GO
    ///! updating the following swing leg
    swing_leg_     = next_leg(swing_leg_);
    current_state_ = (LEGTYPE_IS_FRONT(swing_leg_) ?
            CreepState::CP_SWING_FRONT : CreepState::CP_SWING_HIND);
    break;
  }
  case CreepState::CP_SWING_HIND:
  case CreepState::CP_SWING_FRONT:
  {
    if (!end_walk()) return;

    ///! the end of WK_INIT_POS
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----SWING " << LEGTYPE2STR(swing_leg_)
        << " (" << _s_tmp_span << "ms)----*******";

#ifdef TEST_TIME
  t_time_.push_back(_s_tmp_span);
  double trans = std::abs(leg_ifaces_[LEGTYPE_DL(swing_leg_)]->eef().x() - t_last_stance_pos_);
  t_translation_.push_back(trans);
  LOG_WARNING << "TRANS - " << LEGTYPE2STR(swing_leg_) << ": " << trans;
  LOG_WARNING << "VELOS - " << trans * 1000.0 / _s_tmp_span << " cm/s";
#else
    PRESS_THEN_GO
#endif
    auto diff = leg_ifaces_[swing_leg_]->eef().x() - eef_traj_->sample(eef_traj_->ceiling()).x();
    if (std::abs(diff) < cp_params_->ALLOW_DIFF) {
      ///! updating the following swing leg
      swing_leg_     = next_leg(swing_leg_);
      current_state_ = (LEGTYPE_IS_FRONT(swing_leg_) ?
              CreepState::CP_SWING_FRONT : CreepState::CP_SWING_HIND);
    } else {
      LOG_WARNING << "SWING AGAIN...(" << leg_ifaces_[swing_leg_]->eef().x() << " v.s. "
          << eef_traj_->sample(eef_traj_->ceiling()).x() << ")";
      PRESS_THEN_GO
    }
    break;
  }
  default:
    LOG_ERROR << "What fucking State!";
  }
}

inline int __sign(double t) {
  return (t < 0) ? -1 : 1;
}


void Creep::post_tick() {
  ///! The command frequency control
  static TimeControl _s_post_tick(true);
  static int64_t     _s_sum_interval = 0;
  _s_sum_interval += _s_post_tick.dt();
  if (_s_sum_interval < post_tick_interval_) return;
  _s_sum_interval = 0;

  FOREACH_LEG(leg) {
    ///! Modify the target to keep the stance stability
    if (!is_hang_ && CreepState::CP_INIT_POSE != current_state_
        && CreepState::CP_READY != current_state_) {
      Eigen::Vector3d _eef = leg_ifaces_[leg]->eef();

      if (LegState::TD_STATE != leg_ifaces_[leg]->leg_state()) {
        ///! If has not touch the earth, drop deeper.
        if ((swing_leg_ != leg) || (!swing_timer_->running())) {
          ///! When the moving CoG and not to swing leg yet.
          eef_cmds_[leg].z() = boost::algorithm::clamp(
              _eef.z() - 1,
              balance_fpts_(leg) - 10, balance_fpts_(leg));
        } else {
          ;// Nothing to do here.
        }
      } else {
        ///! If has touch the earth, adjust the height close to the balance height.
        if ((swing_leg_ != leg) || (!swing_timer_->running())) {
          double diff = balance_fpts_(leg) - _eef.z();
          if (std::abs(diff) > 2) {
            eef_cmds_[leg].z() = _eef.z() + __sign(diff) * 0.5;
          }
        }
      }
    }

    leg_ifaces_[leg]->eefPositionTarget(eef_cmds_[leg]);
    leg_ifaces_[leg]->move();
  }

  // print_jnt_pos(/*JNTS_TARGET*/);
  // print_eef_pos();

//  Eigen::Vector3d margins = stability_margin(swing_leg_);
//  LOG_WARNING << "cog -> cf-ch:" << margins.x();
//  LOG_WARNING << "cog -> il-sl:" << margins.y();
//  LOG_WARNING << "cog -> il-dl:" << margins.z();
}

void Creep::pose_init() {
  if (!timer_->running()) {
    FOREACH_LEG(l) {
      eef_cmds_[l] << 0.0, 0.0, -cp_params_->STANCE_HEIGHT;
    }

    timer_->start();
  }

//  print_eef_pos(eef_cmds_[LegType::FL], eef_cmds_[LegType::FR],
//        eef_cmds_[LegType::HL], eef_cmds_[LegType::HR]);
}

void Creep::ready() {
  if (!timer_->running()) {
    ///! start the timer
    timer_->start();
  }

  FOREACH_LEG(l) {
    if (LegState::TD_STATE != leg_ifaces_[l]->leg_state())
      eef_cmds_[l].z() = boost::algorithm::clamp(eef_cmds_[l].z() - 0.3,
          -cp_params_->STANCE_HEIGHT - 10, -cp_params_->STANCE_HEIGHT + 5);
  }
}

bool Creep::end_ready() {
  FOREACH_LEG(l) {
    if (LegState::TD_STATE != leg_ifaces_[l]->leg_state())
      return false;
  }
  return true;
}

void Creep::swing_hind() {
  if (!timer_->running()) {
    ///! programming the next fpt.
    Eigen::Vector3d _next_eef = leg_ifaces_[swing_leg_]->eef();
#ifdef TEST_TIME
    t_last_swing_pos_   = _next_eef.x();
    t_last_stance_pos_  = leg_ifaces_[LEGTYPE_DL(swing_leg_)]->eef().x();
#endif
    _next_eef.x() = cp_params_->FOOT_STEP;
    _next_eef.y() = (LEGTYPE_IS_LEFT(swing_leg_)) ?
                       cp_params_->FOOT_STEP_Y : -cp_params_->FOOT_STEP_Y;
#ifdef CONTINUE_GAIT
    prog_eef_traj(_next_eef);
    swing_timer_->start();
#endif
    ///! propgraming the CoG trajectory.
    _next_eef    = _next_eef + body_iface_->leg_base(swing_leg_);
    Eigen::Vector3d _cf_eef = leg_ifaces_[LEGTYPE_CF(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_CF(swing_leg_));
    Eigen::Vector3d _ch_eef = leg_ifaces_[LEGTYPE_CH(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_CH(swing_leg_));
    Eigen::Vector3d _il_eef = leg_ifaces_[LEGTYPE_IL(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_IL(swing_leg_));

#ifdef CONTINUE_GAIT
    Eigen::Vector2d _last_cog = body_iface_->cog().head(2);
    Eigen::Vector2d _next_cog(_last_cog.x() + 0.5*cp_params_->FOOT_STEP, _last_cog.y());
    Eigen::Vector2d _cs = geometry::cross_point(
        geometry::linear(_next_cog, _last_cog),
        geometry::linear(_cf_eef.head(2), _next_eef.head(2)));
    _next_cog = (_cs + _last_cog) * 0.5;
#else
    Eigen::Vector2d _cs = geometry::cross_point(
        geometry::linear(_cf_eef.head(2), _next_eef.head(2)),
        geometry::linear(_il_eef.head(2), _ch_eef.head(2)));
    Eigen::Vector2d _next_cog = geometry::incenter_of_triangle(_cs, _cf_eef.head(2), _ch_eef.head(2));
#endif
    LOG_WARNING << "NEXT COG: " << _next_cog.transpose();

    prog_cog_traj(_next_cog);
    ///! start the timer
    timer_->start();
    cog_timer_->start();
  }

#ifndef CONTINUE_GAIT
  if (!swing_timer_->running()) {
    Eigen::Vector3d margins = stability_margin(swing_leg_);
    if ((margins.minCoeff() >= cp_params_->MARGIN_THRES)
        || (!cog_timer_->running())
        || (cog_timer_->span() >= 0.8*cp_params_->COG_TIME)) {
      ///! programming the swing trajectory.
      LOG_WARNING << "STARTING...";
      Eigen::Vector3d _next_eef = leg_ifaces_[swing_leg_]->eef();
      _next_eef.x() = cp_params_->FOOT_STEP;
      _next_eef.y() = (LEGTYPE_IS_LEFT(swing_leg_)) ?
                         cp_params_->FOOT_STEP_Y : -cp_params_->FOOT_STEP_Y;
      prog_eef_traj(_next_eef);

      swing_timer_->start();
    }
  }
#endif

  ///! If the swing_timer is running, control to swing leg.
  if (swing_timer_->running()) {
    ///! The default next eef coordination.
    eef_cmds_[swing_leg_] = eef_traj_->sample(swing_timer_->span()/1000.0);

    ///! If close to the floor, we need to judge whether has touch the earth.
    if ((!is_hang_) && swing_timer_->span() >= 0.8*cp_params_->SWING_TIME) {
      Eigen::Vector3d _eef = leg_ifaces_[swing_leg_]->eef();
      ///! If touch the earth, don't move and will stop swing leg in the @end_walk.
      if (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()) {
        eef_cmds_[swing_leg_] = _eef;
      } else {
        ///! If don't touch the earth, we will to touch more deep.
        if (swing_timer_->span() > cp_params_->SWING_TIME) {
          eef_cmds_[swing_leg_].z() = boost::algorithm::clamp(
              leg_ifaces_[swing_leg_]->eef().z() - 1,
              balance_fpts_(swing_leg_) - 10, balance_fpts_(swing_leg_));
        }
      }
    }
  }

  ///! If the cog_timer is running, control to move the CoG.
  if (cog_timer_->running()) {
    FOREACH_LEG(l) {
#ifdef CONTINUE_GAIT
      if (swing_leg_ != l) {
#else
      if (!swing_timer_->running() || swing_leg_ != l) {
#endif
        eef_cmds_[l] = cog2eef_traj_[l]->sample((double)cog_timer_->span()/cp_params_->COG_TIME);
      }

    }
  }

}

void Creep::swing_front() {
  if (!timer_->running()) {
    ///! programming the next fpt.
    Eigen::Vector3d _next_eef = leg_ifaces_[swing_leg_]->eef();
#ifdef TEST_TIME
    t_last_swing_pos_   = _next_eef.x();
    t_last_stance_pos_  = leg_ifaces_[LEGTYPE_DL(swing_leg_)]->eef().x();
#endif
    _next_eef.x() = cp_params_->FOOT_STEP;
    _next_eef.y() = (LEGTYPE_IS_LEFT(swing_leg_)) ?
                       cp_params_->FOOT_STEP_Y : -cp_params_->FOOT_STEP_Y;
    prog_eef_traj(_next_eef);

    ///! propgraming the CoG trajectory.
#ifdef CONTINUE_GAIT
    Eigen::Vector2d _last_cog = body_iface_->cog().head(2);
    Eigen::Vector2d _next_cog(_last_cog.x() + 0.5*cp_params_->FOOT_STEP, 0.0);
    Eigen::Vector3d _cf_eef = leg_ifaces_[LEGTYPE_CF(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_CF(swing_leg_));
    Eigen::Vector3d _il_eef = leg_ifaces_[LEGTYPE_IL(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_IL(swing_leg_));
    _next_cog.y() = (_il_eef.y() + _cf_eef.y()) * 0.5;

    Eigen::Vector2d _cs = geometry::cross_point(
        geometry::linear(_cf_eef.head(2), _il_eef.head(2)),
        geometry::linear(_next_cog, _last_cog));

    _next_cog     = _cs;
    // _next_cog.y() = _cf_eef.y() * 0.3;
    LOG_WARNING << "NEXT COG: " << _next_cog.transpose();

    prog_cog_traj(_next_cog);
    cog_timer_->start();
#else
    Eigen::Vector3d margins = stability_margin(swing_leg_);
    if (margins.minCoeff() < cp_params_->MARGIN_THRES) {
      Eigen::Vector3d _cf_eef = leg_ifaces_[LEGTYPE_CF(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_CF(swing_leg_));
      Eigen::Vector3d _il_eef = leg_ifaces_[LEGTYPE_IL(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_IL(swing_leg_));
      Eigen::Vector3d _ch_eef = leg_ifaces_[LEGTYPE_CH(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_CH(swing_leg_));
      Eigen::Vector2d _next_cog = geometry::incenter_of_triangle(
          _cf_eef.head(2), _il_eef.head(2), _ch_eef.head(2));

      LOG_WARNING << "NEXT COG: " << _next_cog.transpose();
      prog_cog_traj(_next_cog);
      cog_timer_->start();
    } else {
      ; // Don't move the CoG.
    }
#endif
    ///! start the timer
    timer_->start();
    swing_timer_->start();
  }

#ifndef CONTINUE_GAIT
  if (cog_timer_->running()) {
    Eigen::Vector3d margins = stability_margin(swing_leg_);
//    LOG_WARNING << "cog -> cf-ch:" << margins.x();
//    LOG_WARNING << "cog -> il-sl:" << margins.y();
//    LOG_WARNING << "cog -> il-dl:" << margins.z();
//    LOG_WARNING << "threshold:   " << cp_params_->MARGIN_THRES;
    if ((margins.minCoeff() < cp_params_->MARGIN_THRES)
         || !swing_timer_->running()) {
      LOG_WARNING << "STOP...";
      cog_timer_->stop();
    }
  }
#endif

  ///! If the swing_timer is running, control to swing leg.
  if (swing_timer_->running()) {
    ///! The default next eef coordination.
    eef_cmds_[swing_leg_] = eef_traj_->sample(swing_timer_->span()/1000.0);

    ///! If close to the floor, we need to judge whether has touch the earth.
    if (swing_timer_->span() >= 0.8*cp_params_->SWING_TIME) {
      Eigen::Vector3d _eef = leg_ifaces_[swing_leg_]->eef();
      ///! If touch the earth, don't move and will stop swing leg in the @end_walk.
      if (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()) {
        eef_cmds_[swing_leg_] = _eef;
      } else {
        ///! If don't touch the earth, we will to touch more deep.
        if (swing_timer_->span() > cp_params_->SWING_TIME) {
          eef_cmds_[swing_leg_].z() = boost::algorithm::clamp(
              leg_ifaces_[swing_leg_]->eef().z() - 1,
              balance_fpts_(swing_leg_) - 10, balance_fpts_(swing_leg_));
        }
      }
    }
  }

  ///! If the cog_timer is running, control to move the CoG.
  if (cog_timer_->running()) {
    FOREACH_LEG(l) {
      if (swing_leg_ != l) {
        eef_cmds_[l] = cog2eef_traj_[l]->sample((double)cog_timer_->span()/cp_params_->COG_TIME);
      }
    }
  }

}


bool Creep::end_walk() {
  if (is_hang_) {
    if ((swing_timer_->running()) && (swing_timer_->span() >= cp_params_->SWING_TIME))
      swing_timer_->stop();

    if ((cog_timer_->running())   && (cog_timer_->span()   >= cp_params_->COG_TIME))
      cog_timer_->stop();
  } else {
    ///! the real walk.
    if ((swing_timer_->running()) && (swing_timer_->span() >= 0.6*cp_params_->SWING_TIME)
          && (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state())) {
      swing_timer_->stop();
    }

    if ((cog_timer_->running()) && cog_timer_->span() >= /*2**/cp_params_->COG_TIME)
      cog_timer_->stop();
  }

  return (!swing_timer_->running() && !cog_timer_->running());
}


///! The flow of swing leg
///! This method only be called after the moving COG
LegType Creep::next_leg(const LegType curr) {
  switch (curr) {
  case LegType::FL: return LegType::HR;
  case LegType::FR: return LegType::HL;
  case LegType::HL: return LegType::FL;
  case LegType::HR: return LegType::FR;
  default: return LegType::HL; ///! The first swing HL leg.
  }
}

Eigen::Vector3d Creep::stability_margin(LegType sl) {
  static Eigen::Vector3d _eefs[LegType::N_LEGS];
  FOREACH_LEG(leg) {
    _eefs[leg] = leg_ifaces_[leg]->eef() + body_iface_->leg_base(leg);
  }

  if (!geometry::is_in_triangle(
      _eefs[LEGTYPE_CF(sl)].head(2), _eefs[LEGTYPE_CH(sl)].head(2),
      _eefs[LEGTYPE_IL(sl)].head(2), body_iface_->cog().head(2))) {
    // LOG_WARNING << "NO MOVE!";
    return Eigen::Vector3d(
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity(),
        -std::numeric_limits<double>::infinity());
  } else {
    return Eigen::Vector3d(
        geometry::distance(
            geometry::linear(_eefs[LEGTYPE_CF(sl)].head(2), _eefs[LEGTYPE_CH(sl)].head(2)),
            body_iface_->cog().head(2)),
        geometry::distance(
            geometry::linear(_eefs[LEGTYPE_IL(sl)].head(2), _eefs[LEGTYPE_SL(sl)].head(2)),
            body_iface_->cog().head(2)),
        geometry::distance(
            geometry::linear(_eefs[LEGTYPE_IL(sl)].head(2), _eefs[LEGTYPE_DL(sl)].head(2)),
            body_iface_->cog().head(2)));
  }
}


void Creep::prog_cog_traj(const Eigen::Vector2d& _next_cog) {
  Eigen::Vector3d _p0(0.0, 0.0, -cp_params_->STANCE_HEIGHT);
  Eigen::Vector3d _p1(0.0, 0.0, -cp_params_->STANCE_HEIGHT);
  // Eigen::Vector2d _tmp_next_cog = prog_next_cog(swing_leg_);

  FOREACH_LEG(l) {
    _p0 = leg_ifaces_[l]->eef();
    _p1.head(2) = _p0.head(2) - _next_cog.head(2);
    _p1.tail(1) = _p0.tail(1);

    Eigen::MatrixXd A;
    A.resize(4, 4);
    ///!  X = a0 + a1 *t + a2 * t^2 + a3 * t^3
    ///!  X'(0) = 0; X'(1) = 0; X(0) = p0; X(1) = p1;
    A << 0,   1,    0,     0,
         0,   1,    2,     3,
         1,   0,    0,     0,
         1,   1,    1,     1;

    Eigen::MatrixXd b;
    b.resize(4, 3);
    b.row(0) << 0, 0, 0;
    b.row(1) << 0, 0, 0;
    b.row(2) = _p0;
    b.row(3) = _p1;

    Eigen::MatrixXd c;
    if (0 == A.determinant()) {
      c = A.householderQr().solve(b);
      LOG_ERROR << "NO cross point, Using this result: \n" << c.transpose();
    } else {
      c = A.partialPivLu().solve(b);
    }

    cog2eef_traj_[l].reset(new PolyTraj3d(c.transpose()));
    cog2eef_traj_[l]->range(0, 1);

    ///! Just for debug information
    // std::cout << *(boost::dynamic_pointer_cast<PolyTraj3d>(cog2eef_traj_[l])) << std::endl;
  }
}

void Creep::prog_eef_traj(const Eigen::Vector3d& _next_fpt) {
  Eigen::Vector3d _last_fpt = leg_ifaces_[swing_leg_]->eef();
  // Eigen::Vector3d _next_fpt = prog_next_fpt(swing_leg_);

  Eigen::Vector3d _rise_fpt = _last_fpt;
  _rise_fpt.z() = _last_fpt.z() + 0.5*cp_params_->SWING_HEIGHT;

  Eigen::Vector3d _appr_fpt = _next_fpt;
  _appr_fpt.z() = _next_fpt.z() + 0.5*cp_params_->SWING_HEIGHT;

  Eigen::Vector3d _top_fpt(
      (_rise_fpt.x()/2 + _appr_fpt.x()/2),
      (_rise_fpt.y()/2 + _appr_fpt.y()/2),
      (_last_fpt.z() + cp_params_->SWING_HEIGHT));

  SegTraj3dSp swing_traj(new SegTraj3d);
  double _t0 = 0;
  double _t1 = _t0 + cp_params_->SWING_RISE;
  double _t3 = _t1 + cp_params_->SWING_APPROACH;
  double _t4 = _t3 + cp_params_->SWING_DROP;
  double _t2 = 0.5*(_t1 + _t3);
  Eigen::MatrixXd A, b, x;
  Eigen::VectorXd row;
  ///////////////////////////////////////////////////////////
  ///! THE RISE PHASE
  ///!  R = a0 + a1 *t + a2 * t^2 + a3 * t^3
  ///!  R'(t0) = 0; R''(t0) = 0; R(t0) = p0; R(t1) = p1;
  ///////////////////////////////////////////////////////////
  A.resize(4, 4);
  b.resize(4, 3);
//  Eigen::Vector3d _dif_3th_coef(1, 2, 3);
//  Eigen::VectorXd row = __get_state_vec(_t0, 3);
  ///! R'(t0) = 0;
  A.row(0) = __get_diff_vec(_t0, 4);
  // std::cout << A.row(0) << std::endl;
  // A.row(0)(0) = 0; A.row(0).tail(3) = _dif_3th_coef.cwiseProduct(row);
  b.row(0)    << 0, 0, 0;
  ///! R''(t0) = 0;
  Eigen::Vector2d _ddif_3th_coef(2*1, 3*2);
  row = __get_state_vec(_t0, 2);
  A.row(1).head(2).fill(0.0); A.row(1).tail(2) = _ddif_3th_coef.cwiseProduct(row);
  b.row(1) << 0, 0, 0;
  ///! R(t0) = last_fpt
  A.row(2) =  __get_state_vec(_t0, 4);
  b.row(2) = _last_fpt;
  ///! R(t1) = rise_fpt
  A.row(3) =  __get_state_vec(_t1, 4);
  b.row(3) = _rise_fpt;

//  std::cout << "rise - A:\n" << A << std::endl;
//  std::cout << "rise - b:\n" << b << std::endl;
  if (0 == A.determinant()) {
    x = A.householderQr().solve(b);
    std::cout << "NO trajectory results, Using this result: " << x.transpose() << std::endl;
  } else {
    x = A.partialPivLu().solve(b);
  }

  PolyTraj3dSp rise_traj(new PolyTraj3d(x.transpose()));
  rise_traj->range(_t0, _t1);
  ///////////////////////////////////////////////////////////
  ///! THE DROP PHASE
  ///!  D = a0 + a1 *t + a2 * t^2 + a3 * t^3
  ///!  D'(t4) = 0; D''(t4) = 0; D(t3) = p3; D(t4) = p4;
  ///////////////////////////////////////////////////////////
  A.resize(4, 4);
  b.resize(4, 3);
  ///! D'(t4)  = 0
  // row = __get_state_vec<double>(_t3, 3);
  A.row(0) = __get_diff_vec(_t4, 4);
  // A.row(0)(0) = 0; A.row(0).tail(3) = _dif_3th_coef.cwiseProduct(row);
  b.row(0).fill(0.0);
  ///! D''(t4) = 0
  row = __get_state_vec<double>(_t4, 2);
  A.row(1).head(2).fill(0.0); A.row(1).tail(2) = _ddif_3th_coef.cwiseProduct(row);
  b.row(1).fill(0.0);
  ///! D(t3) = p3
  A.row(2) = __get_state_vec<double>(_t3, 4);
  b.row(2) = _appr_fpt;
  ///! D(t4) = p4
  A.row(3) = __get_state_vec<double>(_t4, 4);
  b.row(3) = _next_fpt;

//  std::cout << "drop - A:\n" << A << std::endl;
//  std::cout << "drop - b:\n" << b << std::endl;
  if (0 == A.determinant()) {
    x = A.householderQr().solve(b);
    std::cout << "NO trajectory results, Using this result: " << x.transpose() << std::endl;
  } else {
    x = A.partialPivLu().solve(b);
  }

  PolyTraj3dSp drop_traj(new PolyTraj3d(x.transpose()));
  drop_traj->range(_t3, _t4);
  ///////////////////////////////////////////////////////////
  ///! THE APPROACH PHASE
  ///!  A = a0 + a1 *t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5 + a6 * t^6
  ///!  A(t1) = p1; A'(t1) = R'(t1); A''(t1) = R''(t1);
  ///!  A(t2) = p2
  ///!  A(t3) = p3; A'(t3) = D'(t3); A''(t3) = D''(t3);
  ///////////////////////////////////////////////////////////
  A.resize(7, 7);
  b.resize(7, 3);
  ///! A(t1) = p1
  A.row(0)    = __get_state_vec(_t1, 7);
  b.row(0)    = _rise_fpt;
  ///! A'(t1) = R'(t1)
  A.row(1)    = __get_diff_vec(_t1, 7);
  b.row(1)    = rise_traj->differential(_t1);
  ///! A''(t1) = R''(t1)
  Eigen::VectorXd _ddif_7th_coef;
  _ddif_7th_coef.resize(5);
  _ddif_7th_coef << (2*1), (3*2), (4*3), (5*4), (6*5);
  row         = __get_state_vec(_t1, 5);
  A.row(2).head(2).fill(0.0); A.row(2).tail(5) = _ddif_7th_coef.cwiseProduct(row);
  b.row(2)    = rise_traj->differential()->differential(_t1);
  ///! A(t2) = p2
  A.row(3) = __get_state_vec(_t2, 7);
  b.row(3) = _top_fpt;
  ///! A(t3) = p3
  A.row(4) = __get_state_vec(_t3, 7);
  b.row(4) = _appr_fpt;
  ///! A'(t3) = D'(t3)
  A.row(5) = __get_diff_vec(_t3, 7);
  b.row(5) = drop_traj->differential(_t3);
  ///! A''(t3) = D''(t3)
  row      = __get_state_vec(_t3, 5);
  A.row(6).head(2).fill(0.0); A.row(6).tail(5) = _ddif_7th_coef.cwiseProduct(row);
  b.row(6) = drop_traj->differential()->differential(_t3);

//  std::cout << "appr - A:\n" << A << std::endl;
//  std::cout << "appr - b:\n" << b << std::endl;
  if (0 == A.determinant()) {
    x = A.householderQr().solve(b);
    std::cout << "NO trajectory results, Using this result: " << x.transpose() << std::endl;
  } else {
    x = A.partialPivLu().solve(b);
  }

  PolyTraj3dSp appr_traj(new PolyTraj3d(x.transpose()));
  appr_traj->range(_t1, _t3);

  swing_traj->add(rise_traj, _t0, _t1);
  swing_traj->add(appr_traj, _t1, _t3);
  swing_traj->add(drop_traj, _t3, _t4);
  eef_traj_ = swing_traj;
  ///! Just for debug information
//  std::cout << "Swing Trajectory: " << std::endl;
//  std::cout << " - Rise phase: " << *rise_traj << std::endl;
//  std::cout << " - Appr phase: " << *appr_traj << std::endl;
//  std::cout << " - Drop phase: " << *drop_traj << std::endl;
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_control::Creep, Label)
