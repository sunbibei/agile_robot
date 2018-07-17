/*
 * walk.cpp
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

// #ifdef XXX

#include "gait/walk/walk.h"
#include "adt/segmented.h"
#include "adt/polynomial.h"
#include "adt/geometry.h"

#include <toolbox/time_control.h>
#include <foundation/cfg_reader.h>
#include <robot/agile_robot.h>

#ifdef DIS_JNT_LIMIT
#include <repository/joint_manager.h>
#endif

#include <limits>

namespace agile_control {

bool _s_is_hang = false;

#define JNTS_TARGET \
    leg_cmds_[LegType::FL]->target, \
    leg_cmds_[LegType::FR]->target, \
    leg_cmds_[LegType::HL]->target, \
    leg_cmds_[LegType::HR]->target

Eigen::Vector2d __incenter(const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

Eigen::Vector2d __cross_point(
    const Eigen::Vector2d& p0_0, const Eigen::Vector2d& p0_1,
    const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1);

struct WalkParam {
  ///! The threshold for cog
  double THRES_COG;
  ///! The foot step
  double FOOT_STEP;
  ///! The height value when the robot stay stance.
  double STANCE_HEIGHT;
  ///! The height value when swing leg.
  double SWING_HEIGHT;
  ///! The orientation of forward walk.
  double FORWARD_ALPHA;
  ///! The first target height scale
  double STOP_HEIGHT_SCALE;

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

  WalkParam(const std::string& _tag)
    : THRES_COG(6.5),    FOOT_STEP(10),
      STANCE_HEIGHT(46), SWING_HEIGHT(5),
      FORWARD_ALPHA(0),  STOP_HEIGHT_SCALE(0.3),
      COG_TIME(2000),    SWING_TIME(2000),
      MARGIN_THRES(6) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value(_tag, "cog_threshold",THRES_COG);
    cfg->get_value(_tag, "step",         FOOT_STEP);
    cfg->get_value(_tag, "stance_height",STANCE_HEIGHT);
    cfg->get_value(_tag, "stop_scale",   STOP_HEIGHT_SCALE);
    cfg->get_value(_tag, "swing_height", SWING_HEIGHT);
    cfg->get_value(_tag, "forward_orientation", FORWARD_ALPHA);

    cfg->get_value(_tag, "cog_time",     COG_TIME);
    cfg->get_value(_tag, "swing_time",   SWING_TIME);

    cfg->get_value(_tag, "swing_rise",     SWING_RISE);
    cfg->get_value(_tag, "swing_approach", SWING_APPROACH);
    cfg->get_value(_tag, "swing_drop",     SWING_DROP);

    cfg->get_value(_tag, "margin_threshold", MARGIN_THRES);
  }
};

struct TdParam {
  ///! The minimum delta to keep the robot stance.
  double   STANCE_DELTA;
  ///! In order to keep the stance allowed the minimum height.
  double   STANCE_FLOOR;
  ///! In order to keep the stance allowed the maximum height.
  double   STANCE_CEILING;
  ///! The target for legs
  double   TD_TARGET[LegType::N_LEGS];
  ///! The Kp
  double   TD_KP;

  TdParam(const std::string& _tag)
  : STANCE_DELTA(0.1), STANCE_FLOOR(45), STANCE_CEILING(49),
    TD_KP(0.01) {
    auto cfg = MiiCfgReader::instance();

    cfg->get_value(_tag, "stance_delta",     STANCE_DELTA);
    cfg->get_value(_tag, "stance_floor",     STANCE_FLOOR);
    cfg->get_value(_tag, "stance_ceiling",   STANCE_CEILING);
    cfg->get_value(_tag, "k_p",              TD_KP);

    std::vector<double> targets;
    cfg->get_value_fatal(_tag, "targets", targets);
    if (LegType::N_LEGS != targets.size()) {
      LOG_FATAL << "The size of touchdown parameters is wrong!";
    }

    for (size_t i = 0; i < targets.size(); ++i) {
      TD_TARGET[i] = targets[i];
    }
  }
};

Walk::Walk()
  : current_state_(WalkState::UNKNOWN_WK_STATE),
    body_iface_(nullptr),
    wk_params_(nullptr), td_params_(nullptr),
    timer_(nullptr), swing_timer_(nullptr), cog_timer_(nullptr),
    eef_traj_(nullptr), swing_leg_(LegType::UNKNOWN_LEG),
    post_tick_interval_(50)
{
  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  for (auto& c : leg_cmds_)
    c = nullptr;

  for (auto& t : cog2eef_traj_)
    t = nullptr;
//  Loop_Count = 0;

#ifdef PUB_ROS_TOPIC
  nh_.reset(new ros::NodeHandle("~"));
#endif
}

Walk::~Walk() {
#ifdef RECORDER_EEF_TRAJ
  eefs_traj_recorder_.save("/home/bibei/Workspaces/matlab/Trajectory/trajs.csv");
#endif

#ifdef PUB_ROS_TOPIC
  nh_.reset();
  cmd_pub_.reset();
#endif

  delete timer_;
  timer_ = nullptr;

  for (auto& iface : leg_ifaces_)
    iface = nullptr;

  for (auto& c : leg_cmds_) {
    delete c;
    c = nullptr;
  }
}

bool Walk::auto_init() {
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
  // cfg->get_value(getLabel(), "hang",     is_hang_walk_);
  cfg->get_value(getLabel(), "hang",     _s_is_hang);
  cfg->get_value(getLabel(), "interval", post_tick_interval_);

  std::string _tag = Label::make_label(getLabel(), "coefficient");
  wk_params_ = new WalkParam(_tag);

  _tag = Label::make_label(getLabel(), "touchdown");
  td_params_ = new TdParam(_tag);
  return true;
}

bool Walk::starting() {
  state_machine_ = new StateMachine<WalkState>(current_state_);
  auto _sm       = (StateMachine<WalkState>*)state_machine_;
  _sm->registerStateCallback(WalkState::WK_INIT_POSE, &Walk::pose_init, this);
  _sm->registerStateCallback(WalkState::WK_MOVE_COG,  &Walk::move_cog,  this);
  _sm->registerStateCallback(WalkState::WK_SWING,     &Walk::swing_leg, this);
  _sm->registerStateCallback(WalkState::WK_SWING_1,   &Walk::close_to_floor, this);
  _sm->registerStateCallback(WalkState::WK_STOP,      &Walk::stance,    this);
  _sm->registerStateCallback(WalkState::WK_SPIRALLING,&Walk::spiralling,this);
  _sm->registerStateCallback(WalkState::WK_HANG,      &Walk::hang_walk, this);

  _sm->registerStateCallback(WalkState::WK_WALK,      &Walk::walk, this);

  current_state_ = WalkState::WK_INIT_POSE;
#ifdef PUB_ROS_TOPIC
  cmd_pub_.reset(new realtime_tools::RealtimePublisher<
      std_msgs::Float64MultiArray>(*nh_, "/dragon/joint_commands", 10));
#endif

  ///! initialize the TimeControl
  timer_       = new TimeControl;
  swing_timer_ = new TimeControl;
  cog_timer_   = new TimeControl;

  eef_traj_.reset(new SegTraj3d);
  for (auto& t : cog2eef_traj_)
    t.reset(new PolyTraj3d);

  FOREACH_LEG(l) {
    leg_cmds_[l]   = new LegTarget;
    leg_cmds_[l]->cmd_type = JntCmdType::CMD_POS;
    leg_cmds_[l]->target.resize(3);

    stance_deltas_[l] = 0;
  }

  LOG_INFO << "The walk gait has STARTED!";
  return true;
}

void Walk::stopping() {
  for (auto& t : cog2eef_traj_) {
    t.reset();
  }
  eef_traj_.reset();

  FOREACH_LEG(l) {
    delete leg_cmds_[l];
    leg_cmds_[l] = nullptr;
  }

  delete timer_;
  timer_ = nullptr;
  delete swing_timer_;
  swing_timer_ = nullptr;
  delete cog_timer_;
  cog_timer_   = nullptr;

  delete state_machine_;
  state_machine_ = nullptr;

  LOG_INFO << "The walk gait has STOPPED!";
}

void Walk::checkState() {
  ///! the local static variable for every state time span.
  static int64_t _s_tmp_span = 0;

  switch (current_state_) {
  case WalkState::WK_WALK:
  {
    if (!timer_->running()) {
      static bool _s_is_first = true;
      ///! programming the swing trajectory.
      Eigen::Vector3d _next_eef = leg_ifaces_[swing_leg_]->eef();
      _next_eef.x() = wk_params_->FOOT_STEP;
      // prog_eef_traj(_next_eef);

      ///! propgraming the CoG trajectory.
      _next_eef    = _next_eef + body_iface_->leg_base(swing_leg_);
      Eigen::Vector3d _cf_eef = leg_ifaces_[LEGTYPE_CF(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_CF(swing_leg_));
      Eigen::Vector2d _last_cog = body_iface_->cog().head(2);
      Eigen::Vector2d _next_cog(_last_cog.x() + 0.5*wk_params_->FOOT_STEP, _last_cog.y());
      Eigen::Vector2d _cs(0.0, 0.0);
      if (LEGTYPE_IS_HIND(swing_leg_)) {
        _cs = geometry::cross_point(
            geometry::linear(_cf_eef.head(2), _next_eef.head(2)),
            geometry::linear(_next_cog, _last_cog));
        _next_cog     = (_cs + _last_cog) * 0.5;
        if (_s_is_first) {
          _next_cog.y() = _cf_eef.y() * 0.2;
          _s_is_first = false;
        }
      } else {
        // LegType _il = LEGTYPE_IL(swing_leg_);
        Eigen::Vector3d _il_eef = leg_ifaces_[LEGTYPE_IL(swing_leg_)]->eef() + body_iface_->leg_base(LEGTYPE_IL(swing_leg_));
        _cs = geometry::cross_point(
            geometry::linear(_cf_eef.head(2), _il_eef.head(2)),
            geometry::linear(_next_cog, _last_cog));
        _next_cog = _cs;
      }
      LOG_WARNING << "NEXT COG: " << _next_cog.transpose();
      // _next_cog.x() += 0.5 * _translation;
      prog_cog_traj(_next_cog);

      ///! start the timer
      timer_->start();
    }

    Eigen::Vector3d margins = stability_margin(swing_leg_);
    if ((!swing_timer_->running()) && 
          ((margins.minCoeff() >= wk_params_->MARGIN_THRES)
            || (cog_timer_->span() >= wk_params_->COG_TIME))) {
      LOG_WARNING << "STARTING...";
      ///! programming the swing trajectory.
      Eigen::Vector3d _next_eef = leg_ifaces_[swing_leg_]->eef();
      _next_eef.head(2) << wk_params_->FOOT_STEP, 0;
      prog_eef_traj(_next_eef);

      swing_timer_->start();
    }
    if (!cog_timer_->running()) {
      if (LEGTYPE_IS_HIND(swing_leg_))
        cog_timer_->start();

      if ((LEGTYPE_IS_FRONT(swing_leg_)) && (swing_timer_->span() >= 1000))
        cog_timer_->start();
    }

    if (!end_walk()) return;

    ///! the end of WK_INIT_POS
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----WALK ONE-STEP OK!("
        << _s_tmp_span << "ms)----*******";
//    swing_timer_->stop();
//    cog_timer_->stop();

    // PRESS_THEN_GO
    ///! updating the following swing leg
    swing_leg_     = next_leg(swing_leg_);
    break;
  }
  case WalkState::WK_INIT_POSE:
  {
    ///! the begin of WK_INIT_POS
    if (!timer_->running()) {
      // Eigen::Vector3d _tmp(0, 0, -params_->STANCE_HEIGHT);
      FOREACH_LEG(l) {
        leg_cmd_eefs_[l] << 0.0, 0.0, -wk_params_->STANCE_HEIGHT;
        // leg_ifaces_[l]->ik(_tmp, leg_cmds_[l]->target);
      }

      timer_->start();
    }

    if (!end_pose_init()) return;

    ///! the end of WK_INIT_POS
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----INIT POSE OK!("
        << _s_tmp_span << "ms)----*******";
    // print_jnt_pos(/*JNTS_TARGET*/);
    // print_eef_pos();

    PRESS_THEN_GO
    ///! updating the following swing leg
    swing_leg_     = next_leg(swing_leg_);
    current_state_ = WalkState::WK_MOVE_COG;
    // current_state_ = WalkState::WK_WALK;
    break;
  }
  case WalkState::WK_MOVE_COG:
  {
    ///! the begin of WK_MOVE_COG
    if (!timer_->running()) {
      prog_cog_traj(prog_next_cog(swing_leg_));

      // PRESS_THEN_GO
      timer_->start();
    }

    if (!end_move_cog()) return;

    ///! the end of WK_MOVE_COG
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END MVOE  COG("
        << _s_tmp_span << "ms)----*******";

//    Eigen::Vector3d _tfpt = cog2eef_traj_[0]->sample(1);
//    Eigen::Vector3d _fpt(0.0, 0.0, 0.0);
    print_eef_pos(
        cog2eef_traj_[LegType::FL]->sample(cog2eef_traj_[LegType::FL]->ceiling()),
        cog2eef_traj_[LegType::FR]->sample(cog2eef_traj_[LegType::FR]->ceiling()),
        cog2eef_traj_[LegType::HL]->sample(cog2eef_traj_[LegType::HL]->ceiling()),
        cog2eef_traj_[LegType::HR]->sample(cog2eef_traj_[LegType::HR]->ceiling()));
    // __print_positions(_cog, _tcog);

    PRESS_THEN_GO
    current_state_ = WalkState::WK_SWING;
    break;
  }
  case WalkState::WK_SWING:
  {
    ///! the begin of WK_SWING
    if (!timer_->running()) {
      // Eigen::Vector3d _next_fpt = prog_next_fpt();
      prog_eef_traj(prog_next_fpt(swing_leg_));
      // sum_interval_ = 0;
      timer_->start();
    }

//    auto diff = (eef_traj_->sample(eef_traj_->ceiling()).head(2)
//        - leg_ifaces_[swing_leg_]->eef().head(2)).norm();
//    LOG_EVERY_N(INFO, 20) << "Difference: " << diff;
//    if (diff < 0.2) {
//      LOG_INFO << "Change to WalkState::WK_SWING_1";
//      current_state_ = WalkState::WK_SWING_1;
//    }

    ///! judge whether is end
    if (!end_swing_leg()) return;

    ///! the end of WK_SWING
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END SWING LEG("
        << _s_tmp_span << "ms)----*******";

    Eigen::Vector3d _fpt = eef_traj_->sample(eef_traj_->ceiling());
    print_eef_pos(swing_leg_, _fpt);
    PRESS_THEN_GO

    ///! Every twice swing leg then adjusting COG.
    if ((LegType::FL == swing_leg_) || (LegType::FR == swing_leg_)) {
      current_state_ = WalkState::WK_MOVE_COG;
    } else {
      current_state_ = WalkState::WK_SWING;
    }
    ///! clear the old trajectory.
    eef_traj_.reset();
    ///! program the next swing leg
    swing_leg_ = next_leg(swing_leg_);
    break;
  }
  case WalkState::WK_SWING_1:
  {
    ///! judge whether is end
    if (!end_swing_leg()) return;

    ///! the end of WK_SWING
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END SWING LEG("
        << _s_tmp_span << "ms)----*******";

    print_eef_pos(swing_leg_, eef_traj_->sample(1));
    PRESS_THEN_GO

    Eigen::Vector3d _fpt = eef_traj_->sample(1);
    _fpt.z() -= 0.1*wk_params_->SWING_HEIGHT;
    leg_ifaces_[swing_leg_]->ik(_fpt, leg_cmds_[swing_leg_]->target);

    ///! Every twice swing leg then adjusting COG.
    if ((LegType::FL == swing_leg_) || (LegType::FR == swing_leg_)) {
      current_state_ = WalkState::WK_MOVE_COG;
    } else
      current_state_ = WalkState::WK_SWING;

    ///! program the next swing leg
    swing_leg_ = next_leg(swing_leg_);
    break;
  }
  case WalkState::WK_STOP:
  {
    ///! the begin of WK_SWING
    if (!timer_->running()) {
      timer_->start();
    }

    ///! the end of WK_SWING
    timer_->stop(&_s_tmp_span);
    LOG_WARNING << "*******----END STOP("
        << _s_tmp_span << "ms)----*******";

    print_eef_pos();
    PRESS_THEN_GO
    break;
  }
  case WalkState::WK_SPIRALLING:
  case WalkState::WK_HANG:
  default:
    LOG_ERROR << "What fucking walk state!";
    break;
  // Nothing to do here.
  }
}

//void Walk::prev_tick() {
//  // gesture->updateImuData(imu_quat_[0], imu_quat_[1], imu_quat_[2]);
//}

void Walk::post_tick() {
  ///! The command frequency control
  static TimeControl _s_post_tick(true);
  static int64_t     _s_sum_interval = 0;
  _s_sum_interval += _s_post_tick.dt();
  if (_s_sum_interval < post_tick_interval_) return;
  _s_sum_interval = 0;

  FOREACH_LEG(leg) {
    ///! Modify the target to keep the stance stability
    if (!_s_is_hang) {
      Eigen::Vector3d _eef = leg_ifaces_[leg]->eef();
      if (swing_leg_ != leg) {
        if (LegState::TD_STATE != leg_ifaces_[leg]->leg_state()) {
          double diff = td_params_->TD_TARGET[leg] - leg_ifaces_[leg]->foot_force();

          leg_cmd_eefs_[leg].z() = _eef.z() - td_params_->TD_KP*diff;
          leg_cmd_eefs_[leg].z() = __clamp<double>(leg_cmd_eefs_[leg].z(),
              -td_params_->STANCE_CEILING, -td_params_->STANCE_FLOOR);
        } else {
          leg_cmd_eefs_[leg].z() = _eef.z();
        }
      }
    }

    leg_ifaces_[leg]->eefPositionTarget(leg_cmd_eefs_[leg]);
//    leg_ifaces_[leg]->ik(leg_cmd_eefs_[leg], leg_cmds_[leg]->target);
//    leg_ifaces_[leg]->legTarget(*leg_cmds_[leg]);
    leg_ifaces_[leg]->move();
  }

  // Eigen::Vector3d margins = stability_margin(swing_leg_);
  // LOG_WARNING << "cog -> cf-ch:" << margins.x();
  // LOG_WARNING << "cog -> il-sl:" << margins.y();
  // LOG_WARNING << "cog -> il-dl:" << margins.z();

//  if (current_state_ == WalkState::WK_SWING) {
//    __print_positions(leg_ifaces_[swing_leg_]->eef(), eef_traj_->sample(1));
//  } else
   print_jnt_pos(JNTS_TARGET);
   print_eef_pos();

#ifdef RECORDER_EEF_TRAJ
    Discrete<double, 14>::StateVec vec;
    int idx = 0;
    FOREACH_LEG(l) {
      vec.head(idx + 3).tail(3) = leg_ifaces_[l]->eef();
      idx += 3;
    }
    vec(idx++) = swing_leg_;
    vec(idx++)   = current_state_;
    eefs_traj_recorder_.push_back(_s_post_tick.span()/1000.0, vec);
#endif


#ifdef PUB_ROS_TOPIC
  if(cmd_pub_->trylock()) {
    cmd_pub_->msg_.data.clear();
    for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
      auto& cmds = leg_cmds_[l]->target;
      leg_ifaces_[l]->ik(leg_cmd_eefs_[l], cmds);
      for (const auto& j : {JntType::KFE, JntType::HFE, JntType::HAA}) {
        cmd_pub_->msg_.data.push_back(cmds(j));
      }
    }
    cmd_pub_->unlockAndPublish();
  }
#endif
}

void Walk::pose_init() {
  // Nothing to do here.
  // print_eef_pos(leg_cmd_eefs_[LegType::FL], leg_cmd_eefs_[LegType::FR],
  //   leg_cmd_eefs_[LegType::HL], leg_cmd_eefs_[LegType::HR]);
}

bool Walk::end_pose_init() {
  FOREACH_LEG(l) {
    auto diff = (leg_ifaces_[l]->eef()
        - leg_cmd_eefs_[l]).norm();
    ///! The different of any leg is bigger than 0.1 is considered as
    ///! that the robot has not reach the initialization position.
    if (diff > 0.2) return false;
  }
  return true;
}

///! The flow of swing leg
///! This method only be called after the moving COG
LegType Walk::next_leg(const LegType curr) {
  switch (curr) {
  case LegType::FL: return LegType::HR;
  case LegType::FR: return LegType::HL;
  case LegType::HL: return LegType::FL;
  case LegType::HR: return LegType::FR;
  default: return LegType::HL; ///! The first swing HL leg.
  }
}

void Walk::hang_walk() {
  LOG_ERROR << "NO IMPLEMENT!";
  PRESS_THEN_GO
}

void Walk::move_cog() {
  if (!timer_->running())
    return;

  FOREACH_LEG(l) {
    leg_ifaces_[l]->ik(
        cog2eef_traj_[l]->sample((double)timer_->span()/wk_params_->COG_TIME),
        leg_cmds_[l]->target);
  }
}

bool Walk::end_move_cog() {
  ///! FL, FR, HL, HR and stability_margin
  Eigen::Vector3d margins = stability_margin(swing_leg_);

//  std::cout << "cog -> cf-ch:  " << margins.x() << std::endl;
//  std::cout << "cog -> il-sl:  " << margins.y() << std::endl;
//  std::cout << "cog -> il-dl:  " << margins.z() << std::endl;

  return (margins.minCoeff() > wk_params_->MARGIN_THRES);
}

Eigen::Vector3d Walk::stability_margin(LegType sl) {
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

void Walk::swing_leg() {
  if (!timer_->running())
    return;

//  auto diff = (eef_traj_->sample(eef_traj_->ceiling()) - leg_ifaces_[swing_leg_]->eef()).norm();
//  if (diff > 0.1) {
    leg_ifaces_[swing_leg_]->ik(
        eef_traj_->sample(timer_->span()/1000.0), leg_cmds_[swing_leg_]->target);
//  } else {
//    close_to_floor();
//  }
}

void Walk::close_to_floor() {
  ctf_eef_[swing_leg_].head(2) = eef_traj_->sample(eef_traj_->ceiling()).head(2);
  auto _sl_fpt = leg_ifaces_[swing_leg_]->eef();
  auto _dl_fpt = leg_ifaces_[LEGTYPE_DL(swing_leg_)]->eef();
  ///! print the position.
  print_eef_pos(swing_leg_, _dl_fpt);

  if (std::abs(_dl_fpt.z() - _sl_fpt.z()) < 0.3) {
    if (LEGTYPE_IS_HIND(swing_leg_)) {
      ctf_eef_[swing_leg_].z() = ctf_eef_[swing_leg_].z() - 0.01;
      ctf_eef_[LEGTYPE_DL(swing_leg_)]     = _dl_fpt;
      ctf_eef_[LEGTYPE_DL(swing_leg_)].z() = std::min(ctf_eef_[LEGTYPE_DL(swing_leg_)].z(), ctf_eef_[swing_leg_].z());
      ctf_eef_[swing_leg_].z()             = ctf_eef_[LEGTYPE_DL(swing_leg_)].z();
      ///! setting the DL command
      leg_ifaces_[LEGTYPE_DL(swing_leg_)]->ik(
          ctf_eef_[LEGTYPE_DL(swing_leg_)], leg_cmds_[LEGTYPE_DL(swing_leg_)]->target);
      LOG_INFO << "Setting DL target: " << ctf_eef_[swing_leg_].z();
    } else
      ; // Nothing to do here, waiting to end the swing leg state.
  } else {
    ctf_eef_[swing_leg_].z() = _dl_fpt.z();
    LOG_INFO << "Setting SL target: " << ctf_eef_[swing_leg_].z();
  }
  // LOG_WARNING << "close to floor";
  ///! setting command.
  leg_ifaces_[swing_leg_]->ik(
      ctf_eef_[swing_leg_], leg_cmds_[swing_leg_]->target);
}

bool Walk::end_swing_leg() {
 // auto diff = (ctf_eef_[swing_leg_] - leg_ifaces_[swing_leg_]->eef()).norm();
//  auto diff = (eef_traj_->sample(eef_traj_->ceiling()).head(2)
//                 - leg_ifaces_[swing_leg_]->eef().head(2)).norm();

  ///! for real robot
 // return ( (diff < 1) ? ((LEGTYPE_IS_FRONT(swing_leg_)) ?
 //           ( (timer_->span() > 2*params_->SWING_TIME) && (diff < 0.3) )
 //           : (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()) )
 //           : false );

//  return ( (diff < 1) && ( (LEGTYPE_IS_HIND(swing_leg_)) ?
//      (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state())
//      : (std::abs(leg_ifaces_[LEGTYPE_SL(swing_leg_)]->eef().z() - leg_ifaces_[swing_leg_]->eef().z()) < 0.1) ) );

  ///! for rviz
  return ((LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()) || (timer_->span() > 2*wk_params_->SWING_TIME));

}

// TODO
void Walk::walk() {
  if (!timer_->running())
    return;

  if (swing_timer_->running()) {
    double _dt = swing_timer_->span()/1000.0;
    if ((_dt >= 0.8*eef_traj_->ceiling()) && (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state()))
      leg_cmd_eefs_[swing_leg_] = leg_ifaces_[swing_leg_]->eef();
    else
      leg_cmd_eefs_[swing_leg_] = eef_traj_->sample(swing_timer_->span()/1000.0);
  }

  if (cog_timer_->running()) {
    FOREACH_LEG(l) {
      if (!swing_timer_->running() || swing_leg_ != l) {
        leg_cmd_eefs_[l] = cog2eef_traj_[l]->sample((double)cog_timer_->span()/wk_params_->COG_TIME);
  //      leg_ifaces_[l]->ik(
  //          cog2eef_traj_[l]->sample((double)timer_->span()/params_->COG_TIME),
  //          leg_cmds_[l]->target);
      }
    }
  }

}

bool Walk::end_walk() {
  if (_s_is_hang) {
    if (swing_timer_->span() >= 2000)
      swing_timer_->stop();
    if (cog_timer_->span() >= wk_params_->COG_TIME)
      cog_timer_->stop();

  } else {
    ///! the real walk.
    if ((swing_timer_->span() >= 0.6*2000)
          && (LegState::TD_STATE == leg_ifaces_[swing_leg_]->leg_state())) {
      swing_timer_->stop();
    }

    if (cog_timer_->span() >= /*2**/wk_params_->COG_TIME)
      cog_timer_->stop();
  }

  return (!swing_timer_->running() && !cog_timer_->running());
}

// TODO
void Walk::stance() {
//  FOR_EACH_LEG(l) {
//    ;
//  }
}

bool Walk::end_stance() {
  Eigen::Vector3d _tmp(0.0, 0.0, wk_params_->STANCE_HEIGHT);
  // ;
  FOREACH_LEG(l) {
    auto diff  = (cog2eef_traj_[l]->sample(1) - leg_ifaces_[l]->eef()).norm();
    if (diff > 0.1) return false;
  }
  return true;
}

// TODO
void Walk::spiralling() {
  ;
}

void Walk::prog_eef_traj(const Eigen::Vector3d& _next_fpt) {
  Eigen::Vector3d _last_fpt = leg_ifaces_[swing_leg_]->eef();
  // Eigen::Vector3d _next_fpt = prog_next_fpt(swing_leg_);

  Eigen::Vector3d _rise_fpt = _last_fpt;
  _rise_fpt.z() = _last_fpt.z() + 0.5*wk_params_->SWING_HEIGHT;

  Eigen::Vector3d _appr_fpt = _next_fpt;
  _appr_fpt.z() = _next_fpt.z() + 0.5*wk_params_->SWING_HEIGHT;

  Eigen::Vector3d _top_fpt(
      (_rise_fpt.x()/2 + _appr_fpt.x()/2),
      (_rise_fpt.y()/2 + _appr_fpt.y()/2),
      (_last_fpt.z() + wk_params_->SWING_HEIGHT));

  SegTraj3dSp swing_traj(new SegTraj3d);
  double _t0 = 0;
  double _t1 = _t0 + wk_params_->SWING_RISE;
  double _t3 = _t1 + wk_params_->SWING_APPROACH;
  double _t4 = _t3 + wk_params_->SWING_DROP;
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

Eigen::Vector3d Walk::prog_next_fpt(LegType _fsl) {
  Eigen::Vector3d _last_fpt = leg_ifaces_[_fsl]->eef();
  Eigen::Vector3d _next_fpt = _last_fpt;
  Eigen::Vector3d _other_fpt= _last_fpt;

  auto _other_leg = LEGTYPE_SL(_fsl);
  if (LegType::UNKNOWN_LEG != _other_leg)
    _other_fpt = leg_ifaces_[_other_leg]->eef();

  if (std::abs(_other_fpt.x() - _last_fpt.x()) > 0.5*wk_params_->FOOT_STEP) {
    _next_fpt.x() = _other_fpt.x() + wk_params_->FOOT_STEP;
  } else {
    _next_fpt.x() += wk_params_->FOOT_STEP;
    // _next_fpt.y()  = _last_fpt.y() + std::abs(_next_fpt.x() - _last_fpt.x()) * tan(params_->FORWARD_ALPHA);
  }
  _next_fpt.y()  = _last_fpt.y() + std::abs(_next_fpt.x() - _last_fpt.x()) * tan(wk_params_->FORWARD_ALPHA);
  _next_fpt.z() = _last_fpt.z()/* + 0.3*params_->SWING_HEIGHT*/;

  return _next_fpt;
}

void Walk::prog_cog_traj(const Eigen::Vector2d& _next_cog) {
  Eigen::Vector3d _p0(0.0, 0.0, -wk_params_->STANCE_HEIGHT);
  Eigen::Vector3d _p1(0.0, 0.0, -wk_params_->STANCE_HEIGHT);
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
    std::cout << *(boost::dynamic_pointer_cast<PolyTraj3d>(cog2eef_traj_[l])) << std::endl;
  }
}

Eigen::Vector2d Walk::prog_next_cog(LegType _fsl) {
//  Eigen::Vector2d _next_cog_proj(0.0, 0.0), _p[3];
  Eigen::Vector2d _last_cog_proj(0.0, 0.0), _lwp[LegType::N_LEGS];
  _last_cog_proj = body_iface_->cog().head(2);

  FOREACH_LEG(l) {
    if (l != _fsl)
      _lwp[l] = (leg_ifaces_[l]->eef() + body_iface_->leg_base(l)).head(2);
    else
      _lwp[l] = (prog_next_fpt(_fsl).head(2) + body_iface_->leg_base(l).head(2));
  }
  auto _cs = geometry::cross_point(
      geometry::linear(_lwp[LegType::FL], _lwp[LegType::HR]),
      geometry::linear(_lwp[LegType::FR], _lwp[LegType::HL]));
  auto _next_cog_proj = geometry::incenter_of_triangle(_cs, _lwp[LEGTYPE_CF(_fsl)], _lwp[LEGTYPE_CH(_fsl)]);

//  std::cout << "fl-hr: " << _lwp[LegType::FL].transpose() << "; " << _lwp[LegType::HR].transpose() << std::endl;
//  std::cout << "fl-hr: " << geometry::linear(_lwp[LegType::FL], _lwp[LegType::HR]).transpose() << std::endl;
//  std::cout << "fr-hl: " << _lwp[LegType::FR].transpose() << "; " << _lwp[LegType::HL].transpose() << std::endl;
//  std::cout << "fr-hl: " << geometry::linear(_lwp[LegType::FR], _lwp[LegType::HL]).transpose() << std::endl;
//  std::cout << "cs:    " << _cs.transpose() << std::endl;
//  std::cout << "cog:   " << _next_cog_proj  << std::endl;

  return _next_cog_proj;
//  _p[0] = __cross_point(_lwp[LegType::FL], _lwp[LegType::HR],
//      _lwp[LegType::FR], _lwp[LegType::HL]);
//  _p[1] = _lwp[LEGTYPE_CF(_fsl)];
//  if ((LegType::FL == _fsl) || (LegType::HL == _fsl)) {
//    _p[1] = (leg_ifaces_[LegType::FR]->eef() + body_iface_->leg_base(LegType::FR)).head(2);
//    _p[2] = (leg_ifaces_[LegType::HR]->eef() + body_iface_->leg_base(LegType::HR)).head(2);
//  } else {
//    _p[1] = (leg_ifaces_[LegType::FL]->eef() + body_iface_->leg_base(LegType::FL)).head(2);
//    _p[2] = (leg_ifaces_[LegType::HL]->eef() + body_iface_->leg_base(LegType::HL)).head(2);
//  }
//  _next_cog_proj = geometry::incenter_of_triangle(_p[0], _p[1], _p[2]);
//  // _next_cog_proj = __incenter(_p[0], _p[1], _p[2]);
//  return _next_cog_proj;
//
//  int idx = 0;
//  FOR_EACH_LEG(l) {
//    if (l == _fsl) continue;
//    _p[idx++] = (leg_ifaces_[l]->eef()
//        + body_iface_->leg_base(l)).head(2);
//  }
//
//  return __incenter(_p[0], _p[1], _p[2]);
}

//Eigen::Vector2d Walk::cog_proj1() {
//  if (LegType::UNKNOWN_LEG == swing_leg_)
//    return Eigen::Vector2d(0.0, 0.0);
//
//  double _x = 0;
//  double _y = 0;
//  Eigen::Vector2d _eef_proj(0.0, 0.0);
//  FOR_EACH_LEG(l) {
//    if (swing_leg_ == l) continue;
//    _eef_proj = (leg_ifaces_[l]->eef()
//        + body_iface_->leg_base(l)).head(2);
//    _x += _eef_proj.x();
//    _y += _eef_proj.y();
//  }
//
//  return Eigen::Vector2d(_x, _y);
//
//  Eigen::Vector2d _cs, _p0, _p1, _p2, _p3;
//  _p2 = (leg_ifaces_[LegType::FL]->eef()
//      + body_iface_->leg_base(LegType::FL)).head(2);
//
//  _p3 = (leg_ifaces_[LegType::HR]->eef()
//      + body_iface_->leg_base(LegType::HR)).head(2);
//
//  _p0 = (leg_ifaces_[LegType::FR]->eef()
//      + body_iface_->leg_base(LegType::FR)).head(2);
//
//  _p1 = (leg_ifaces_[LegType::HL]->eef()
//      + body_iface_->leg_base(LegType::HL)).head(2);
//
//  return _cs;
//}

//void Walk::prog_next_fpt() {
//  last_foot_pos_ = leg_ifaces_[swing_leg_]->eef();
//  // last_foot_pos_ = foots_pos_[swing_leg_];
//  // next_foot_pos_ << coeff_->FOOT_STEP, last_foot_pos_.y(), -coeff_->STANCE_HEIGHT;
//  next_foot_pos_ = last_foot_pos_;
//  next_foot_pos_.x() += coeff_->FOOT_STEP;
//}

// Eigen::Vector2d Walk::prog_next_cog(LegType) {
//  if ((LegType::FL == _swing_leg) || (LegType::HL == _swing_leg)) {
//    _p0 = (leg_ifaces_[LegType::FR]->eef()
//        + body_iface_->leg_base(LegType::FR)).head(2);
//    _p1 = (leg_ifaces_[LegType::HR]->eef()
//        + body_iface_->leg_base(LegType::HR)).head(2);
//    _next_cog_proj = __incentre(_last_cog_proj, _p0, _p1);
//  } else if ((LegType::FL == _swing_leg) || (LegType::HL == _swing_leg)) {
//    _p0 = (leg_ifaces_[LegType::FL]->eef()
//        + body_iface_->leg_base(LegType::FL)).head(2);
//    _p1 = (leg_ifaces_[LegType::HL]->eef()
//              + body_iface_->leg_base(LegType::HL)).head(2);
//    _next_cog_proj = inner_triangle(_last_cog_proj, _p0, _p1);
//  }
//  return _next_cog_proj;

//  if ((LegType::FL == swing_leg_) || (LegType::HL == swing_leg_)) {
//    _p0 = (leg_ifaces_[LegType::FR]->eef()
//        + body_iface_->leg_base(LegType::FR)).head(2);
//    _p1 = (leg_ifaces_[LegType::HR]->eef()
//        + body_iface_->leg_base(LegType::HR)).head(2);
//    _next_cog_proj = inner_triangle(_last_cog_proj,_p0, _p1);
//  } else {
//    _p0 = (leg_ifaces_[LegType::FL]->eef()
//        + body_iface_->leg_base(LegType::FL)).head(2);
//    _p1 = (leg_ifaces_[LegType::HL]->eef()
//              + body_iface_->leg_base(LegType::HL)).head(2);
//    _next_cog_proj = inner_triangle(_last_cog_proj, _p0, _p1);
//  }
//  return _next_cog_proj;

//  Eigen::Vector2d _cs, _p0, _p1, _p2, _p3;
//  Eigen::Vector3d _s;
//
//  switch (leg)
//  {
//    case LegType::FL:
//    case LegType::HL:
//      // TODO
//      // body_iface_->leg_base(LegType::FL, _s);
//      _p0 = (leg_ifaces_[LegType::FL]->eef()
//          + body_iface_->leg_base(LegType::FL)).head(2);
//
//      // body_iface_->leg_base(LegType::HR, _s);
//      _p1 = (leg_ifaces_[LegType::HR]->eef()
//          + body_iface_->leg_base(LegType::HR)).head(2);
//
//      // body_iface_->leg_base(LegType::FR, _s);
//      _p2 = (leg_ifaces_[LegType::FR]->eef()
//          + body_iface_->leg_base(LegType::FR)).head(2);
//
//      // body_iface_->leg_base(LegType::HL, _s);
//      _p3 = (leg_ifaces_[LegType::HL]->eef()
//          + body_iface_->leg_base(LegType::HL)).head(2);
//      break;
//    case LegType::FR:
//    case LegType::HR:
//      // body_iface_->leg_base(LegType::FL, _s);
//      _p2 = (leg_ifaces_[LegType::FL]->eef()
//          + body_iface_->leg_base(LegType::FL)).head(2);
//
//      // body_iface_->leg_base(LegType::HR, _s);
//      _p3 = (leg_ifaces_[LegType::HR]->eef()
//          + body_iface_->leg_base(LegType::HR)).head(2);
//
//      // body_iface_->leg_base(LegType::FR, _s);
//      _p0 = (leg_ifaces_[LegType::FR]->eef()
//          + body_iface_->leg_base(LegType::FR)).head(2);
//
//      // body_iface_->leg_base(LegType::HL, _s);
//      _p1 = (leg_ifaces_[LegType::HL]->eef()
//          + body_iface_->leg_base(LegType::HL)).head(2);
//      break;
//    default:
//      LOG_WARNING << "What fucking code with LEG!";
//      return Eigen::Vector2d(0.0, 0.0);
//  }
//
//  _cs = cog_proj();
//  std::cout << "cs1: " << _cs.transpose() << std::endl;
//
//  __cross_point(_p0, _p1, _p2, _p3, _cs);
//  std::cout << "cs1: " << _cs.transpose() << std::endl;
//
//  Eigen::Vector2d it = inner_triangle(_cs, _p2, _p1);
//
//
//
//  std::cout << "p0: " << _p0.transpose() << std::endl;
//  std::cout << "p1: " << _p1.transpose() << std::endl;
//  std::cout << "p2: " << _p2.transpose() << std::endl;
//  std::cout << "p3: " << _p3.transpose() << std::endl;
//  std::cout << "cs: " << _cs.transpose() << std::endl;
//  std::cout << "it: " << it.transpose() << std::endl;
//
//  return it;

  // return inner_triangle(_cs, _p2, _p1);
// }

//Eigen::Vector2d Walk::inner_triangle(
//    const Eigen::Vector2d& _a, const Eigen::Vector2d& _b, const Eigen::Vector2d& _c) {
//  Eigen::Vector2d _init_cog(0.0, 0.0);
//  double ratio = 1;
//  double radius = __inscribed_circle_radius(_a, _b, _c);
//
//  auto iheart = __inner_heart(_a, _b, _c);
//  if ( radius > coeff_->THRES_COG) {
//    ratio = (radius - coeff_->THRES_COG)/radius;
//    auto _ia = __line_section(_a, iheart, ratio);
//    auto _ib = __line_section(_b, iheart, ratio);
//    auto _ic = __line_section(_c, iheart, ratio);
//
//    Eigen::Vector2d _cs1, _cs2;
//    __cross_point(_ia, _ib, _init_cog, iheart, _cs1);
//    __cross_point(_ia, _ic, _init_cog, iheart, _cs2);
//
//    if (_cs1.x() <= std::max(_ia.x(), _ib.x())
//      && _cs1.x() >= std::min(_ia.x(), _ib.x())) {
//        swing_delta_cog_ = (_ib.x() > _ic.x()) ? _ib - _cs1 : _ic - _cs1;
//        swing_delta_cog_ = swing_delta_cog_ / 2.0;
//        return _cs1;
//    }
//    if (_cs2.x() <= std::max(_ia.x(), _ic.x())
//      && _cs2.x() >= std::min(_ia.x(), _ic.x())) {
//      // std::cout<<"second_cross true"<<std::endl;
//      swing_delta_cog_ = (_ib.x() > _ic.x()) ? _ib - _cs2 : _ic - _cs2;
//      swing_delta_cog_ = swing_delta_cog_ / 2.0;
//      // return _cs2;
//    }
//    return _cs2;
//  } else {
//    return iheart;
//  }
//}

//Eigen::Vector2d Walk::stance_velocity(const Eigen::Vector2d& _target_cog, int64_t _span) {
//  double T  = coeff_->COG_TIME;
//  double dt = _span / T;
//
//  // TODO
//  // Eigen::Vector3d samp = eef2cog_traj_->sample(dt);
//  // return (samp.head(2) - _target_cog);
//
//  Eigen::Vector2d stance_vel(0.0, 0.0);
//  stance_vel.x() = (30 * _target_cog.x() * pow(dt, 4)
//      - 60 * _target_cog.x() * pow(dt, 3)
//      + 30 * _target_cog.x() * pow(dt, 2)) / T;
//
//  stance_vel.y() = (30 * _target_cog.y() * pow(dt, 4)
//      - 60 * _target_cog.y() * pow(dt, 3)
//      + 30 * _target_cog.y() * pow(dt, 2)) / T;
//  return stance_vel;
//}
//
//Eigen::Vector2d Walk::stance_velocity(const Eigen::Vector2d& Adj_vec, int Loop) {
//
//  Eigen::Vector2d stance_vel(0.0, 0.0);
//  stance_vel.x() = 30 * Adj_vec.x() * pow(Loop,4) / pow(50, 5)
//      - 60 * Adj_vec.x() * pow(Loop,3) / pow(50, 4)
//      + 30 * Adj_vec.x() * pow(Loop,2) / pow(50, 3);
//
//  stance_vel.y() = 30 * Adj_vec.y() * pow(Loop,4) / pow(50, 5)
//      - 60 * Adj_vec.y() * pow(Loop,3) / pow(50, 4)
//      + 30 * Adj_vec.y() * pow(Loop,2) / pow(50, 3);
//  return stance_vel;
//}

///////////////////////////////////////////////////////////////////////////////
////////////        The implementation of inline methods         //////////////
///////////////////////////////////////////////////////////////////////////////
Eigen::Vector2d __incenter(
    const Eigen::Vector2d& A, const Eigen::Vector2d& B, const Eigen::Vector2d& C) {
  double a = (B - C).norm();
  double b = (A - C).norm();
  double c = (A - B).norm();
  double sum = a + b + c;
  return Eigen::Vector2d(
      (a*A.x() + b*B.x() + c*C.x())/sum,(a*A.y() + b*B.y() + c*C.y())/sum);
}

Eigen::Vector2d __cross_point(
    const Eigen::Vector2d& p0_0, const Eigen::Vector2d& p0_1,
    const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1) {
  Eigen::Vector2d res;
  Eigen::Matrix2d cof_mat;
  Eigen::Vector2d beta;

  if (p0_0.x() != p0_1.x()) {
    cof_mat(0, 0) = -(p0_1.y() - p0_0.y()) / (p0_1.x() - p0_0.x());
    cof_mat(0, 1) = 1;
  } else {
    cof_mat(0, 0) = 1;
    cof_mat(0, 1) = 0;
  }
  beta(0) = cof_mat.row(0) * p0_0;

  if (p1_0.x() != p1_1.x()) {
    cof_mat(1, 0) = -(p1_1.y() - p1_0.y()) / (p1_1.x() - p1_0.x());
    cof_mat(1, 1) = 1;
  } else {
    cof_mat(1, 0) = 1;
    cof_mat(1, 1) = 0;
  }
  beta(1) = cof_mat.row(1) * p1_0;
  Eigen::Vector2d x(0., 0.);
  if (0 == cof_mat.determinant()) {
    res = cof_mat.householderQr().solve(beta);
    std::cout << "NO cross point, Using the result: " << res.transpose() << std::endl;
  } else {
    res = cof_mat.partialPivLu().solve(beta);
  }

  return res;
}

} /* namespace qr_control */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_control::Walk, Label)
