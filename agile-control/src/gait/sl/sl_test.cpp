/*
 * sl_test.cpp
 *
 *  Created on: Mar 10, 2018
 *      Author: bibei
 */

#include "gait/sl/sl_test.h"
#include "robot/agile_robot.h"

#include "adt/segmented.h"
#include "adt/polynomial.h"

#include "toolbox/time_control.h"
#include "foundation/cfg_reader.h"

#include <vector>
#include <thread>

namespace agile_control {

struct WSParam {
  ///! The foot step
  std::vector<double>   floors;
  ///! The height value when the robot stay stance.
  std::vector<double>   ceilings;

  WSParam(const std::string& _tag) {
    auto cfg = MiiCfgReader::instance();
    cfg->get_value(_tag, "floors",   floors);
    cfg->get_value(_tag, "ceilings", ceilings);
  }
};


struct LTParam {
    ///! The foot step
    double   FOOT_STEP;
    ///! The height value when the robot stay stance.
    double   INIT_HEIGHT;
    ///! The height value when swing leg.
    double   SWING_HEIGHT;

    ///! The time for swing leg (in ms)
    int64_t   SWING_TIME;

    LTParam(const std::string& _tag)
    : FOOT_STEP(10),   INIT_HEIGHT(46),
      SWING_HEIGHT(5), SWING_TIME(1000) {
      auto cfg = MiiCfgReader::instance();
      cfg->get_value(_tag, "step",         FOOT_STEP);
      cfg->get_value(_tag, "swing_height", SWING_HEIGHT);
      cfg->get_value(_tag, "init_height",  INIT_HEIGHT);

      cfg->get_value(_tag, "swing_time",     SWING_TIME);
    }
};

//struct LTParam {
//  ///! The foot step
//  double   FOOT_STEP;
//  ///! The height value when the robot stay stance.
//  double   STANCE_HEIGHT;
//  ///! The height value when swing leg.
//  double   SWING_HEIGHT;
//  ///! The time for swing leg in rise phase (in s)
//  double   SWING_RISE;
//  ///! The time for swing leg in approach phase (in s)
//  double   SWING_APPROACH;
//  ///! The time for swing leg in drop phase (in s)
//  double   SWING_DROP;
//
//  ///! The time for swing leg (in ms)
//  int64_t   SWING_TIME;
//
//  LTParam(const std::string& _tag)
//  : FOOT_STEP(10),       STANCE_HEIGHT(46),
//    SWING_HEIGHT(5),     SWING_RISE(0.5),
//    SWING_APPROACH(1.0), SWING_DROP(0.5),
//    SWING_TIME(1000) {
//
//    auto cfg = MiiCfgReader::instance();
//    cfg->get_value(_tag, "step",         FOOT_STEP);
//    cfg->get_value(_tag, "swing_height", SWING_HEIGHT);
//    cfg->get_value(_tag, "stance_height",STANCE_HEIGHT);
//
//    cfg->get_value(_tag, "swing_rise",     SWING_RISE);
//    cfg->get_value(_tag, "swing_approach", SWING_APPROACH);
//    cfg->get_value(_tag, "swing_drop",     SWING_DROP);
//
//    cfg->get_value(_tag, "swing_time",     SWING_TIME);
//  }
//};


SlTest::SlTest()
  : current_state_(LTestState::UNKNOWN_LT_STATE),
    leg_type_(LegType::UNKNOWN_LEG), leg_iface_(nullptr),
    leg_cmd_(nullptr), swing_timer_(nullptr),
    ws_params_(nullptr),
    params_(nullptr),  thread_alive_(false) {
  ;
}

SlTest::~SlTest() {
  stopping();
}

bool SlTest::auto_init() {
  if (!GaitBase::auto_init()) return false;
  auto cfg = MiiCfgReader::instance();

  auto ifaces = LegRobot::instance();
  if (!ifaces) {
    LOG_FATAL << "The interface for robot is null!";
    return false;
  }

  cfg->get_value(getLabel(), "traj_type", traj_type_);
  cfg->get_value(getLabel(), "leg",       leg_type_);
  if (LegType::UNKNOWN_LEG == leg_type_) {
    leg_type_ = LegType::FL;
    LOG_WARNING << "No define the leg type, using the default value "
        << LEGTYPE2STR(leg_type_);
  }
  leg_iface_ = ifaces->robot_leg(leg_type_);
  if (!leg_iface_)
        LOG_FATAL << "The interface of RobotLeg " << LEGTYPE2STR(leg_type_) << " is null!";
  LOG_INFO << "get interface(LegType::" << LEGTYPE2STR(leg_type_) << "): " << leg_iface_;

  params_    = new LTParam(Label::make_label(getLabel(), "trajectory"));
  ws_params_ = new WSParam(Label::make_label(getLabel(), "workspace"));

  init_foothold_ << -params_->FOOT_STEP*0.5, 0, -params_->INIT_HEIGHT;
  goal_foothold_ <<  params_->FOOT_STEP*0.5, 0, -params_->INIT_HEIGHT;

  LOG_INFO << "init: " << init_foothold_.transpose();
  LOG_INFO << "goal: " << goal_foothold_.transpose();
  return true;
}

bool SlTest::starting() {
  state_machine_ = new StateMachine<LTestState>(current_state_);
  auto _sm       = (StateMachine<LTestState>*)state_machine_;
  _sm->registerStateCallback(LTestState::LT_INIT_POSE, &SlTest::pose_init, this);
  _sm->registerStateCallback(LTestState::LT_SWING,     &SlTest::swing_leg, this);
  _sm->registerStateCallback(LTestState::LT_WS_CALC,   &SlTest::ws_calc,   this);
  _sm->registerStateCallback(LTestState::LT_PROD_TRAJ, &SlTest::prod_traj, this);

  current_state_ = LTestState::LT_INIT_POSE;
  // current_state_ = LTestState::LT_PROD_TRAJ;

  leg_cmd_   = new LegTarget;
  leg_cmd_->cmd_type = JntCmdType::CMD_POS;
  leg_cmd_->target.resize(3);

  swing_timer_ = new TimeControl;

  LOG_INFO << "The SlTest gait has STARTED!";

  PRESS_THEN_GO
  return true;
}

void SlTest::stopping() {
  delete leg_cmd_;
  leg_cmd_ = nullptr;

  delete swing_timer_;
  swing_timer_ = nullptr;

  delete state_machine_;
  state_machine_ = nullptr;

  LOG_INFO << "The SlTest gait has STOPPED!";
}

void SlTest::post_tick() {
  ///! The command frequency control
//  static TimeControl   _s_post_tick(true);
//  static const int64_t _s_post_tick_interval = 2;
//  static int64_t       _s_sum_interval = 0;
//  _s_sum_interval += _s_post_tick.dt();
//  if (_s_sum_interval < _s_post_tick_interval) return;
//  _s_sum_interval = 0;

  leg_iface_->eefPositionTarget(leg_cmd_eef_);
  leg_iface_->move();
}

void SlTest::checkState() {
  switch (current_state_) {
  case LTestState::LT_INIT_POSE:
  {
    if (!end_pose_init()) return;

    LOG_INFO << "Arrival the initialization pose.";
    PRESS_THEN_GO
    current_state_ = LTestState::LT_SWING;
    break;
  }
  case LTestState::LT_SWING:
  {
    if (!end_swing_leg()) return;

    LOG_INFO << "Finished swing leg.";
    PRESS_THEN_GO
    swing_timer_->stop();
    current_state_ = LTestState::LT_INIT_POSE;
    break;
  }
  default:
    LOG_ERROR << "What fucking walk state!";
    break;
  // Nothing to do here.
  }
}

void SlTest::pose_init() {
  leg_cmd_eef_ = init_foothold_;
}

bool SlTest::end_pose_init() {
  const static double _epsilon = 1;
  print_eef_pos(leg_type_, init_foothold_);
  Eigen::VectorXd angles;
  leg_iface_->ik(init_foothold_, angles);
  print_jnt_pos(leg_type_, angles);
  return ((leg_iface_->eef() - init_foothold_).norm() < _epsilon);
}

void SlTest::swing_leg() {
  if (!swing_timer_->running()) {
//    if (0 == traj_type_.compare("quad"))
//      prog_eef_traj_poly(targ_foothold_, eef_traj_);
//    else if (0 == traj_type_.compare("tri"))
//      prog_eef_traj_seg(targ_foothold_, eef_traj_);
//    else if (0 == traj_type_.compare("seg"))
//      prog_eef_traj_seg(targ_foothold_, eef_traj_);
//    else
//      prog_eef_traj_seg(targ_foothold_, eef_traj_);

    prog_eef_traj_poly(goal_foothold_, eef_traj_);
    swing_timer_->start();
  }

  ///! get the current need to reach target of joint position.
  int64_t span = swing_timer_->span();
  double  t = (double)span/params_->SWING_TIME;
//  if (0 == traj_type_.compare("quad"))
//    t = (double)span/params_->SWING_TIME;
//  else if (0 == traj_type_.compare("tri"))
//    t = (double)span/params_->SWING_TIME;
//  else if (0 == traj_type_.compare("seg"))
//    t = span/1000.0;
//  else
//    t = (double)span/params_->SWING_TIME;

  leg_cmd_eef_ = eef_traj_->sample(t);
  leg_iface_->ik(leg_cmd_eef_, leg_cmd_->target);
  ///! Output the trajectory.
  printf("%04ld - %+6.3f %+6.3f %+6.3f\n", span, leg_cmd_->target(JntType::HAA),
      leg_cmd_->target(JntType::HFE), leg_cmd_->target(JntType::KFE));
}

bool SlTest::end_swing_leg() {
  // auto diff = (ctf_eef_[swing_leg_] - leg_ifaces_[swing_leg_]->eef()).norm();
//   auto diff = (eef_traj_->sample(eef_traj_->ceiling()).head(2)
//                  - leg_iface_->eef().head(2)).norm();
  if (nullptr == eef_traj_.get()) return false;

   ///! for rviz
  // LOG_INFO << "The ceiling: " << eef_traj_->ceiling();
  return (/*(LegState::TD_STATE == leg_iface_->leg_state())
       || */((swing_timer_->span() - params_->SWING_TIME) > 1000));
}

void SlTest::ws_calc() {
  print_eef_pos();
}

void SlTest::prod_traj() {
  prog_eef_traj_poly(goal_foothold_, eef_traj_);

  PRESS_THEN_GO
  Eigen::Vector3d fpt;
  Eigen::VectorXd jnts;
  for (double t = 0.00; t < 1.01; t += 0.01) {
    fpt = eef_traj_->sample(t);
    leg_iface_->ik(fpt, jnts);

    printf("%4.2f - ", t);
    __print_color_helper(LegType::FL, jnts);
    printf("\n");
  }
  PRESS_THEN_GO
}

void SlTest::prog_eef_traj_seg(const Eigen::Vector3d& _next_fpt, Traj3dSp& _traj) {
  Eigen::Vector3d _last_fpt = leg_iface_->eef();
  // Eigen::Vector3d _next_fpt = prog_next_fpt(swing_leg_);

  Eigen::Vector3d _rise_fpt = _last_fpt;
  _rise_fpt.z() = _last_fpt.z() + 0.5*params_->SWING_HEIGHT;

  Eigen::Vector3d _appr_fpt = _next_fpt;
  _appr_fpt.z() = _next_fpt.z() + 0.5*params_->SWING_HEIGHT;

  Eigen::Vector3d _top_fpt(
      (_rise_fpt.x()/2 + _appr_fpt.x()/2),
      (_rise_fpt.y()/2 + _appr_fpt.y()/2),
      (_last_fpt.z() + params_->SWING_HEIGHT));

  SegTraj3dSp swing_traj(new SegTraj3d);
  double _t0 = 0;
  // TODO
  double _t1 = _t0 + 0; //params_->SWING_RISE;
  double _t3 = _t1 + 0; //params_->SWING_APPROACH;
  double _t4 = _t3 + 0; //params_->SWING_DROP;
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
  // eef_traj_ = swing_traj;
  _traj = swing_traj;
  ///! Just for debug information
  std::cout << "Swing Trajectory: " << std::endl;
  std::cout << " - Rise phase: " << *rise_traj << std::endl;
  std::cout << " - Appr phase: " << *appr_traj << std::endl;
  std::cout << " - Drop phase: " << *drop_traj << std::endl;
}

void SlTest::prog_eef_traj_poly(const Eigen::Vector3d& _next_fpt, Traj3dSp& _traj) {
  ///////////////////////////////////////////////////////////
  ///! THE POLYNOMIAL CURVE
  ///!  A = a0 + a1 *t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5 + a6 * t^6
  ///!  A(0)   = p0; A'(0) = 0; A''(0) = 0;
  ///!  A(1)   = p1; A'(1) = 0; A''(1) = 0;
  ///!  A(0.5) = 0.5*(p0+p1) + (0, 0, SWING_HEIGHT);
  ///////////////////////////////////////////////////////////
  Eigen::MatrixXd A, b, x;
  Eigen::VectorXd row;
  // Eigen::Vector3d _curr_fpt = leg_iface_->eef();
  Eigen::Vector3d _curr_fpt = init_foothold_;

  A.resize(7, 7);
  b.resize(7, 3);
  ///! A(0)   = p0
  A.row(0)    = __get_state_vec<double>(0, 7);
  b.row(0)    = _curr_fpt;
  ///! A'(0)  = 0
  A.row(1)    = __get_diff_vec<double>(0, 7);
  b.row(1)    << 0, 0, 0;
  ///! A''(0) = 0
  Eigen::VectorXd _ddif_7th_coef;
  _ddif_7th_coef.resize(5);
  _ddif_7th_coef << (2*1), (3*2), (4*3), (5*4), (6*5);
  row         = __get_state_vec<double>(0, 5);
  A.row(2).head(2).fill(0.0); A.row(2).tail(5) = _ddif_7th_coef.cwiseProduct(row);
  b.row(2)    << 0, 0, 0;
  ///! A(0.5) = 0.5*(p0+p1) + (0, 0, SWING_HEIGHT);
  A.row(3) = __get_state_vec<double>(0.5, 7);
  b.row(3) = 0.5*(_curr_fpt + _next_fpt);
  b.row(3).z() += params_->SWING_HEIGHT;
  ///! A(1) = p1
  A.row(4) = __get_state_vec<double>(1, 7);
  b.row(4) = _next_fpt;
  ///! A'(1) = 0
  A.row(5) = __get_diff_vec<double>(1, 7);
  b.row(5) << 0, 0, 0;
  ///! A''(1) = 0
  row      = __get_state_vec<double>(1, 5);
  A.row(6).head(2).fill(0.0); A.row(6).tail(5) = _ddif_7th_coef.cwiseProduct(row);
  b.row(6) << 0, 0, 0;

//  std::cout << "poly - A:\n" << A << std::endl;
//  std::cout << "appr - b:\n" << b << std::endl;
  if (0 == A.determinant()) {
    x = A.householderQr().solve(b);
    LOG_WARNING << "NO trajectory results, Using this result: " << x.transpose();
  } else {
    x = A.partialPivLu().solve(b);
  }

  PolyTraj3d* traj = new PolyTraj3d(x.transpose());
  _traj.reset(traj);
  _traj->range(0, 1);

  ///! Just for debug information
  LOG_INFO << "Swing Trajectory: ";
  LOG_INFO << " - Rise phase: " << *traj;
}

//void SlTest::prog_eef_traj_tri(const Eigen::Vector3d& _next_fpt, Traj3dSp& _traj) {
//  ///////////////////////////////////////////////////////////
//  ///! THE TRIANGLE CURVE
//  ///! CURVE = RISE_PAHSE + DROP_PAHSE
//  ///////////////////////////////////////////////////////////
//  Eigen::MatrixXd A, b, x;
//  Eigen::VectorXd row;
//  Eigen::Vector3d _curr_fpt = leg_iface_->eef();
//
//  ///////////////////////////////////////////////////////////
//  ///! THE RISE PAHSE
//  ///!  A = a0 + a1 *t + a2 * t^2 + a3 * t^3 + a4 * t^4 + a5 * t^5 + a6 * t^6
//  ///!  A(0)   = p0; A'(0) = 0; A''(0) = 0;
//  ///!  A(0.5) = p1; A'(1) = 0; A''(1) = 0;
//  ///!  A(0.5) = 0.5*(p0+p1) + (0, 0, SWING_HEIGHT)
//  ///////////////////////////////////////////////////////////
//  A.resize(7, 7);
//  b.resize(7, 3);
//  ///! A(0)   = p0
//  A.row(0)    = __get_state_vec<double>(0, 7);
//  b.row(0)    = _curr_fpt;
//  ///! A'(0)  = 0
//  A.row(1)    = __get_diff_vec<double>(0, 7);
//  b.row(1)    << 0, 0, 0;
//  ///! A''(0) = 0
//  Eigen::VectorXd _ddif_7th_coef;
//  _ddif_7th_coef.resize(5);
//  _ddif_7th_coef << (2*1), (3*2), (4*3), (5*4), (6*5);
//  row         = __get_state_vec<double>(0, 5);
//  A.row(2).head(2).fill(0.0); A.row(2).tail(5) = _ddif_7th_coef.cwiseProduct(row);
//  b.row(2)    << 0, 0, 0;
//  ///! A(0.5) = 0.5*(p0+p1) + (0, 0, SWING_HEIGHT)
//  A.row(3) = __get_state_vec<double>(0.5, 7);
//  b.row(3) = 0.5*(_curr_fpt + _next_fpt);
//  b.row(3).z() += params_->SWING_HEIGHT;
//  ///! A(1) = p1
//  A.row(4) = __get_state_vec<double>(1, 7);
//  b.row(4) = _next_fpt;
//  ///! A'(1) = 0
//  A.row(5) = __get_diff_vec<double>(1, 7);
//  b.row(5) << 0, 0, 0;
//  ///! A''(1) = 0
//  row      = __get_state_vec<double>(1, 5);
//  A.row(6).head(2).fill(0.0); A.row(6).tail(5) = _ddif_7th_coef.cwiseProduct(row);
//  b.row(6) << 0, 0, 0;
//
////  std::cout << "appr - A:\n" << A << std::endl;
////  std::cout << "appr - b:\n" << b << std::endl;
//  if (0 == A.determinant()) {
//    x = A.householderQr().solve(b);
//    std::cout << "NO trajectory results, Using this result: " << x.transpose() << std::endl;
//  } else {
//    x = A.partialPivLu().solve(b);
//  }
//
//  _traj.reset(new PolyTraj3d(x.transpose()));
//  _traj->range(0, 1);
//}

void SlTest::error_estimate() {
  TIMER_INIT

  while (thread_alive_) {
    ;


    TIMER_CONTROL(1)
  }
}

} /* namespace gaile_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_control::SlTest, Label)
