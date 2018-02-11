/*
 * creep.h
 *
 *  Created on: Nov 21, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_CREEP_CREEP_H_
#define INCLUDE_GAIT_CREEP_CREEP_H_

#include "gait/gait_base.h"
#include "adt/trajectory.h"

#include <Eigen/Dense>

#define TEST_TIME
#define CONTINUE_GAIT

///! Forward declaration
class TimeControl;

namespace qr_control {

enum CreepState {
  UNKNOWN_CP_STATE = -1,
  CP_INIT_POSE = 0,
  CP_READY,
  CP_SWING_FRONT,
  CP_SWING_HIND,
  N_CP_STATE
};

class Creep: public GaitBase {
public:
  Creep(const MiiString& _n = "creep");
  virtual bool auto_init() override;

  virtual ~Creep();

///! These methods are inherited from super class.
public:
  virtual void checkState() override;
  ///! call it after the state callback
  virtual void post_tick() override;

  virtual bool starting()   override;
  virtual void stopping()   override;

protected:
  ///! The state enumeration for creep.
  CreepState                current_state_;

///! These methods are the callback methods for WalkState.
protected:
  ///! The callback for CP_INIT_POSE
  void pose_init();
  ///! The callback for CP_READY
  void ready();
  ///! The callback for CP_SWING_FRONT
  void swing_front();
  ///! The callback for CP_SWING_HIND
  void swing_hind();
  /*!
   * @brief The criterion for CP_INIT_POS
   */
  bool end_pose_init();
  /*!
   * @brief The criterion for CP_READY
   */
  bool end_ready();
  /*!
   * @brief The criterion for CP_SWING_XXX
   */
  bool end_walk();
protected:
  /*!
   * @brief Choice the next swing leg @next by @curr LegType.
   *        The flow of swing leg is designed by this method.
   *        The order as follow:
   *        HL -> FL -> HR -> FR
   *
   *        **NOTE**: This method ONLY be called after the end of swing leg.
   */
  LegType next_leg(const LegType);
  /*!
   * @brief print the stability margin
   */
  Eigen::Vector3d stability_margin(LegType sl);
  /*!
   * @brief Update the next foot point for the swing leg.
   *        The default next foot point is {STEP, XX, -BH},
   *        namely the swing leg move forward STEP cm. The
   *        trajectory is quadratic curve.
   */
  void prog_eef_traj(const Eigen::Vector3d& _next_fpt);
  /*!
   * @brief Programming the trajectory of COG, and convert to the trajectory
   *        of end-effector. This trajectory is linearity curve.
   */
  void prog_cog_traj(const Eigen::Vector2d& _next_cog);
protected:
  class CreepParams*       cp_params_;

  ///! The interface for body
  class RobotBody*      body_iface_;
  ///! The interface for legs
  class RobotLeg*       leg_ifaces_[LegType::N_LEGS];
  ///! The end-effector command.
  Eigen::Vector3d       eef_cmds_[LegType::N_LEGS];
  ///! The next foothold.
  Eigen::Vector4d       balance_fpts_;

  ///! The time control. this TimeControl will be started in the begin of
  ///! each state, stopped in the end of each state.
  TimeControl*     timer_;
  ///! The time control. this TimeControl is used to decide when to swing the leg.
  TimeControl*     swing_timer_;
  ///! The time control. this TimeControl is used to decide when to move the cog.
  TimeControl*     cog_timer_;
  ///! The trajectory for swing leg
  Traj3dSp         eef_traj_;
  ///! The trajectory for moving COG
  Traj3dSp         cog2eef_traj_[LegType::N_LEGS];
  ///! The following swing leg, it indicates the following swing leg at any time.
  LegType          swing_leg_;
  ///! The Control tick interval for send command(in ms)
  int64_t          post_tick_interval_;

  ///! Whether is hang
  bool is_hang_;

#ifdef TEST_TIME
  MiiVector<int64_t> t_time_;
  double             t_last_stance_pos_;
  double             t_last_swing_pos_;
  MiiVector<double>  t_translation_;
#endif
};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_CREEP_CREEP_H_ */
