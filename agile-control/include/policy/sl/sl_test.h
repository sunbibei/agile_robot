/*
 * sl_test.h
 *
 *  Created on: Mar 10, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_SL_TEST_H_
#define INCLUDE_GAIT_SL_TEST_H_

#include "policy/policy.h"
#include "adt/trajectory.h"

#include <Eigen/Dense>

///! Forward declaration
class TimeControl;

namespace agile_control {

enum LTestState {
  UNKNOWN_LT_STATE = -1,
  LT_INIT_POSE = 0,
  LT_SWING,
  LT_KICK,
  LT_WS_CALC,
  LT_PROD_TRAJ,
  N_LT_STATE,
};

class SlTest: public Policy {
public:
  SlTest();
  virtual bool auto_init() override;

  virtual ~SlTest();

///! These methods are inherited from super class.
public:
  ///! The flow of state convert
  virtual void checkState() override;
  ///! call it before the state callback
  // virtual void prev_tick()  override;
  ///! call it after the state callback
  virtual void post_tick()  override;

  ///! starting the gait
  virtual bool starting()  override;
  ///! stopping the gait
  virtual void stopping()  override;

///! These methods are the callback methods for WalkState.
private:
  ///! The callback for WK_INIT_POSE
  void pose_init();
  ///! The callback for WK_SWING
  void swing_leg();
  ///! The callback for LT_KICK
  void kick();
  ///! The callback for LT_WS_CALC
  void ws_calc();
  ///! The callback for LT_PROD_TRAJ
  void prod_traj();

///! The helper for move COG and swing leg.
private:
  /*!
   * @brief The criterion for WK_INIT_POS
   */
  bool end_pose_init();
  /*!
   * @brief The criterion for WK_SWING
   */
  bool end_swing_leg();
  /*!
   * @brief Update the next foot point for the swing leg.
   *        The default next foot point is {STEP, XX, -BH},
   *        namely the swing leg move forward STEP cm. The
   *        trajectory is quadratic curve.
   */
  void prog_eef_traj_seg(const Eigen::Vector3d&,  Traj3dSp&);

  void prog_eef_traj_poly(const Eigen::Vector3d&, Traj3dSp&);

  // void prog_eef_traj_tri(const Eigen::Vector3d&,  Traj3dSp&);

protected:
  LTestState                current_state_;
  ///! The interface for leg.
  LegType                   leg_type_;
  class RobotLeg*           leg_iface_;
  ///! The commands of legs.
  class LegTarget*          leg_cmd_;
  ///! The end-effector command.
  Eigen::Vector3d           leg_cmd_eef_;
  ///! The initialize position
  Eigen::Vector3d           init_foothold_;
  ///! The position of foothold.
  Eigen::Vector3d           goal_foothold_;
  ///! The swing timer
  TimeControl*              swing_timer_;
  ///! The trajectory for swing leg
  Traj3dSp                  eef_traj_;
  ///! parameters for workspaces calculate.
  class WSParam*            ws_params_;
  ///! parameters for gait control
  class LTParam*            params_;
  ///! which type of trajectory
  std::string               traj_type_;

  ///! save the data into file.
  bool            is_save_enable_;
  int64_t         save_episode_;
  int64_t         wait_episode_;
  std::string     save_path_;
  ///! pos(3-dim), cmd(3-dim)
  Eigen::MatrixXd save_data_;

///! The estimate function along with the main process.
private:
  void error_estimate();

  bool thread_alive_;
};

} /* namespace agile_control */

#endif /* INCLUDE_GAIT_SL_TEST_H_ */
