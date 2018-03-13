/*
 * walk.h
 *
 *  Created on: Dec 27, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_GAIT_WALK_WALK_H_
#define INCLUDE_GAIT_WALK_WALK_H_

#include "gait/gait_base.h"
#include "adt/trajectory.h"

#include <Eigen/Dense>

///! whether the colored output the joint position.
#define DIS_JNT_LIMIT
///! whether record the EEF trajectory
// #define RECORDER_EEF_TRAJ
#ifdef  RECORDER_EEF_TRAJ
#include "adt/discrete.h"
#endif

#define PUB_ROS_TOPIC
#ifdef PUB_ROS_TOPIC
#include <ros/ros.h>
#include <boost/scoped_ptr.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#endif

///! Forward declaration
class TimeControl;

namespace agile_control {

enum WalkState {
  UNKNOWN_WK_STATE = -1,
  WK_INIT_POSE = 0,
  WK_MOVE_COG,
  WK_SWING,
  WK_SWING_1,
  WK_STOP,
  WK_WALK_INIT,
  WK_WALK,
  WK_SPIRALLING,
  WK_HANG,
  N_WK_STATE,
};

class Walk: public GaitBase {
public:
  Walk();
  virtual bool auto_init() override;

  virtual ~Walk();

///! These methods are inherited from super class.
public:
  ///! The flow of state convert
  virtual void              checkState()    override;
  // virtual StateMachineBase* state_machine() override;

  ///! call it before the state callback
  // virtual void              prev_tick() override;
  ///! call it after the state callback
  virtual void              post_tick() override;

  ///! starting the gait
  virtual bool              starting()  override;
  ///! stopping the gait
  virtual void              stopping()  override;

protected:
  WalkState                  current_state_;
  ///! The state machine corresponds with CreepState for creep
  // StateMachine<WalkState>*   state_machine_;

///! These methods are the callback methods for WalkState.
private:
  ///! The callback for WK_INIT_POSE
  void pose_init();
  ///! The callback for WK_MOVE_COG
  void move_cog();
  ///! The callback for WK_SWING
  void swing_leg();
  ///! The callback for WK_STOP
  void stance();
  ///! The callback for WK_SPIRALLING
  void spiralling();
  ///! The callback for WK_XXX
  void walk();
  ///! The debug callback for WK_HANG
  void hang_walk();

///! The helper for move COG and swing leg.
private:
  /*!
   * @brief The criterion for WK_INIT_POS
   */
  bool end_pose_init();
  /*!
   * @brief The criterion for WK_MOVE_COG
   */
  bool end_move_cog();
  /*!
   * @brief The criterion for WK_SWING
   */
  bool end_swing_leg();
  /*!
   * @brief The criterion for WK_STOP
   */
  bool end_stance();
  /*!
   * @brief The criterion for WK_WALK
   */
  bool end_walk();
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
private:
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
   * @brief It get a reference of next COG coordination when given the following
   *        swing leg.
   */
  Eigen::Vector2d prog_next_cog(LegType _fsl);
  /*!
   * @brief It get a reference of next foothold coordination under give the
   *        following swing leg.
   */
  Eigen::Vector3d prog_next_fpt(LegType _fsl);
  /*!
   * @brief It control the phase of that the foot is close to stand.
   */
  void close_to_floor();
  /*!
   * @brief print the stability margin
   */
  Eigen::Vector3d stability_margin(LegType sl);
protected:
  ///! The interface for body
  class RobotBody*      body_iface_;
  ///! The interface for legs
  class RobotLeg*       leg_ifaces_[LegType::N_LEGS];
  ///! The commands of legs.
  class LegTarget*      leg_cmds_[LegType::N_LEGS];
  ///! The end-effector command.
  Eigen::Vector3d       leg_cmd_eefs_[LegType::N_LEGS];
  ///! parameters for gait control
  class WalkParam*      wk_params_;
  ///! parameters for touchdown control
  class TdParam*        td_params_;

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
  ///! The stance modify deltas
  double           stance_deltas_[LegType::N_LEGS];
  ///! The following swing leg, it indicates the following swing leg at any time.
  LegType          swing_leg_;
  ///! The Control tick interval for send command(in ms)
  int64_t          post_tick_interval_;

#ifdef PUB_ROS_TOPIC
  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<
    std_msgs::Float64MultiArray>> cmd_pub_;
#endif

///! These variables and methods are temporary.
private:
  ///! save the last close_to_floor eef target
  Eigen::Vector3d ctf_eef_[LegType::N_LEGS];
#ifdef RECORDER_EEF_TRAJ
  ///! record the EEF trajectory.
  Discrete<double, 14> eefs_traj_recorder_;
#endif
//  Eigen::Vector2d cog_proj1();
//  ///! The last position of swing leg
//  Eigen::Vector3d last_foot_pos1_;
//  ///! The target position of swing leg
//  Eigen::Vector3d next_foot_pos1_;
  ///! Whether is hang?
  // bool            is_hang_walk_;
//  Eigen::Vector2d stance_velocity(const Eigen::Vector2d&, int);
//  Eigen::Vector2d stance_velocity(const Eigen::Vector2d&, int64_t);
  // Eigen::Vector2d inner_triangle(const Eigen::Vector2d&, const Eigen::Vector2d&, const Eigen::Vector2d&);

};

} /* namespace qr_control */

#endif /* INCLUDE_GAIT_WALK_WALK_H_ */
