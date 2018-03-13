/*
 * robot_leg.h
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_ROBOT_LEG_H_
#define INCLUDE_ROBOT_LEG_ROBOT_LEG_H_

#include <adt/trajectory.h>
#include "robot/leg/math_leg.h"

namespace agile_control {

struct LegTarget {
  JntCmdType       cmd_type;
  Eigen::VectorXd  target;
};

struct JntTarget {
  JntType     jnt_type;
  JntCmdType  jnt_cmd_type;
  double      target;
};

struct EefTarget {
  Eigen::Quaterniond rpy;
  Eigen::Vector3d    xyz;
};

class RobotLeg: public MathLeg {
public:
  RobotLeg();
  virtual bool auto_init() override;
  virtual ~RobotLeg();

///! The status of robot-leg of getter
public:
  // void eef(Eigen::Vector3d& _xyz, Eigen::Quaterniond& _rpy);
  void eef(Eigen::Vector3d&);
  // void eef(Eigen::Quaterniond&);
  Eigen::Vector3d eef();
 
public:
  /*!
   * TODO waiting to implement
   * @brief This method will be return immediately, the process of control is
   *        running in a detach thread.
   * @param remainder The remainder time in ms.
   */
  // virtual void asyncMove(size_t& ret_ref);
  /*!
   * @brief This method will be block until the process of control
   *        has completed.
   */
  virtual void move();

public:
  ///! setter target methods
  /*!
   * @brief set the target for leg, the command for three joints.
   * @param type the type of command
   * @param vals the values of command
   */
  void legTarget             (const LegTarget&);
  void legTarget             (JntCmdType, const Eigen::VectorXd&);
  /*!
   * @brief Set the specially joint target.
   */
  void jointTarget           (const JntTarget&);
  void jointTarget           (JntType, JntCmdType, double);
  /*!
   * @brief Set a trajectory target for a specially joint.
   */
  void jointTrajectoryTarget (JntType, const Traj1dSp&);
  ///! The default order by knee, hip and yaw
  /*!
   * @brief Set a trajectory target for the leg.
   */
  void jointTrajectoryTarget (const Traj3dSp&);
  /*!
   * @brief Set a end-effector pose target.
   */
  // void eefOrientationTarget  (const Eigen::Quaterniond&);
  /*!
   * @brief Set a end-effector position target.
   */
  void eefPositionTarget     (const Eigen::Vector3d&);
  /*!
   * @brief Set a end-effector target.
   */
  void eefTarget             (const EefTarget&);
  // void eefTarget             (const Eigen::Vector3d&, const Eigen::Quaterniond&);
  /*!
   * @brief Set a end-effector trajectory target.
   */
  void eefTrajectoryTarget   (const Traj3dSp&);

  ///! getter target methods
//  const LegTarget&          legTarget             ();
//  const JntTarget&          jointTarget           ();
//  const Trajectory1d&       jointTrajectoryTarget ();
//  const Eigen::Quaterniond& eefOrientationTarget  ();
//  const Eigen::Vector3d&    eefPositionTarget     ();
//  const EefTarget&          eefTarget             ();
//  const Trajectory3d&       eefTrajectoryTarget   ();

///! These are the helper methods
protected:
  /*!
   * @brief The abstract method, is completed the joint trajectory target.
   */
  virtual void followJntTrajectory(JntType, const Traj1dSp&) = 0;
  /*!
   * @brief The abstract method, is completed the leg trajectory target.
   */
  virtual void followJntTrajectory(const Traj3dSp&) = 0;
  /*!
   * @brief The abstract method, is completed the end-effector trajectory target.
   */
  virtual void followEefTrajectory(const Traj3dSp&) = 0;

  virtual void executLeg(const LegTarget&);
  virtual void executJnt(const JntTarget&);
  virtual void executEef(const EefTarget&);
  virtual void executEef(const Eigen::Vector3d&);
  // virtual void executEef(const Eigen::Quaterniond&);


///! The targets for each spaces.
protected:
  LegTarget          leg_target_;
  JntTarget          jnt_target_;

  Traj1dSp  jnt_traj_target_;
  Traj3dSp  jnts_traj_target_;

  EefTarget eef_target_;
  Traj3dSp  eef_traj_target_;

private:
  enum TargetType {
    INVALID_TARGET = -1,
    LEG_CMD,
    JNT_CMD,
    JNT_TRAJ,
    EEF_XYZ,
    EEF_RPY,
    EEF_POSE,
    EEF_TRAJ,
    N_TARGET_TYPE
  };

  TargetType curr_target_;
  JntType    curr_target_jnt_for_traj_;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_ROBOT_LEG_H_ */
