/*
 * robot_leg.cpp
 *
 *  Created on: Nov 23, 2017
 *      Author: bibei
 */

#include "robot/leg/robot_leg.h"
#include "robot/leg_robot.h"

///! Just test
// class Trajectory { };

namespace qr_control {

RobotLeg::RobotLeg()
  : MathLeg(),
    curr_target_(TargetType::INVALID_TARGET) {
  curr_target_jnt_for_traj_ = JntType::UNKNOWN_JNT;
}

bool RobotLeg::auto_init() {
  if (!MathLeg::auto_init()) return false;

  LegRobot::instance()->leg_ifaces_[leg_type()] = this;
  return true;
}

RobotLeg::~RobotLeg() {
}
/*
void RobotLeg::asyncMove(bool& ret_ref) {
  ;
}*/

void RobotLeg::move() {
  switch (curr_target_) {
  case TargetType::LEG_CMD:
    executLeg(leg_target_);
    break;
  case TargetType::JNT_CMD:
    executJnt(jnt_target_);
    break;
  case TargetType::JNT_TRAJ:
    // Using the N_JNTS represent the all of joints.
    if (JntType::N_JNTS != curr_target_jnt_for_traj_)
      followJntTrajectory(curr_target_jnt_for_traj_, jnt_traj_target_);
    else
      followJntTrajectory(jnts_traj_target_);
    break;
  case TargetType::EEF_XYZ:
    executEef(eef_target_.xyz);
    break;
//  case TargetType::EEF_RPY:
//    executEef(eef_target_.rpy);
//    break;
  case TargetType::EEF_POSE:
    executEef(eef_target_);
    break;
  case TargetType::EEF_TRAJ:
    followEefTrajectory(eef_traj_target_);
    break;
  default:
    break;
  }
}

void RobotLeg::executLeg(const LegTarget& p) {
  if ((JntCmdType::CMD_POS == p.cmd_type)
      || (JntCmdType::CMD_MOTOR_VEL == p.cmd_type)) {
    joint_mode(p.cmd_type);
    joint_command(p.target);
  } else {
    LOG_ERROR << "Only Support Position and Motor Velocity Mode!";
  }
}

void RobotLeg::executJnt(const JntTarget& p) {
  if ((JntCmdType::CMD_POS == p.jnt_cmd_type)
      || (JntCmdType::CMD_MOTOR_VEL == p.jnt_cmd_type)) {
    joint_mode(p.jnt_cmd_type);
    joint_command(p.jnt_type, p.target);
  } else {
    LOG_ERROR << "Only Support Position and Motor Velocity Mode!";
  }
}

void RobotLeg::executEef(const EefTarget& p) {
  EVX _jnt_pos;
  if (TargetType::EEF_XYZ == curr_target_) {
    ik(p.xyz, _jnt_pos);
//  } else if (TargetType::EEF_RPY  == curr_target_) {
//    ik(p.rpy, _jnt_pos);;
//  } else if (TargetType::EEF_POSE == curr_target_) {
//    ik(p.xyz, p.rpy, _jnt_pos);;
  } else {
    LOG_ERROR << "What fucking code!";
  }

  joint_command(_jnt_pos);
}

void RobotLeg::executEef(const Eigen::Vector3d& xyz) {
  EVX _jnt_pos;
  ik(xyz, _jnt_pos);

  joint_command(_jnt_pos);
  if (false/* && LegType::FL == leg_type()*/)
    printf("%s - %+8.5f, %+8.5f, %+8.5f\n", LEGTYPE_TOSTRING(leg_type()),
        _jnt_pos[JntType::KNEE], _jnt_pos[JntType::HIP], _jnt_pos[JntType::YAW]);
}

//void RobotLeg::executEef(const Eigen::Quaterniond& rpy) {
//  EVX _jnt_pos;
//  ik(rpy, _jnt_pos);
//
//  joint_command(_jnt_pos);
//}

///! setter methods
void RobotLeg::legTarget(const LegTarget& t) {
  leg_target_  = t;
  curr_target_ = TargetType::LEG_CMD;
}

void RobotLeg::legTarget(JntCmdType jnt_cmd_type, const Eigen::VectorXd& target) {
  leg_target_.cmd_type = jnt_cmd_type;
  leg_target_.target   = target;
  curr_target_         = TargetType::LEG_CMD;
}

void RobotLeg::jointTarget(const JntTarget& t) {
  jnt_target_  = t;
  curr_target_ = TargetType::JNT_CMD;
}

void RobotLeg::jointTarget(JntType jnt_type, JntCmdType jnt_cmd_type, double target) {
  jnt_target_.jnt_cmd_type = jnt_cmd_type;
  jnt_target_.jnt_type     = jnt_type;
  jnt_target_.target       = target;
  curr_target_             = TargetType::JNT_CMD;
}

void RobotLeg::jointTrajectoryTarget(JntType _t, const Traj1dSp& _traj) {
  curr_target_jnt_for_traj_ = _t;
  jnt_traj_target_          = _traj;
  curr_target_              = TargetType::JNT_TRAJ;
}

void RobotLeg::jointTrajectoryTarget(const Traj3dSp& _traj) {
  curr_target_jnt_for_traj_  = JntType::N_JNTS;
  jnts_traj_target_          = _traj;
  curr_target_               = TargetType::JNT_TRAJ;
}

//void RobotLeg::eefOrientationTarget(const Eigen::Quaterniond& t) {
//  eef_target_.rpy = t;
//  curr_target_    = TargetType::EEF_RPY;
//}

void RobotLeg::eefPositionTarget(const Eigen::Vector3d& t) {
  eef_target_.xyz = t;
  curr_target_    = TargetType::EEF_XYZ;
}

void RobotLeg::eefTarget(const EefTarget& t) {
  eef_target_  = t;
  curr_target_ = TargetType::EEF_POSE;
}

void RobotLeg::eefTrajectoryTarget(const Traj3dSp& t) {
  eef_traj_target_ = t;
  curr_target_     = TargetType::EEF_TRAJ;
}

///! getter methods
//const JntTarget& RobotLeg::jointTarget() {
//  return jnt_target_;
//}
//
//const Trajectory1d&   RobotLeg::jointTrajectoryTarget() {
//  return (*jnt_traj_target_);
//}
//
//const Eigen::Quaterniond& RobotLeg::eefOrientationTarget() {
//  return eef_target_.rpy;
//}
//
//const EV3&    RobotLeg::eefPositionTarget() {
//  return eef_target_.xyz;
//}
//
//const Trajectory3d&   RobotLeg::eefTrajectoryTarget() {
//  return *eef_traj_target_;
//}

//void RobotLeg:: eef(Eigen::Vector3d& _xyz, Eigen::Quaterniond& _rpy) {
//  fk(_xyz, _rpy);
//}

void RobotLeg::eef(Eigen::Vector3d& _xyz) {
  fk(_xyz);
}

//void RobotLeg::eef(Eigen::Quaterniond& _rpy) {
//  fk(_rpy);
//}

Eigen::Vector3d RobotLeg::eef() {
  Eigen::Vector3d _xyz;
  fk(_xyz);
  return _xyz;
}

} /* namespace qr_control */
