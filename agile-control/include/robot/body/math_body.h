/*
 * math_body.h
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_MATH_BODY_H_
#define INCLUDE_ROBOT_BODY_MATH_BODY_H_

#include <robot/body/data_body.h>

namespace agile_control {

class MathBody: public DataBody {
///! These are the
public:
  ///! The pose of robot against the world frame
  virtual void pose(Eigen::Vector3d&, Eigen::Quaterniond&) = 0;
  ///! The translation of robot against the world frame
  virtual Eigen::Vector3d    translation()      = 0;
  ///! The rotation of robot against the world frame
  virtual Eigen::Quaterniond rotation()         = 0;
  ///! The velocity of robot against the world frame
  virtual Eigen::Vector3d    velocity()         = 0;
  ///! The centre of gravity of robot
  virtual Eigen::Vector3d    cog()              = 0;
  ///! The translation of LegType shoulder(leg_base frame) of robot against the world frame.
  virtual Eigen::Vector3d    leg_base(LegType)  = 0;

///! The convenient interface for methods override.
public:
  void leg_base(LegType _l, Eigen::Vector3d& _xyz) { _xyz = leg_base(_l); }
  ///! The translation of robot against the world frame
  void translation(Eigen::Vector3d& _xyz) { _xyz = translation(); }
  ///! The rotation of robot against the world frame
  void rotation(Eigen::Quaterniond& _rpy) { _rpy = rotation(); }
  ///! The velocity of robot against the world frame
  void velocity(Eigen::Vector3d& v)  { v = velocity(); }
  ///! The centre of gravity of robot
  void cog(Eigen::Vector3d& _cog)     { _cog = cog(); };
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_MATH_BODY_H_ */
