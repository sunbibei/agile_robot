/*
 * qr_body.h
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_QR_BODY_H_
#define INCLUDE_ROBOT_BODY_QR_BODY_H_

#include "robot_body.h"

namespace qr_control {

class QrBody: public RobotBody {
public:
  QrBody();
  virtual ~QrBody();

///! inherit from MathBody
public:
  ///! The pose of robot against the world frame
  virtual void pose(Eigen::Vector3d&, Eigen::Quaterniond&) override/*= 0*/;
  ///! The translation of robot against the world frame
  virtual Eigen::Vector3d    translation() override /*= 0*/;
  ///! The rotation of robot against the world frame
  virtual Eigen::Quaterniond rotation()    override /*= 0*/;
  ///! The velocity of robot against the world frame
  virtual Eigen::Vector3d    velocity()    override /*= 0*/;
  ///! The centre of gravity of robot
  virtual Eigen::Vector3d    cog()         override /*= 0*/;
  ///! The translation of LegType shoulder(leg_base frame) of robot against the world frame.
  virtual Eigen::Vector3d    leg_base(LegType) override;  /*= 0*/;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_QR_BODY_H_ */
