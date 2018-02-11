/*
 * qr_body.cpp
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#include <robot/body/qr_body.h>

namespace qr_control {

QrBody::QrBody() {
  // TODO Auto-generated constructor stub

}

QrBody::~QrBody() {
  // TODO Auto-generated destructor stub
}

///! The pose of robot against the world frame
void QrBody::pose(Eigen::Vector3d&, Eigen::Quaterniond&)  {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
}

///! The translation of robot against the world frame
Eigen::Vector3d QrBody::translation() {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

///! The rotation of robot against the world frame
Eigen::Quaterniond QrBody::rotation() {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
}

///! The velocity of robot against the world frame
Eigen::Vector3d QrBody::velocity() {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

///! The centre of gravity of robot
Eigen::Vector3d QrBody::cog() {
  // LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Vector3d(3.0, 0.0, 0.0);
}

///! The translation of LegType shoulder(leg_base frame) of robot against the world frame.
Eigen::Vector3d QrBody::leg_base(LegType leg) {
  switch (leg) {
  case LegType::FL: return Eigen::Vector3d( body_length(),  body_width(), 0);
  case LegType::FR: return Eigen::Vector3d( body_length(), -body_width(), 0);
  case LegType::HL: return Eigen::Vector3d(-body_length(),  body_width(), 0);
  case LegType::HR: return Eigen::Vector3d(-body_length(), -body_width(), 0);
  default:
    LOG_ERROR << "ERROR LegType";
    return Eigen::Vector3d(0, 0, 0);
    break;
  }
}

} /* namespace qr_control */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(qr_control::QrBody, Label)
