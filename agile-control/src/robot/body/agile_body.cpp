/*
 * qr_body.cpp
 *
 *  Created on: Dec 28, 2017
 *      Author: bibei
 */

#include <robot/body/agile_body.h>

namespace agile_control {

AgileBody::AgileBody() {
  // TODO Auto-generated constructor stub

}

AgileBody::~AgileBody() {
  // TODO Auto-generated destructor stub
}

///! The pose of robot against the world frame
void AgileBody::pose(Eigen::Vector3d&, Eigen::Quaterniond&)  {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
}

///! The translation of robot against the world frame
Eigen::Vector3d AgileBody::translation() {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

///! The rotation of robot against the world frame
Eigen::Quaterniond AgileBody::rotation() {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
}

///! The velocity of robot against the world frame
Eigen::Vector3d AgileBody::velocity() {
  LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Vector3d(0.0, 0.0, 0.0);
}

///! The centre of gravity of robot
Eigen::Vector3d AgileBody::cog() {
  // LOG_ERROR << "Call the 'translation' which has does not complemented.";
  return Eigen::Vector3d(3.0, 0.0, 0.0);
}

///! The translation of LegType shoulder(leg_base frame) of robot against the world frame.
Eigen::Vector3d AgileBody::leg_base(LegType leg) {
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

#include <class_loader/register_macro.hpp>
// #include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_control::AgileBody, Label)
