/*
 * qr_robot.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#include <robot/agile_robot.h>

namespace agile_control {

SINGLETON_IMPL(AgileRobot)

AgileRobot::AgileRobot()
  : LegRobot() {
  ; // Nothing to do here.
}

AgileRobot::~AgileRobot() {
  ; // Nothing to do here.
}

double AgileRobot::stability_margin() const {
  // TODO
  return 10000.0;
}

} /* namespace qr_control */

//#include <class_loader/class_loader_register_macro.h>
//CLASS_LOADER_REGISTER_CLASS(qr_control::QrRobot, Label)
