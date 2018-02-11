/*
 * qr_robot.cpp
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#include <robot/qr_robot.h>

namespace qr_control {

SINGLETON_IMPL(QrRobot)

QrRobot::QrRobot()
  : LegRobot() {
  ; // Nothing to do here.
}

QrRobot::~QrRobot() {
  ; // Nothing to do here.
}

double QrRobot::stability_margin() const {
  // TODO
  return 10000.0;
}

} /* namespace qr_control */

//#include <class_loader/class_loader_register_macro.h>
//CLASS_LOADER_REGISTER_CLASS(qr_control::QrRobot, Label)
