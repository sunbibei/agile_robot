/*
 * qr_robot.h
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_QR_ROBOT_H_
#define INCLUDE_ROBOT_QR_ROBOT_H_

// #include <foundation/utf.h>

#include <robot/leg_robot.h>

namespace qr_control {

class QrRobot: public LegRobot {
SINGLETON_DECLARE(QrRobot)

///! inherit from LegRobot
public:
  virtual double stability_margin() const override;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_QR_ROBOT_H_ */
