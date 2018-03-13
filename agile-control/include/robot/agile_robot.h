/*
 * qr_robot.h
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_AGILE_ROBOT_H_
#define INCLUDE_ROBOT_AGILE_ROBOT_H_

// #include <foundation/utf.h>

#include <robot/leg_robot.h>

namespace agile_control {

class AgileRobot: public LegRobot {
SINGLETON_DECLARE(AgileRobot)

///! inherit from LegRobot
public:
  virtual double stability_margin() const override;
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_AGILE_ROBOT_H_ */
