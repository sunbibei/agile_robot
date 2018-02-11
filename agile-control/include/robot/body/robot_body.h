/*
 * robot_body.h
 *
 *  Created on: Nov 22, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_BODY_ROBOT_BODY_H_
#define INCLUDE_ROBOT_BODY_ROBOT_BODY_H_

#include "math_body.h"

namespace qr_control {

/*!
 * @brief The body of robot can not operate by itself, so the RobotBody is null.
 *        In order to keep the same hierarchy with Leg, so the RobotBody be reserve, and
 *        may be it will be fill some methods.
 */
class RobotBody: public MathBody {
public:
  virtual bool auto_init() override;
//  virtual ~RobotBody();
//
};

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_BODY_ROBOT_BODY_H_ */
