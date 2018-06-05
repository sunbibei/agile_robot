/*
 * leg_pid.h
 *
 *  Created on: Nov 3, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_CONTROL_TOOLBOX_LEG_PID_H_
#define INCLUDE_REPOSITORY_CONTROL_TOOLBOX_LEG_PID_H_

#include "foundation/utf.h"

namespace agile_robot {

class LegPid {
public:
  LegPid(const std::string& prefix);
  virtual ~LegPid();
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_CONTROL_TOOLBOX_LEG_PID_H_ */
