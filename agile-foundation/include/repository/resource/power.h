/*
 * power.h
 *
 *  Created on: Nov 9, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_REPOSITORY_RESOURCE_POWER_H_
#define INCLUDE_REPOSITORY_RESOURCE_POWER_H_

#include "foundation/label.h"

namespace agile_robot {

class Power: public Label {
  friend class PowerNode;
public:
  Power(const std::string& _l = Label::null);
  virtual bool auto_init() override;

  virtual ~Power();

public:
  double        current(const LegType&);
  const double& current_const_ref(const LegType&);
  const double* current_const_pointer(const LegType&);

protected:
  void updatePowerInfo(size_t w, double c);

protected:
  class PowerInfor* power_infor_;
  class PowerError* power_error_;
};

} /* namespace middleware */

#endif /* INCLUDE_REPOSITORY_RESOURCE_POWER_H_ */
