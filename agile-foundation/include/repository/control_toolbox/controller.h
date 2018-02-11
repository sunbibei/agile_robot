/*
 * controller.h
 *
 *  Created on: Dec 21, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_MII_FOUNDATION_REPOSITORY_CONTROL_TOOLBOX_CONTROLLER_H_
#define INCLUDE_MII_FOUNDATION_REPOSITORY_CONTROL_TOOLBOX_CONTROLLER_H_

#include "foundation/label.h"

#include <Eigen/Dense>

class Controller : public Label {
public:
  Controller(const MiiString& _l = "controller");
  virtual bool auto_init() override;

  virtual ~Controller();

public:
  /*!
   * @brief Calculate the action at given state.
   */
  virtual void control(const Eigen::VectorXd& _x, Eigen::VectorXd& _u);
};

#endif /* INCLUDE_MII_FOUNDATION_REPOSITORY_CONTROL_TOOLBOX_CONTROLLER_H_ */
