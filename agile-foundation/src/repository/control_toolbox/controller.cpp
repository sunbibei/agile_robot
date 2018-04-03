/*
 * control_toolboxcontroller.cpp
 *
 *  Created on: Dec 21, 2017
 *      Author: bibei
 */

#include "repository/control_toolbox/controller.h"

Controller::Controller(const std::string& _l)
  : Label(_l) {
  // TODO Auto-generated constructor stub

}

bool Controller::auto_init() {
  ;
}

Controller::~Controller() {
  // TODO Auto-generated destructor stub
}

void Controller::control(const Eigen::VectorXd& _x, Eigen::VectorXd& _u) {
  ;
}
