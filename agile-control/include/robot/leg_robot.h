/*
 * leg_robot.h
 *
 *  Created on: Jan 9, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_ROBOT_LEG_ROBOT_H_
#define INCLUDE_ROBOT_LEG_ROBOT_H_

#include "leg/robot_leg.h"
#include "body/robot_body.h"

#include <Eigen/Dense>

namespace agile_control {
/*!
 * @brief Any sub-class inherit from LegRobot, is need be implemented as a singleton.
 *        This class is similar to a manager of robot's resources, it avoid the fact that each
 *        gait need the same configure to get the interfaces of leg and body from LabelTable,
 *        and it is designed as a high-level information provider, such as stability margin,
 *        the response of emergency, the origin of camera(not now), the odometer, and so on.
 */
class LegRobot {
///! Just for permission leg_robot_inst_
friend class RobotLeg;
friend class RobotBody;

protected:
  LegRobot();
  virtual ~LegRobot();

  static LegRobot* leg_robot_inst_;

public:
  static LegRobot* instance();

public:
	/*!
	 * @brief Calculate the stability margin of the current robot.
	 */
	virtual double stability_margin() const = 0;

///! The getter for the interfaces of robot
public:
	/*!
	 * @brief The interface for robot leg.
	 */
	virtual RobotLeg* robot_leg(LegType);
	/*!
	 * @brief The interface for robot body,
	 */
	virtual RobotBody* robot_body();

protected:
	RobotLeg*  leg_ifaces_[LegType::N_LEGS];
	RobotBody* body_iface_;
};

/*!
 * These are the convenient helper methods for information display.
 */
///! the helpers
void __print_color_helper(LegType);
///! the helpers
void __print_color_helper(LegType, const Eigen::VectorXd&);
///! print joint position of the single leg
void print_jnt_pos(LegType);
///! print the v.s. result and different between joint position of the single leg
void print_jnt_pos(LegType, const Eigen::VectorXd&);
///! print the joint position of the all of leg.
void print_jnt_pos();
///! print the v.s. result and different between joint position of the all of leg.
void print_jnt_pos(const Eigen::VectorXd& fl, const Eigen::VectorXd& fr,
    const Eigen::VectorXd& hl, const Eigen::VectorXd& hr);
///! print FPT(foot point) coordinate of the special leg.
void print_eef_pos(LegType);
///! print the v.s. result and different between FPT or COG
void print_eef_pos(LegType, const Eigen::Vector3d&);
///! print the PFT of the all of leg.
void print_eef_pos();
///! print the v.s. result and different between FPT
void print_eef_pos(const Eigen::Vector3d& fl, const Eigen::Vector3d& fr,
    const Eigen::Vector3d& hl, const Eigen::Vector3d& hr);

} /* namespace qr_control */

#endif /* INCLUDE_ROBOT_LEG_ROBOT_H_ */
