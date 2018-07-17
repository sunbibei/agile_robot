/*
 * control_wrapper.h
 *
 *  Created on: Jul 10, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_APPS_CONTROL_WRAPPER_H_
#define INCLUDE_APPS_CONTROL_WRAPPER_H_

#include "mii_control.h"

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#define DEBUG_TOPIC

using namespace agile_control;

class ControlWrapper: public MiiControl {
  SINGLETON_DECLARE(ControlWrapper, const std::string&)

protected:
  virtual bool init() override;
  virtual void create_system_singleton() override;

  /*!
   * @brief The callback for gait_topic.
   */
  void gaitControlCb(const std_msgs::String::ConstPtr&);

  // Just for test
#ifdef DEBUG_TOPIC
  void cbForDebug(const std_msgs::Float32ConstPtr&);
  ros::Subscriber cmd_sub_;
#endif

private:
  // The ROS handle
  ros::NodeHandle nh_;
  std::string     root_tag_;
  ros::Subscriber gait_ctrl_sub_;
  bool alive_;
};

#endif /* INCLUDE_APPS_CONTROL_WRAPPER_H_ */
