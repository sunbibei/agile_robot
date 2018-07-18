/*
 * ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_APPS_PD_ROS_WRAPPER_H_
#define INCLUDE_APPS_PD_ROS_WRAPPER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <chrono>
#include <boost/scoped_ptr.hpp>

#include "mii_robot.h"
///! qr-next-control
#include "mii_control.h"

#define DEBUG_TOPIC
#ifdef DEBUG_TOPIC
#include <repository/joint.h>
#endif

using namespace agile_robot;

using agile_control::MiiControl;

class PdRosWrapper : public MiiRobot, public MiiControl {
SINGLETON_DECLARE(PdRosWrapper, const std::string&, const std::string&)

protected:
  virtual bool init() override;
  virtual void create_system_singleton() override;

protected:
  /**
   * @brief Support the Registry
   */
  virtual void supportRegistry();

private:
  ///! The helper method for the @supportRegistry()
  void __reg_resource_and_command();
  class __RegRes*   jnt_reg_res_;

private:
  ///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
  void publishRTMsg();
  /*!
   * @brief The callback for gait_topic.
   */
  void cbForControl(const std_msgs::String::ConstPtr&);

  // Just for test
#ifdef DEBUG_TOPIC
  void cbForDebug(const std_msgs::Float32ConstPtr&);
  ros::Subscriber cmd_sub_;
#endif

private:
  // The ROS handle
  ros::NodeHandle nh_;
  // The root of configure file
  std::string     root_robot_;
  std::string     root_control_;

  ///! The resource
  class JointManager* jnt_manager_;

  bool            alive_;
  ros::Subscriber ctrl_sub_;

  std::chrono::microseconds  tick_interval_;
  std::chrono::milliseconds  rt_duration_;
};

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
