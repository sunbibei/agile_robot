/*
 * ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_QR_ROS_WRAPPER_H_
#define INCLUDE_QR_ROS_WRAPPER_H_

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

class RosWrapper : public MiiRobot, public MiiControl {
SINGLETON_DECLARE(RosWrapper, const std::string&, const std::string&)

protected:
  virtual bool init() override;
  virtual void create_system_instance() override;

private:
  ///! The helper methods.
  void sys_inst_robot(const std::string&);
  void sys_inst_control(const std::string&);

  ///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
  void publishRTMsg();
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
  std::string     robot_root_;
  std::string     control_root_;

  bool            alive_;
  ros::Subscriber gait_ctrl_sub_;

  // About ROS control
  std::chrono::milliseconds  rt_duration_;
};

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
