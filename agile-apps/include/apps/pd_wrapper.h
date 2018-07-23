/*
 * ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_APPS_PD_WRAPPER_H_
#define INCLUDE_APPS_PD_WRAPPER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <chrono>

#include "mii_robot.h"

#define DEBUG_TOPIC

///! Forward declaration
namespace agile_robot {
  class Joint;
}

using agile_robot::Joint;

class PdWrapper : public agile_robot::MiiRobot {
SINGLETON_DECLARE(PdWrapper)

protected:
  virtual bool init() override;
  virtual void create_system_singleton() override;

private:
  ///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
  void pub_rt_msg();

  ///! TODO The thread of control.
  void controlRobot();

  // Just for test
#ifdef DEBUG_TOPIC
  void cbForDebug(const std_msgs::Float32ConstPtr&);
  ros::Subscriber cmd_sub_;
#endif
  void cbForControl(const std_msgs::StringConstPtr&);

private:
  // The ROS handle
  ros::NodeHandle nh_;
  // The namespace of configures
  std::string     param_ns_;
  // The root of configure file
  std::string     root_wrapper_;

  ///! The resource of joints.
  MiiPtr<Joint>   jnt_lft_leg_;
  MiiPtr<Joint>   jnt_rgt_leg_;
  MiiPtr<Joint>   jnt_lft_wing_;
  MiiPtr<Joint>   jnt_rgt_wing_;
  MiiPtr<Joint>   jnt_cog_;
  MiiPtr<Joint>   jnt_pitch_;

  bool            alive_;
  ros::Subscriber ctrl_sub_;

  std::chrono::microseconds  tick_interval_;
  std::chrono::milliseconds  rt_interval_;
};

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
