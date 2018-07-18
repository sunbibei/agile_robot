/*
 * qr_ros_wrapper.h
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */

#ifndef INCLUDE_QR_ROBOT_WRAPPER_H_
#define INCLUDE_QR_ROBOT_WRAPPER_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>

#include <chrono>
#include <boost/scoped_ptr.hpp>

#include "mii_robot.h"

#define DEBUG_TOPIC
#ifdef DEBUG_TOPIC
#include <repository/joint.h>
#endif

using namespace agile_robot;

class RobotWrapper : public MiiRobot {
SINGLETON_DECLARE(RobotWrapper, const std::string&)

protected:
  /**
   * @brief Constructed function.
   * @param _tag        Every necessary parameters will be found in this __tag
   */
  RobotWrapper(const std::string&, const std::string&);
  /*!
   * @brief STEP 2:
   *        Create the all of singleton in our system, this method will be
   *        called before the @init() after the @prev_init(),
   *        If something was wrong, return NOTHING, SHUTDOWN the process directly.
   */
  virtual void create_system_singleton() override;
  /*!
   * @brief STEP 3:
   *        Create the all of instance in the configure file using the given
   *        Callback method or the default Callback @auto_inst_cb(), If you
   *        want to move the default callback to a self-define callback, calling
   *        the @mv_auto_inst_cb()
   */
  // virtual void auto_inst() override;
  /*!
   * @brief STEP 4:
   *        This function will be called lastly after the @init(), it must
   *        be completed the process of AutoInst and register the all of
   *        thread what our system need.
   */
  virtual bool init() override;

private:
  ///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
  void publishRTMsg();

  // Just for test
#ifdef DEBUG_TOPIC
  void cbForDebug(const std_msgs::Float32ConstPtr&);
  ros::Subscriber cmd_sub_;
#endif

private:
  // The ROS handle
  ros::NodeHandle   nh_;
  std::string       root_tag_;

  bool alive_;
  std::chrono::milliseconds  rt_duration_;
};

#endif /* INCLUDE_QR_ROS_WRAPPER_H_ */
