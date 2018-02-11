#ifndef __QR_DRIVER_TEST_CONTROLLER2_H__
#define __QR_DRIVER_TEST_CONTROLLER2_H__

#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <repository/resource/joint.h>
#include <repository/resource/imu_sensor.h>
#include <repository/resource/force_sensor.h>

namespace qr_driver_test {

class QrDriverTestController2 :
        public controller_interface::Controller<
        hardware_interface::PositionJointInterface> {

public:
  QrDriverTestController2();
  ~QrDriverTestController2();

  bool init(hardware_interface::PositionJointInterface* robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

  void cbForReset(const std_msgs::BoolConstPtr&);

private:
  std::vector<middleware::Joint*>             joint_handles_;
  std::vector<middleware::ForceSensor*>       td_handles_;
  middleware::ImuSensor*                      imu_handle_;

  ros::Subscriber reset_sub_;
  bool is_control_;
};

} /*end namespace qr_driver_test*/

#endif
