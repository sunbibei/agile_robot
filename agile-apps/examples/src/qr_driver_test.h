#ifndef __QR_DRIVER_TEST_CONTROLLER_H__
#define __QR_DRIVER_TEST_CONTROLLER_H__

#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>

#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/force_torque_sensor_interface.h>

#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>

namespace qr_driver_test {

class QrDriverTestController :
        public controller_interface::MultiInterfaceController<
        hardware_interface::PositionJointInterface,
        hardware_interface::ForceTorqueSensorInterface,
        hardware_interface::ImuSensorInterface> {
public:
  QrDriverTestController();
  ~QrDriverTestController();

  bool init(hardware_interface::RobotHW* robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

  void cbForReset(const std_msgs::BoolConstPtr&);
private:
  std::vector<hardware_interface::JointHandle>             joint_handles_;
  std::vector<hardware_interface::ForceTorqueSensorHandle> td_handles_;
  hardware_interface::ImuSensorHandle                      imu_handle_;
  ros::Subscriber reset_sub_;
  bool is_control_;
};

} /*end namespace qr_driver_test*/

#endif
