#ifndef __QR_DRIVER_TEST_CONTROLLER3_H__
#define __QR_DRIVER_TEST_CONTROLLER3_H__

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

class QrDriverTestController3 :
        public controller_interface::Controller<
        hardware_interface::PositionJointInterface> {

public:
  QrDriverTestController3();
  ~QrDriverTestController3();

  bool init(hardware_interface::PositionJointInterface* robot, ros::NodeHandle &n);
  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);

  void cbForReset(const std_msgs::BoolConstPtr&);

private:
  std::vector<agile_robot::Joint*>             joint_handles_;
  std::vector<agile_robot::ForceSensor*>       td_handles_;
  agile_robot::ImuSensor*                      imu_handle_;

///! The all of data pointer
private:
  void __initAllofData();
  std::vector<const double*>     jnt_poss_;
  std::vector<const double*>     jnt_vels_;
  std::vector<const double*>     jnt_tors_;
  std::vector<const double*>     td_datas_;
  const double*                  imu_ang_vel_;
  const double*                  imu_lin_acc_;
  const double*                  imu_quat_;

  ros::Subscriber reset_sub_;
  bool is_control_;
};

} /*end namespace qr_driver_test*/

#endif
