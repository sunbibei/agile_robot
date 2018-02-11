#include "qr_driver_test2.h"

#include <repository/resource/joint_manager.h>

namespace qr_driver_test {

static double s_init_value[] = {1.1452, -0.5471, 3.00415,
                                0.2453, -2.3457, 2.24567,
                                -2.3542, 1.1324, -1.5789,
                                1.2457, -0.4681, -2.3546};

QrDriverTestController2::QrDriverTestController2() :
  is_control_(false), imu_handle_(nullptr) {

}

QrDriverTestController2::~QrDriverTestController2() {

}

bool QrDriverTestController2::init(hardware_interface::PositionJointInterface*, ros::NodeHandle &n) {

  auto jnt_manager = middleware::JointManager::instance();

  int count = 0;
  while(true) {
    std::string joint_name;
    std::string param_name = std::string("joint_" + std::to_string(count));
    if (ros::param::get(param_name.c_str(), joint_name)) {
        std::cout << "Get Joint Name: " << joint_name << std::endl;
        auto jnt_handle = jnt_manager->getJointHandle(joint_name);
        if (nullptr == jnt_handle)
          std::cout << "The named '" << joint_name << "' joint does not exist!" << std::endl;
        else
          joint_handles_.push_back(jnt_handle);
    } else {
        break;
    }
    ++count;
  }

  count = 0;
  while(true) {
      std::string td_label;
      std::string param_name = std::string("touchdown_" + std::to_string(count));
      if (ros::param::get(param_name.c_str(), td_label)) {
          std::cout << "Get Touchdown Name: " << td_label << std::endl;
          auto td = Label::getHardwareByName<middleware::ForceSensor>(td_label);
          if (nullptr == td)
            std::cout << "The named '" << td_label << "' joint does not exist!" << std::endl;
          else
            td_handles_.push_back(td);
      } else {
          break;
      }
      ++count;
  }

    std::string imu_label;
    if (ros::param::get("imu", imu_label)) {
      imu_handle_ = Label::getHardwareByName<middleware::ImuSensor>(imu_label);
      if (nullptr == imu_handle_)
        std::cout << "The named '" << imu_label << "' joint does not exist!" << std::endl;
    }

    reset_sub_ = n.subscribe<std_msgs::Bool>("QrDriverTest2", 1,
       &QrDriverTestController2::cbForReset, this);
    return true;
}

void QrDriverTestController2::cbForReset(const std_msgs::BoolConstPtr& msg) {
  is_control_ = msg->data;
}

void QrDriverTestController2::starting(const ros::Time& time) {

}

void QrDriverTestController2::update(const ros::Time&, const ros::Duration&) {
  if (!is_control_) return;
  is_control_ = false;

  std::cout << "JointStates: \n";
  for (const auto& j : joint_handles_) {
    printf("[%d] - (%d): %+04f, %+04f, %+04f\n", j->leg_type(), j->joint_type(),
        j->joint_position(), j->joint_velocity(), j->joint_torque());
  }
  std::cout << std::endl;

  std::cout << "ForceSensor: ";
  for (const auto& td : td_handles_) {
    printf("%04f ", td->force_data());
  }
  std::cout << std::endl;

  if (imu_handle_) {
    ; // Output the information of IMU
    std::cout << "ImuSensor:" << std::endl;
    auto d = imu_handle_->angular_velocity_const_pointer();
    std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
    d = imu_handle_->linear_acceleration_const_pointer();
    std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
    d = imu_handle_->orientation_const_pointer();
    std::cout << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << std::endl;
  }

  std::cout << "JointCommand: \n";
  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    printf("[%d] - (%d): %+01.04f %+01.04f\n", joint_handles_[i]->leg_type(),
      joint_handles_[i]->joint_type(), i*0.01, 0.88);

    joint_handles_[i]->updateJointCommand(i*0.01, 0.88);
  }
  std::cout << std::endl;
}

} /* end namespace qr_driver_test */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(qr_driver_test::QrDriverTestController2, controller_interface::ControllerBase)
