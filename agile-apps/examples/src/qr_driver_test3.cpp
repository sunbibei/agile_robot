#include "qr_driver_test3.h"

#include <repository/resource/joint_manager.h>

namespace qr_driver_test {

static double _s_init_value[] = {1.1452, -0.5471, 3.00415,
                                 0.2453, -2.3457, 2.24567,
                                 -2.3542, 1.1324, -1.5789,
                                 1.2457, -0.4681, -2.3546};

QrDriverTestController3::QrDriverTestController3() :
  is_control_(false), imu_handle_(nullptr),
  imu_ang_vel_(nullptr), imu_lin_acc_(nullptr), imu_quat_(nullptr) {
}

QrDriverTestController3::~QrDriverTestController3() {

}

/**************************************************************************
   Description: initialize joints from robot_description
**************************************************************************/
bool QrDriverTestController3::init(hardware_interface::PositionJointInterface*, ros::NodeHandle &n) {

  auto jnt_manager = middleware::JointManager::instance();
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR})
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW})
      joint_handles_.push_back(jnt_manager->getJointHandle(leg, jnt));


  int count = 0;
  td_handles_.resize(LegType::N_LEGS);
  while(true) {
      std::string td_label;
      std::string param_name = std::string("touchdown_" + std::to_string(count));
      if (ros::param::get(param_name.c_str(), td_label)) {
          std::cout << "Get Touchdown Name: " << td_label << std::endl;
          auto td = Label::getHardwareByName<middleware::ForceSensor>(td_label);
          if (nullptr == td)
            LOG_ERROR << "The named '" << td_label << "' joint does not exist!";
          else
            td_handles_[td->leg_type()] = td;
      } else {
          break;
      }
      ++count;
  }

  std::string imu_label;
  if (ros::param::get("imu", imu_label)) {
    imu_handle_ = Label::getHardwareByName<middleware::ImuSensor>(imu_label);
    if (nullptr == imu_handle_)
      LOG_ERROR << "The named '" << imu_label << "' joint does not exist!";
  }

  __initAllofData();
  reset_sub_ = n.subscribe<std_msgs::Bool>("QrDriverTest3", 1,
     &QrDriverTestController3::cbForReset, this);
  return true;
}

void QrDriverTestController3::__initAllofData() {
  jnt_poss_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_vels_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  jnt_tors_.reserve(LegType::N_LEGS * JntType::N_JNTS);
  td_datas_.reserve(LegType::N_LEGS);

  auto jnt_manager = middleware::JointManager::instance();
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    td_datas_.push_back(td_handles_[leg]->force_data_const_pointer());
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      auto j = jnt_manager->getJointHandle(leg, jnt);
      jnt_poss_.push_back(j->joint_position_const_pointer());
      jnt_vels_.push_back(j->joint_velocity_const_pointer());
      jnt_tors_.push_back(j->joint_torque_const_pointer());
    }
  }

  imu_ang_vel_ = imu_handle_->angular_velocity_const_pointer();
  imu_lin_acc_ = imu_handle_->linear_acceleration_const_pointer();
  imu_quat_    = imu_handle_->orientation_const_pointer();
}

void QrDriverTestController3::cbForReset(const std_msgs::BoolConstPtr& msg) {
  is_control_ = msg->data;
}


void QrDriverTestController3::starting(const ros::Time& time) {

}

void QrDriverTestController3::update(const ros::Time&, const ros::Duration&) {
  if (!is_control_) return;
  is_control_ = false;

  std::cout << "JointStates: \n";
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      int idx = leg*JntType::N_JNTS+jnt;
      printf("[%d] - (%d): %+04f, %+04f, %+04f\n", leg, jnt,
          *(jnt_poss_[idx]), *(jnt_vels_[idx]), *(jnt_tors_[idx]));
    }
  }
  std::cout << std::endl;

  std::cout << "ForceSensor: ";
  for (const auto& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    printf("%04f ", *(td_datas_[leg]));
  }
  std::cout << std::endl;

  if (imu_handle_) {
    ; // Output the information of IMU
    std::cout << "ImuSensor:" << std::endl;
    auto d = imu_ang_vel_;
    std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
    d = imu_lin_acc_;
    std::cout << d[0] << " " << d[1] << " " << d[2] << std::endl;
    d = imu_quat_;
    std::cout << d[0] << " " << d[1] << " " << d[2] << " " << d[3] << std::endl;
  }


  std::cout << "JointCommand: " << std::endl;
  for (size_t i = 0; i < joint_handles_.size(); ++i) {
    printf("[%d] - (%d): %+01.04f %+01.04f\n", joint_handles_[i]->leg_type(),
      joint_handles_[i]->joint_type(), _s_init_value[i] + 0.0001*i, 0.88*i);
    // The first value is position command, and the second value is velocity command.
    joint_handles_[i]->updateJointCommand(_s_init_value[i] + 0.0001*i, 0.88*i);
  }
  std::cout << std::endl;
}

} /* end namespace qr_driver_test */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(qr_driver_test::QrDriverTestController3, controller_interface::ControllerBase)
