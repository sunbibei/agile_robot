/*
 * ros_robothw.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: silence
 */

#include "apps/ros_robothw.h"
#include "foundation/cfg_reader.h"
#include "repository/resource/imu_sensor.h"
#include "repository/resource/force_sensor.h"

RosRobotHW::~RosRobotHW() {
  for (auto& cmd : jnt_pos_cmds_) {
    delete cmd.second;
    cmd.second = nullptr;
  }

  for (auto& cmd : jnt_vel_cmds_) {
    delete cmd.second;
    cmd.second = nullptr;
  }

  for (auto& cmd : jnt_tor_cmds_) {
    delete cmd.second;
    cmd.second = nullptr;
  }
}

RosRobotHW::RosRobotHW(ros::NodeHandle& nh, const std::string& __tag)
  : nh_(nh), tag_(__tag), jnt_manager_(nullptr) {
  jnt_manager_ = JointManager::instance();
  // robot->getJointNames(joint_names_);
  num_joints_ = joint_names_.size();

  init();
  LOG_INFO << "Loaded RosRobotHW";
}

/// \brief Initialize the hardware interface
void RosRobotHW::init() {
  initJointInterface();
  initForceSensorInterface();
  initImuSensorInterface();
}

void RosRobotHW::initJointInterface() {
  if (jnt_manager_->empty()) {
    LOG_FATAL <<
        "No joints found on parameter server for controller, "
        << "did you load the proper yaml file?";
  }

  std::string jnt_name;
  const double *_pos, *_vel, *_tor;
  double* _cmd = nullptr;
  // Initialize controller
  for (size_t i = 0; i < jnt_manager_->size(); ++i) {
    jnt_name = (*jnt_manager_)[i]->joint_name();
    jnt_manager_->joint_position_const_pointer(jnt_name, _pos);
    jnt_manager_->joint_velocity_const_pointer(jnt_name, _vel);
    jnt_manager_->joint_torque_const_pointer(jnt_name,   _tor);
    // Create joint state interface
    jnt_state_iface_.registerHandle(
        hardware_interface::JointStateHandle(jnt_name,
            _pos, _vel, _tor));

    // Create position joint interface
    _cmd = new double;
    jnt_pos_iface_.registerHandle(
        hardware_interface::JointHandle(
            jnt_state_iface_.getHandle(jnt_name),
            _cmd));
    jnt_pos_cmds_.insert(std::make_pair(jnt_name, _cmd));

    // Create velocity joint interface
    _cmd = new double;
    jnt_vel_iface_.registerHandle(
        hardware_interface::JointHandle(
            jnt_state_iface_.getHandle(jnt_name),
            _cmd));
    jnt_vel_cmds_.insert(std::make_pair(jnt_name, _cmd));

    // Create effort joint interface
    _cmd = new double;
    jnt_tor_iface_.registerHandle(
        hardware_interface::JointHandle(
            jnt_state_iface_.getHandle(jnt_name),
            _cmd));
    jnt_tor_cmds_.insert(std::make_pair(jnt_name, _cmd));
  } // end for (size_t i = 0; i < jnt_manager_->size(); ++i)

  registerInterface(&jnt_state_iface_); // From RobotHW base class.
  registerInterface(&jnt_pos_iface_);   // From RobotHW base class.
  registerInterface(&jnt_vel_iface_);   // From RobotHW base class.
  registerInterface(&jnt_tor_iface_);   // From RobotHW base class.
  velocity_interface_running_ = false;
  position_interface_running_ = false;
  effort_interface_running_   = false;
}

void RosRobotHW::initForceSensorInterface() {
  auto cfg = MiiCfgReader::instance();
  int count = 0;
  std::string str = "";
  std::string tag = Label::make_label(tag_, "touchdown_0");

  while(cfg->get_value(tag, "label", str)) {
    ForceSensor* td = Label::getHardwareByName<ForceSensor>(str);
    LOG_DEBUG << tag << "'s touchdown_" << count
        << ": " << str << ",\t" << td;
    if (nullptr == td) {
      LOG_WARNING << "Can't get ForceSensor '" << str
          << "' pointer from LabelSystem.";
      tag = Label::make_label(tag_, "touchdown_" + std::to_string(++count));
      continue;
    }
    std::string frame_id, name;
    cfg->get_value_fatal(tag, "name", name);
    cfg->get_value_fatal(tag, "frame_id", frame_id);
    td_state_iface_.registerHandle(
        hardware_interface::ForceTorqueSensorHandle(name, frame_id,
            td->force_data_const_pointer(), td->force_data_const_pointer()));

    tag = Label::make_label(tag_, "touchdown_" + std::to_string(++count));
  }

  registerInterface(&td_state_iface_);
}

void RosRobotHW::initImuSensorInterface() {
  auto cfg = MiiCfgReader::instance();
  std::string tag = Label::make_label(tag_, "imu");
  hardware_interface::ImuSensorHandle::Data data;

  std::string label;
  cfg->get_value_fatal(tag, "label",    label);
  cfg->get_value_fatal(tag, "name",     data.name);
  cfg->get_value_fatal(tag, "frame_id", data.frame_id);

  ImuSensor* imu = Label::getHardwareByName<ImuSensor>(label);
  data.orientation                    = imu->orientation_const_pointer();
  data.orientation_covariance         = imu->orientation_covariance_const_pointer();
  data.linear_acceleration            = imu->linear_acceleration_const_pointer();
  data.linear_acceleration_covariance = imu->linear_acceleration_covariance_const_pointer();
  data.angular_velocity               = imu->angular_velocity_const_pointer();
  data.angular_velocity_covariance    = imu->angular_velocity_covariance_const_pointer();
  imu_state_iface_.registerHandle(data);

  registerInterface(&imu_state_iface_);
}

/// \brief Read the state from the robot hardware.
void RosRobotHW::read() {
  // auto read from Joint
}

/// \brief write the command to the robot hardware.
void RosRobotHW::write() {
  // TODO
  if (velocity_interface_running_) {
    for (auto& jnt : *jnt_manager_)
      jnt->updateJointCommand(*jnt_vel_cmds_[jnt->joint_name()]);
  } else if (position_interface_running_) {
    for (auto& jnt : *jnt_manager_)
      ;//jnt->updateJointCommand(*jnt_pos_cmds_[jnt->joint_name()]);
  } else if (effort_interface_running_) {
    LOG_WARNING << "NO IMPLEMENTES"; // Nothing to do here
  } else {
    ; // Nothing to do here
  }
}

bool RosRobotHW::canSwitch(
      const std::list<hardware_interface::ControllerInfo> &start_list,
      const std::list<hardware_interface::ControllerInfo> &stop_list) const {
  return true;
  // TODO 
}

void RosRobotHW::doSwitch(const std::list<hardware_interface::ControllerInfo>&start_list,
    const std::list<hardware_interface::ControllerInfo>&stop_list) {
  // TODO
  for (const auto& info : start_list) {
    if (0 != info.name.compare("joint_state_controller"))
      position_interface_running_ = true;
  }
}
