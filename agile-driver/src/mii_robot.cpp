/*
 * mii_robot.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: silence
 */

#include "repository/force_sensor.h"
#include "repository/imu_sensor.h"
#include "repository/joint.h"
#include "repository/joint_manager.h"

#include "foundation/label.h"
#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/thread/threadpool.h"
#include "foundation/registry/registry.h"
#include "foundation/registry/registry2.h"

#include "platform/master.h"
#include "platform/propagate/propagate_manager.h"

#include "mii_robot.h"

namespace agile_robot {

MiiRobot::MiiRobot(const std::string& __tag)
  : MiiApp(), root_tag_(__tag) {
  ;
}

MiiRobot::~MiiRobot() {
  Master::destroy_instance();
  JointManager::destroy_instance();
  Registry::destroy_instance();
  Registry2::destroy_instance();
  SharedMem::destroy_instance();
  // destroy the auto_instancor.
  // AutoInstor::destroy_instance();
  // LOG_DEBUG << "The deconstructor of MiiRobot almost finished.";
  // Label::printfEveryInstance();
}

/**
 * This method creates the part of singleton.
 */
void MiiRobot::create_system_singleton() {
  MiiApp::create_system_singleton();

  if (nullptr == Master::create_instance())
    LOG_FATAL << "Create the singleton 'Master' has failed.";

  if (nullptr == SharedMem::create_instance())
    LOG_FATAL << "Create the singleton 'SharedMem' has failed.";

  if (nullptr == Registry::create_instance())
    LOG_FATAL << "Create the singleton 'Registry' has failed.";

  if (nullptr == Registry2::create_instance())
    LOG_FATAL << "Create the singleton 'Registry2' has failed.";

  if (nullptr == JointManager::create_instance())
    LOG_FATAL << "Create the singleton 'JointManager' has failed.";
}

bool MiiRobot::init() {
  JointManager::instance()->init();

  auto cfg = CfgReader::instance();
  ///! The default mode is the joint position.
  JntCmdType mode = JntCmdType::CMD_POS;
  cfg->get_value(root_tag_, "control_mode", mode);
  JointManager::instance()->setJointCommandMode(mode);
  LOG_DEBUG << "The mode of control is '" << JNTCMDTYPE2STR(mode) << "'.";

  ///! initialize the Master.
  return Master::instance()->init();
}

bool MiiRobot::run() {
  return (Master::instance()->run() && MiiApp::run());
}

///! These methods has been deleted.
/*
void MiiRobot::addJntCmd(const MiiString& name, double command) {
  jnt_manager_->addJointCommand(name, command);
}

void MiiRobot::addJntCmd(LegType _owner, JntType _jnt, double _command) {
  jnt_manager_->addJointCommand(_owner, _jnt, _command);
}

void MiiRobot::getJointNames(std::vector<MiiString>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_name());
  }
}

void MiiRobot::getJointPositions(std::vector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_position());
  }
}

void MiiRobot::getJointVelocities(std::vector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_velocity());
  }
}

void MiiRobot::getJointTorques(std::vector<double>& ret) {
  ret.clear();
  for (const auto& jnt : *jnt_manager_) {
    ret.push_back(jnt->joint_torque());
  }
}*/

} /* namespace middleware */
