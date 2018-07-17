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

struct __RegJntRes {
  ///! Order by { JntType::KNEE, JntType::HIP, JntType::YAW }
  Eigen::VectorXd*  resource[LegType::N_LEGS][JntDataType::N_JNT_DATA_TYPES];
  Eigen::VectorXd*  command[LegType::N_LEGS];
  __RegJntRes() {
    for (auto& c : command)
      c = nullptr;
    for (auto& rs : resource)
      for (auto& r : rs)
        r = nullptr;
  }
  ~__RegJntRes() {
    for (auto& c : command)
      if (c) delete c;
    for (auto& rs : resource)
      for (auto& r : rs)
        if (r) delete r;
  }
};

void __auto_inst(const std::string& __p, const std::string& __type) {
  if (!AutoInstor::instance()->make_instance(__p, __type)) {
    LOG_ERROR << "Create instance(" << __type << " " << __p << ") fail!";
  }

  LOG_INFO << "Create instance(" << __type << " " << __p << ")";
}

MiiRobot::MiiRobot(const std::string& __tag)
: MiiApp(), prefix_tag_(__tag), jnt_manager_(nullptr), imu_sensor_(nullptr),
  tick_interval_(20), is_alive(true), jnt_reg_res_(nullptr) {
  ;
}

MiiRobot::~MiiRobot() {
  is_alive = false;
  ///! safety control for joint, add the stop command during the shutdown.
  for (auto& j : *jnt_manager_)
    j->stop();
  sleep(1);

  Master::destroy_instance();
  JointManager::destroy_instance();
  Registry2::destroy_instance();
  SharedMem::destroy_instance();
  // destroy the auto_instancor.
  AutoInstor::destroy_instance();

  ThreadPool::destroy_instance();
  // LOG_DEBUG << "The deconstructor of MiiRobot almost finished.";
  // Label::printfEveryInstance();
}

/**
 * This method creates the part of singleton.
 */
void MiiRobot::create_system_instance() {
  // Create the other instance.
  if (nullptr == ThreadPool::create_instance())
    LOG_FATAL << "Create the singleton 'ThreadPool' has failed.";

  if (nullptr == Master::create_instance())
    LOG_FATAL << "Create the singleton 'Master' has failed.";

  if (nullptr == SharedMem::create_instance())
    LOG_FATAL << "Create the singleton 'SharedMem' has failed.";

  if (nullptr == Registry2::create_instance())
    LOG_FATAL << "Create the singleton 'Registry' has failed.";

  if (nullptr == JointManager::create_instance())
    LOG_FATAL << "Create the singleton 'JointManager' has failed.";
}

bool MiiRobot::init() {
  auto cfg = CfgReader::instance();
  if (nullptr == cfg)
    LOG_FATAL << "The CfgReader::create_instance(const std::string&) "
        << "method must to be called by subclass before MiiRobot::init()";

  std::string str;
  cfg->get_value(prefix_tag_, "control_mode", str);
  if (str.empty() || (0 == str.compare("position")))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_POS);
  else if (0 == str.compare("velocity"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_VEL);
  else if (0 == str.compare("torque"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_TOR);
  else if (0 == str.compare("pos-vel"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_POS_VEL);
  else if (0 == str.compare("motor-velocity"))
    JointManager::instance()->setJointCommandMode(JntCmdType::CMD_MOTOR_VEL);
  else
    ;

  LOG_DEBUG << "The mode of control is '" << str << "'.";
  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  cfg->regAttrCb("auto_inst", __auto_inst);
  // Just for debug
  LOG_DEBUG << "Auto instance has finished. The results list as follow:";
  Label::printfEveryInstance();

  // Now initialize the Master
  Master::instance()->init();

  jnt_manager_ = JointManager::instance();
  std::vector<std::string> vec_str;
  cfg->get_value_fatal(
      Label::make_label(prefix_tag_, "touchdowns"), "labels", vec_str);
  td_list_by_type_.resize(LegType::N_LEGS);
  for (const auto& td : vec_str) {
    ForceSensor* p_td = Label::getHardwareByName<ForceSensor>(td);
    if (nullptr != p_td) {
      auto leg = p_td->leg_type();
      td_list_.push_back(p_td);
      td_list_by_type_[leg] = p_td;
    }
  }

  std::string imu_name;
  cfg->get_value(Label::make_label(prefix_tag_, "imu"), "labels", imu_name);
  imu_sensor_  = Label::getHardwareByName<ImuSensor>(imu_name);

  __reg_resource_and_command();
  Registry2::instance()->print();
  double frequency = 1000;
  cfg->get_value(prefix_tag_, "frequency", frequency);
  tick_interval_ = std::chrono::microseconds((int)(1000000/frequency));

  // registry the thread.
  ThreadPool::instance()->add("support-registry2", &MiiRobot::supportRegistry2, this);
  return true;
}

void MiiRobot::__reg_resource_and_command() {
  auto reg    = Registry2::instance();
  auto cfg    = CfgReader::instance();
  int count   = 0;
  LegType leg = LegType::UNKNOWN_LEG;

  std::string _ptag = Label::make_label(prefix_tag_, "registry2");
  std::string _ltag = Label::make_label(_ptag, "legs");
  std::string str;
  cfg->get_value_fatal(_ltag, "mode", str);

  jnt_reg_res_ = new __RegJntRes;
//  jnt_reg_res_->cmd_mode = jnt_manager_->getJointCommandMode();
//  REG_COMMAND_NO_FLAG(str, (int*)(&jnt_reg_res_->cmd_mode));
  std::string _legs_tag = Label::make_label(_ltag, "leg_" + std::to_string(count));
  while (cfg->get_value(_legs_tag, "leg", leg)) {
    // publish the data of touchdown
    cfg->get_value_fatal(_legs_tag, "tdlo", str);
    reg->publish(str, td_list_by_type_[leg]->force_data_const_pointer());
    // publish the position of joint.
    cfg->get_value_fatal(_legs_tag, "pos", str);
    jnt_reg_res_->resource[leg][JntDataType::POS]
                    = new Eigen::VectorXd((int)JntType::N_JNTS);
    reg->publish(str, jnt_reg_res_->resource[leg][JntDataType::POS]);
    // publish the velocity of joint.
    cfg->get_value_fatal(_legs_tag, "vel", str);
    jnt_reg_res_->resource[leg][JntDataType::VEL]
                    = new Eigen::VectorXd((int)JntType::N_JNTS);
    reg->publish(str, jnt_reg_res_->resource[leg][JntDataType::VEL]);
    // publish the torque of joint.
    cfg->get_value_fatal(_legs_tag, "tor", str);
    jnt_reg_res_->resource[leg][JntDataType::TOR]
                    = new Eigen::VectorXd((int)JntType::N_JNTS);
    reg->publish(str, jnt_reg_res_->resource[leg][JntDataType::TOR]);
    // subscribe the command.
    cfg->get_value_fatal(_legs_tag, "command", str);
    jnt_reg_res_->command[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
    reg->subscribe(str, jnt_reg_res_->command[leg]);

    // advance the @_legs_tag
    _legs_tag = Label::make_label(_ltag, "leg_" + std::to_string(++count));
  }
}

void MiiRobot::supportRegistry2() {
  TICKER_INIT(std::chrono::microseconds);
  const JntCmdType& mode = jnt_manager_->getJointCommandMode();
  while (is_alive) {
    /// read joint states
    FOREACH_LEG(l) {
      FOREACH_JNTDATATYPE(d) {
        FOREACH_JNT(j) {
          if (nullptr != jnt_reg_res_->resource[l][d])
            (*(jnt_reg_res_->resource[l][d]))(j) = (*jnt_manager_)(l, j, d);
        } // end foreach jnt
      } // end foreach jntdatatype
    } // end foreach leg

    // TODO IMU
    // write
    FOREACH_LEG(l) {
      if (nullptr == jnt_reg_res_->command[l]) continue;

      auto& cmd_ref = *jnt_reg_res_->command[l];
      FOREACH_JNT(j)
        jnt_manager_->addJointCommand(l, j, cmd_ref(j));
    }
    TICKER_CONTROL(tick_interval_, std::chrono::microseconds);
  }
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
