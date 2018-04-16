/*
 * motor_node.cpp
 *
 *  Created on: Apr 12, 2018
 *      Author: bibei
 */

#include "platform/sw_node/motor_node.h"

#include "foundation/utf.h"
#include "foundation/cfg_reader.h"

#include "repository/resource/joint_manager.h"
#include "repository/resource/joint.h"
#include "repository/resource/motor.h"

#include <boost/algorithm/string.hpp>

namespace agile_robot {

MotorNode::MotorNode()
  : SWNode("motor_node"), leg_(LegType::UNKNOWN_LEG),
    jnt_mode_(JointManager::instance()->getJointCommandMode()) {
  for (auto& c : jnt_cmds_)   c = nullptr;
  for (auto& c : motor_cmds_) c = nullptr;
}

MotorNode::~MotorNode() {
  ; // Nothing to do here.
}

bool MotorNode::auto_init() {
  if (!SWNode::auto_init()) return false;
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "leg", leg_);

  motors_by_type_.resize(JntType::N_JNTS);
  joints_by_type_.resize(JntType::N_JNTS);
  FOREACH_JNT(j) {
    std::string _tag = Label::make_label(getLabel(),
        boost::to_lower_copy(std::string(JNTTYPE2STR(j))));

    std::string tmp_str;
    cfg->get_value(_tag, "label", tmp_str);
    Joint* jnt = Label::getHardwareByName<Joint>(tmp_str);
    if (!jnt || !jnt->joint_motor()) {
      LOG_ERROR << "Can't get the joint '" << tmp_str
          << "' pointer from LabelSystem, or the motor within joint.";
      continue;
    }

    joints_by_type_[jnt->joint_type()] = jnt;
    motors_by_type_[jnt->joint_type()] = jnt->joint_motor();

    jnt_cmds_[jnt->joint_type()]       = jnt->joint_command_const_pointer();
    motor_cmds_[jnt->joint_type()]     = jnt->joint_motor()->motor_command_const_pointer();
  }

  return true;
}

void MotorNode::handleMsg(const Packet&) {
  // TODO
  ;
}

bool MotorNode::generateCmd(std::vector<Packet>&) {
  // TODO
  return false;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorNode, Label)
