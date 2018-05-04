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

const size_t JNT_P_CMD_DSIZE   = 6;
const size_t JNT_PV0_CMD_DSIZE = 8;
const size_t JNT_PV1_CMD_DSIZE = 4;

// angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
// so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
// offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
struct __PrivateLinearParams {
  double scale;
  double offset;
};

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

  jnt_params_.resize(JntType::N_JNTS);
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

    auto param  = new __PrivateLinearParams;
    double alpha = 0, beta = 0;
    cfg->get_value_fatal(_tag, "scale",  alpha);
    cfg->get_value_fatal(_tag, "offset", beta);
    param->scale  = alpha * 0.001533981;
    param->offset = alpha * beta * -0.000174528;
    // LOG_DEBUG << jnt->joint_name() << ": " << param->scale << ", " << param->offset;
    jnt_params_[j] = param;

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

bool MotorNode::generateCmd(std::vector<Packet>& pkts) {

  bool is_any_valid = false;
  switch (jnt_mode_) {
  case JntCmdType::CMD_POS:
    is_any_valid = __fill_pos_cmd(pkts);
    break;
  case JntCmdType::CMD_VEL:
    is_any_valid = __fill_vel_cmd(pkts);
    break;
  case JntCmdType::CMD_TOR:
    is_any_valid = __fill_tor_cmd(pkts);
    break;
  case JntCmdType::CMD_POS_VEL:
    is_any_valid = __fill_pos_vel_cmd(pkts);
    break;
  case JntCmdType::CMD_MOTOR_VEL:
    is_any_valid = __fill_motor_vel_cmd(pkts);
    break;
  default:
    LOG_ERROR << "What a fucking the command mode of joint.";
  }

  return is_any_valid;
}


bool MotorNode::__fill_pos_cmd(std::vector<Packet>& pkts) {
//  double cmds[JntType::N_JNTS]  = {0};
//  short counts[JntType::N_JNTS] = {0};

  int offset  = 0;
  short count = 0;
  bool is_any_valid = false;
  Packet cmd = {INVALID_BYTE, node_id_, MII_MSG_COMMON_1, JNT_P_CMD_DSIZE, {0}};
  for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
    if (joints_by_type_[type]->new_command_) {
      is_any_valid = true;
//      if ((LegType::FL == leg_) && (JntType::HIP == type))
//        printf("LegNode: [%s] - (%s): %+01.04f\n", LEGTYPE_TOSTRING(leg_),
//            JNTTYPE_TOSTRING(type), jnt_cmds_[type][0]);
      count = (*jnt_cmds_[type] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
      memcpy(cmd.data + offset, &count, sizeof(count));
//      cmds[type]   = *jnt_cmds_[type];
//      counts[type] = count;
//      if (LegType::FL == leg_)
//        printf("LegNode: [%s] - (%s):\t%05d\n", LEGTYPE_TOSTRING(leg_), JNTTYPE_TOSTRING(type), count);
      joints_by_type_[type]->new_command_ = false;
    } else {
      cmd.data[offset]     = INVALID_BYTE;
      cmd.data[offset + 1] = INVALID_BYTE;
    }
    offset += sizeof(count); // Each count stand two bytes.
  }

  if (is_any_valid) {
//    if (false && LegType::FL == leg_)
//      printf("%s - %+8.5f, %+8.5f, %+8.5f\n", LEGTYPE_TOSTRING(leg_),
//          cmds[JntType::KNEE], cmds[JntType::HIP], cmds[JntType::YAW]);

    pkts.push_back(cmd);
  }

  return is_any_valid;
}

bool MotorNode::__fill_vel_cmd(std::vector<Packet>& pkts) {
  // cmd = {INVALID_BYTE, node_id_, MII_MSG_MOTOR_CMD_1, JNT_P_CMD_DSIZE, {0}};
  LOG_ERROR << "No implement velocity command!";
  return false;
}

bool MotorNode::__fill_tor_cmd(std::vector<Packet>&) {
  LOG_ERROR << "No implement torque command!";
  return false;
}

bool MotorNode::__fill_pos_vel_cmd(std::vector<Packet>& pkts) {
  int offset  = 0;
  short count = 0;
  bool is_any_valid = false;
  Packet cmd = {INVALID_BYTE, node_id_, MII_MSG_COMMON_4, JNT_PV0_CMD_DSIZE, {0}};

  for (const auto& type : {JntType::KFE, JntType::HFE}) {
    if (joints_by_type_[type]->new_command_) {
      is_any_valid = true;
      // printf("[%d] - (%d): %+01.04f %+01.04f\n", leg_, type, jnt_cmds_[type][0], jnt_cmds_[type][1]);
      count = (jnt_cmds_[type][0] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
      memcpy(cmd.data + offset, &count, sizeof(count));
      // printf("[%d] - (%d): %04d ", leg_, type, count);

      count = (jnt_cmds_[type][1] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
      memcpy(cmd.data + offset + sizeof(count), &count, sizeof(count));
      joints_by_type_[type]->new_command_ = false;
      // printf("%04d\n", count);

    } else {
      memset(cmd.data + offset, INVALID_BYTE, 2*sizeof(count));
    }

    offset += 2*sizeof(count); // Each count stand 2*two bytes.
  }
  if (is_any_valid) pkts.push_back(cmd);
  if (!joints_by_type_[JntType::HAA]->new_command_) return is_any_valid;

  is_any_valid = true;
  cmd = {INVALID_BYTE, node_id_, MII_MSG_COMMON_5, JNT_PV1_CMD_DSIZE, {0}};
  // printf("[%d] - (%d): %+01.04f %+01.04f\n", leg_, JntType::YAW, jnt_cmds_[JntType::YAW][0], jnt_cmds_[JntType::YAW][1]);
  count = (jnt_cmds_[JntType::HAA][0] - jnt_params_[JntType::HAA]->offset)
      / jnt_params_[JntType::HAA]->scale;
  memcpy(cmd.data, &count, sizeof(count));
  // printf("[%d] - (%d): %04d ", leg_, JntType::YAW, count);

  count = (jnt_cmds_[JntType::HAA][1] - jnt_params_[JntType::HAA]->offset)
      / jnt_params_[JntType::HAA]->scale;
  memcpy(cmd.data + sizeof(count), &count, sizeof(count));
  joints_by_type_[JntType::HAA]->new_command_ = false;
  // printf("%d\n", count);
  pkts.push_back(cmd);
  return is_any_valid;
}

bool MotorNode::__fill_motor_vel_cmd(std::vector<Packet>& pkts) {
  int offset  = 0;
  bool is_any_valid = false;
  Packet cmd = {INVALID_BYTE, node_id_, MII_MSG_MOTOR_2, JNT_P_CMD_DSIZE, {0}};
  for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
    if (motors_by_type_[type]->new_command_) {
      is_any_valid = true;
      // printf("Motor velocity: %04d ", *motor_cmds_[type]);
      memcpy(cmd.data + offset, motor_cmds_[type], sizeof(short));
      motors_by_type_[type]->new_command_ = false;
    } else {
      cmd.data[offset]     = INVALID_BYTE;
      cmd.data[offset + 1] = INVALID_BYTE;
    }
    offset += sizeof(short); // Each count stand two bytes.
  }

  if (is_any_valid) pkts.push_back(cmd);
  return is_any_valid;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorNode, Label)
