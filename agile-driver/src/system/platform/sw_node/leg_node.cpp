/*
 * leg_node.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#include "foundation/utf.h"
#include "foundation/cfg_reader.h"

#include "repository/resource/force_sensor.h"
#include "repository/resource/joint_manager.h"
#include "repository/resource/joint.h"
#include "repository/resource/motor.h"

#include "system/platform/sw_node/leg_node.h"
#include "system/platform/protocol/qr_protocol.h"

#include <iomanip>
#include <boost/algorithm/string.hpp>

namespace middleware {

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

LegNode::LegNode(const MiiString& __l)
  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr),
    jnt_mode_(JointManager::instance()->getJointCommandMode()) {
  for (auto& c : jnt_cmds_)
    c = nullptr;
}

LegNode::~LegNode() {
  for (auto& jnt : jnts_by_type_) {
    jnt = nullptr;
  }
  td_ = nullptr;
  for (auto& p : jnt_params_) {
    delete p;
    p = nullptr;
  }
}

bool LegNode::auto_init() {
  if (!SWNode::auto_init())     return false;
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(getLabel(), "leg", leg_);

  int count = 0;
  jnts_by_type_.resize(JntType::N_JNTS);
  motors_by_type_.resize(JntType::N_JNTS);
  jnt_params_.resize(JntType::N_JNTS);
  MiiString tag = Label::make_label(getLabel(), "joint_0");
  MiiString tmp_str;
  while(cfg->get_value(tag, "label", tmp_str)) {
    Joint* jnt = Label::getHardwareByName<Joint>(tmp_str);
    // LOG_DEBUG << getLabel() << "'s joint_" << count
    //     << ": " << tmp_str << ",\t" << jnt;
    if (nullptr == jnt) {
      LOG_WARNING << "Can't get joint '" << tmp_str
          << "' pointer from LabelSystem.";
      tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
      continue;
    }
    jnts_by_type_[jnt->joint_type()]   = jnt;
    if (jnt->joint_motor_)
      motors_by_type_[jnt->joint_type()] = jnt->joint_motor_;

    auto param  = new __PrivateLinearParams;
    double alpha, beta = 0;
    cfg->get_value_fatal(tag, "scale",  alpha);
    cfg->get_value_fatal(tag, "offset", beta);
    param->scale  = alpha * 0.001533981;
    param->offset = alpha * beta * -0.000174528;
    jnt_params_[jnt->joint_type()] = param;
    jnt_cmds_[jnt->joint_type()]   = jnt->joint_command_const_pointer();
    if (jnt->joint_motor_)
      motor_cmds_[jnt->joint_type()] = jnt->joint_motor_->motor_command_const_pointer();

    tag = Label::make_label(getLabel(), "joint_" + std::to_string(++count));
    if (3 == count) break;
  }

  tag = Label::make_label(getLabel(), "touchdown");
  if ((cfg->get_value(tag, "label", tmp_str))
      && (td_ = Label::getHardwareByName<ForceSensor>(tmp_str))) {
    // LOG_DEBUG << getLabel() << "'s TD: " << td_->getLabel() << "\t" << td_;
    return true;
  } else {
    LOG_WARNING << "The touchdown parameter is not found.";
    return false;
  }
}

void LegNode::handleMsg(const Packet& pkt) {
  if (pkt.node_id != node_id_) {
    LOG_ERROR << "Wrong match id between Packet and Joint";
    return;
  }

  if (true)
    printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)pkt.node_id,
      (int)pkt.msg_id,  (int)pkt.size,
      (int)pkt.data[0], (int)pkt.data[1],
      (int)pkt.data[2], (int)pkt.data[3],
      (int)pkt.data[4], (int)pkt.data[5],
      (int)pkt.data[6], (int)pkt.data[7]);

  switch (pkt.msg_id) {
  case MII_MSG_HEARTBEAT_MSG_1:
    if (8 != pkt.size) {
      LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
          << ", the expect size is 8, but the real size is " << pkt.size;
      return;
    }
    // parse the joint state and touchdown data
    __parse_heart_beat_1(pkt.data);
    break;
  case MII_MSG_MOTOR_CMD_1:
    if (6 != pkt.size) {
      LOG_ERROR << "The data size of MII_MSG_MOTOR_CMD_1 message does not match!"
          << ", the expect size is 6, but the real size is " << pkt.size;
      return;
    }
    // PRESS_THEN_GO
    __parse_motor_cmd_1(pkt.data);
    break;
  case MII_MSG_MOTOR_CMD_2:
    if (6 != pkt.size) {
      LOG_ERROR << "The data size of MII_MSG_MOTOR_CMD_2 message does not match!"
          << ", the expect size is 6, but the real size is " << pkt.size;
      return;
    }
    // PRESS_THEN_GO
    __parse_motor_cmd_2(pkt.data);
    break;
  default:
    SWNode::handleMsg(pkt);
  }
}

void LegNode::__parse_heart_beat_1(const unsigned char* __p) {
  /*printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", 
      __p[0], __p[1], __p[2], __p[3], __p[4], __p[5], __p[6], __p[7]);*/
  int offset  = 0;
  short count = 0;
  double pos  = 0.0;
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    memcpy(&count, __p + offset, sizeof(count));
    // angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
    // so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
    // offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
    pos = jnt_params_[type]->scale * (double)count + jnt_params_[type]->offset;
    
    jnts_by_type_[type]->updateJointPosition(pos);
    offset += sizeof(count); // each count will stand two bytes.
  }

  // if (LegType::HL == leg_) printf("%d: 0x%02X, 0x%02X", leg_, __p[offset], __p[offset + 1]);
  td_->updateForceCount((__p[offset] | (__p[offset + 1] << 8)));
}

void LegNode::__parse_motor_cmd_1(const unsigned char* __p) {
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    // memcpy(&count, __p + offset, sizeof(count));
    motors_by_type_[type]->updateMotorPosition((__p[0] | (__p[1] << 8)));
    __p += sizeof(short); // each count will stand two bytes.
  }
}

void LegNode::__parse_motor_cmd_2(const unsigned char* __p) {
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    // memcpy(&count, __p + offset, sizeof(count));
    motors_by_type_[type]->updateMotorVelocity((__p[0] | (__p[1] << 8)));
    __p += sizeof(short); // each count will stand two bytes.
  }
}

bool LegNode::generateCmd(MiiVector<Packet>& pkts) {

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


bool LegNode::__fill_pos_cmd(MiiVector<Packet>& pkts) {
  int offset  = 0;
  short count = 0;
  bool is_any_valid = false;
  Packet cmd = {INVALID_BYTE, node_id_, MII_MSG_COMMON_DATA_1, JNT_P_CMD_DSIZE, {0}};
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    if (jnts_by_type_[type]->new_command_) {
      is_any_valid = true;
      // if ((LegType::FL == leg_) && (JntType::HIP == type))
      //   printf("LegNode: [%d] - (%d): %+01.04f\n", leg_, type, jnt_cmds_[type][0]);
      count = (*jnt_cmds_[type] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
      memcpy(cmd.data + offset, &count, sizeof(count));
      // printf("LegNode: [%s] - (%s):\t%05d\n", LEGTYPE_TOSTRING(leg_), JNTTYPE_TOSTRING(type), count);
      jnts_by_type_[type]->new_command_ = false;
    } else {
      cmd.data[offset]     = INVALID_BYTE;
      cmd.data[offset + 1] = INVALID_BYTE;
    }
    offset += sizeof(count); // Each count stand two bytes.
  }

  if (is_any_valid) pkts.push_back(cmd);
  return is_any_valid;
}

bool LegNode::__fill_vel_cmd(MiiVector<Packet>& pkts) {
  // cmd = {INVALID_BYTE, node_id_, MII_MSG_MOTOR_CMD_1, JNT_P_CMD_DSIZE, {0}};
  LOG_ERROR << "No implement velocity command!";
  return false;
}

bool LegNode::__fill_tor_cmd(MiiVector<Packet>&) {
  LOG_ERROR << "No implement torque command!";
  return false;
}

bool LegNode::__fill_pos_vel_cmd(MiiVector<Packet>& pkts) {
  int offset  = 0;
  short count = 0;
  bool is_any_valid = false;
  Packet cmd = {INVALID_BYTE, node_id_, MII_MSG_COMMON_DATA_4, JNT_PV0_CMD_DSIZE, {0}};

  for (const auto& type : {JntType::KNEE, JntType::HIP}) {
    if (jnts_by_type_[type]->new_command_) {
      is_any_valid = true;
      // printf("[%d] - (%d): %+01.04f %+01.04f\n", leg_, type, jnt_cmds_[type][0], jnt_cmds_[type][1]);
      count = (jnt_cmds_[type][0] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
      memcpy(cmd.data + offset, &count, sizeof(count));
      // printf("[%d] - (%d): %04d ", leg_, type, count);

      count = (jnt_cmds_[type][1] - jnt_params_[type]->offset) / jnt_params_[type]->scale;
      memcpy(cmd.data + offset + sizeof(count), &count, sizeof(count));
      jnts_by_type_[type]->new_command_ = false;
      // printf("%04d\n", count);

    } else {
      memset(cmd.data + offset, INVALID_BYTE, 2*sizeof(count));
    }

    offset += 2*sizeof(count); // Each count stand 2*two bytes.
  }
  if (is_any_valid) pkts.push_back(cmd);
  if (!jnts_by_type_[JntType::YAW]->new_command_) return is_any_valid;

  is_any_valid = true;
  cmd = {INVALID_BYTE, node_id_, MII_MSG_COMMON_DATA_5, JNT_PV1_CMD_DSIZE, {0}};
  // printf("[%d] - (%d): %+01.04f %+01.04f\n", leg_, JntType::YAW, jnt_cmds_[JntType::YAW][0], jnt_cmds_[JntType::YAW][1]);
  count = (jnt_cmds_[JntType::YAW][0] - jnt_params_[JntType::YAW]->offset)
      / jnt_params_[JntType::YAW]->scale;
  memcpy(cmd.data, &count, sizeof(count));
  // printf("[%d] - (%d): %04d ", leg_, JntType::YAW, count);

  count = (jnt_cmds_[JntType::YAW][1] - jnt_params_[JntType::YAW]->offset)
      / jnt_params_[JntType::YAW]->scale;
  memcpy(cmd.data + sizeof(count), &count, sizeof(count));
  jnts_by_type_[JntType::YAW]->new_command_ = false;
  // printf("%d\n", count);
  pkts.push_back(cmd);
  return is_any_valid;
}

bool LegNode::__fill_motor_vel_cmd(MiiVector<Packet>& pkts) {
  int offset  = 0;
  bool is_any_valid = false;
  Packet cmd = {INVALID_BYTE, node_id_, MII_MSG_MOTOR_CMD_2, JNT_P_CMD_DSIZE, {0}};
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
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

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::LegNode, Label)
