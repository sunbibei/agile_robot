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

#include "platform/proto/agile_proto.h"
#include "platform/sw_node/leg_node.h"

#include <iomanip>
#include <boost/algorithm/string.hpp>

namespace agile_robot {

// #define SAVE_MSG_TO_FILE
#ifdef  SAVE_MSG_TO_FILE
FILE* _msg_fd = nullptr;
#endif

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

//LegNode::LegNode(const std::string& __l)
//  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr),
//    jnt_mode_(JointManager::instance()->getJointCommandMode()) {
//  for (auto& c : jnt_cmds_)
//    c = nullptr;
//}


LegNode::LegNode(const std::string& __l)
  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr) {
  ;
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

  // motors_by_type_.resize(JntType::N_JNTS);
  jnts_by_type_.resize(JntType::N_JNTS);
  jnt_params_.resize(JntType::N_JNTS);
  std::string tmp_str, _tag;
  FOREACH_JNT(j) {
    // tmp_str = ;
    _tag = Label::make_label(getLabel(),
        boost::to_lower_copy(std::string(JNTTYPE2STR(j))));

    cfg->get_value(_tag, "label", tmp_str);
    Joint* jnt = Label::getHardwareByName<Joint>(tmp_str);
    if (nullptr == jnt) {
      LOG_ERROR << "Can't get the joint '" << tmp_str
          << "' pointer from LabelSystem, or the motor within joint.";
      continue;
    }

    jnts_by_type_[j]   = jnt;
    //    if (jnt->joint_motor_)
    //      motors_by_type_[jnt->joint_type()] = jnt->joint_motor_;

    auto param  = new __PrivateLinearParams;
    double scale = 0, offset = 0;
    cfg->get_value_fatal(_tag, "scale",  scale);
    cfg->get_value_fatal(_tag, "offset", offset);
    param->scale  = scale;
    param->offset = offset;
    // LOG_DEBUG << jnt->joint_name() << ": " << param->scale << ", " << param->offset;
    jnt_params_[j] = param;
    //    jnt_cmds_[j]   = jnt->joint_command_const_pointer();
    //    if (jnt->joint_motor_)
    //      motor_cmds_[j] = jnt->joint_motor_->motor_command_const_pointer();
  } // end FOREACH_JNT(j)

  // PRESS_THEN_GO
#ifdef  SAVE_MSG_TO_FILE
  _msg_fd = fopen("/home/bibei/Workspaces/agile_ws/src/agile_robot/agile-apps/config/ag", "w+");
#endif

  _tag = Label::make_label(getLabel(), "touchdown");
  if ((cfg->get_value(_tag, "label", tmp_str))
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

  if (false) {
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)pkt.node_id,
      (int)pkt.msg_id,  (int)pkt.size,
      (int)pkt.data[0], (int)pkt.data[1],
      (int)pkt.data[2], (int)pkt.data[3],
      (int)pkt.data[4], (int)pkt.data[5],
      (int)pkt.data[6], (int)pkt.data[7]);
  }

  switch (pkt.msg_id) {
  case MII_MSG_HEARTBEAT_1:
    if (8 != pkt.size) {
      LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
          << ", the expect size is 8, but the real size is " << (int)pkt.size;
      return;
    }
    // parse the joint state and touchdown data
    __parse_heart_beat_1(pkt.data);
    break;
  case MII_MSG_HEARTBEAT_2:
    if (4 != pkt.size) {
      LOG_ERROR << "The data size of MII_MSG_HEARTBEAT_MSG_1 message does not match!"
          << ", the expect size is 8, but the real size is " << (int)pkt.size;
      return;
    }
    // parse the joint state and touchdown data
    __parse_heart_beat_2(pkt.data);
    break;
  default:
    SWNode::handleMsg(pkt);
  }
}

void LegNode::__parse_heart_beat_1(const unsigned char* __p) {
//  if (true && LegType::FL == leg_)
//    printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//        __p[0], __p[1], __p[2], __p[3], __p[4], __p[5], __p[6], __p[7]);
  int offset  = 0;
  short count = 0;
  double pos  = 0.0;
  short counts[JntType::N_JNTS] = {0};
  // for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
  FOREACH_JNT(type) {
    memcpy(&count, __p + offset, sizeof(count));
    counts[type] = count;
    // scale = scale * 10000 ; offset = offset * 10000 ;
    pos = jnt_params_[type]->scale * (double)counts[type] + jnt_params_[type]->offset;
    pos *= 0.0001;
    // pos = 15.343 * (double)count /10000 + 11116 /10000;
    // pos = count; // TODO
   // std::cout << "bianmaqidzhi: " << JNTTYPE2STR(type) << ": " << pos << std::endl;
    if (JntType::HAA == type) /// TODO
      jnts_by_type_[type]->updateJointPosition(0.0);
    else
      jnts_by_type_[type]->updateJointPosition(pos);
    offset += sizeof(count); // each count will stand two bytes.
  }

  if (false && LegType::FL == leg_)
#ifdef  SAVE_MSG_TO_FILE
    fprintf(_msg_fd, "%s - %+5d, %+5d, %+5d\n", LEGTYPE2STR(leg_),
        counts[JntType::KFE], counts[JntType::HFE], counts[JntType::HAA]);
#else
    printf("%s - %+5d, %+5d, %+5d\n", LEGTYPE2STR(leg_),
        counts[JntType::KFE], counts[JntType::HFE], counts[JntType::HAA]);
#endif
  // if (LegType::HL == leg_) printf("%d: 0x%02X, 0x%02X", leg_, __p[offset], __p[offset + 1]);
  td_->updateForceCount((__p[offset] | (__p[offset + 1] << 8)));
}

void LegNode::__parse_heart_beat_2(const unsigned char* __p) {
  // if (true && LegType::FL == leg_)
//    printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//        __p[0], __p[1], __p[2], __p[3], __p[4], __p[5], __p[6], __p[7]);
  int offset  = 0;
  short count = 0;
  double tor  = 0.0;
  short counts[JntType::N_JNTS] = {0};
  for (const auto& type : {JntType::KFE, JntType::HFE}) {
    memcpy(&count, __p + offset, sizeof(count));
    counts[type] = count;
    // angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
    // so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
    // offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
    // pos = jnt_params_[type]->scale * (double)count + jnt_params_[type]->offset;
    tor = count; // TODO

    jnts_by_type_[type]->updateJointTorque(tor);
    offset += sizeof(count); // each count will stand two bytes.
  }

  if (false && LegType::FL == leg_)
#ifdef  SAVE_MSG_TO_FILE
    fprintf(_msg_fd, "%s - %+5d, %+5d, %+5d\n", LEGTYPE2STR(leg_),
        counts[JntType::KFE], counts[JntType::HFE], counts[JntType::HAA]);
#else
    printf("%s - %+5d, %+5d, %+5d\n", LEGTYPE2STR(leg_),
        counts[JntType::KFE], counts[JntType::HFE], counts[JntType::HAA]);
#endif
  // if (LegType::HL == leg_) printf("%d: 0x%02X, 0x%02X", leg_, __p[offset], __p[offset + 1]);
  td_->updateForceCount((__p[offset] | (__p[offset + 1] << 8)));
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::LegNode, Label)
