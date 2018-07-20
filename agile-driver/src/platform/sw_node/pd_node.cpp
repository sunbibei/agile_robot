/*
 * leg_node.cpp
 *
 *  Created on: Aug 27, 2017
 *      Author: silence
 */

#include "foundation/utf.h"
#include "foundation/cfg_reader.h"

#include "repository/joint.h"
#include "repository/joint_manager.h"

#include "platform/proto/agile_proto.h"
#include "platform/sw_node/pd_node.h"

#include <iomanip>
#include <boost/algorithm/string.hpp>

namespace agile_robot {

// #define SAVE_MSG_TO_FILE
#ifdef  SAVE_MSG_TO_FILE
FILE* _msg_fd = nullptr;
#endif

// angle = \frac{360 \pi \alpha}{180*4096} C - \frac{\pi}{18000}\alpha*\beta
// so, the ABS(scale) = \frac{360 \pi \alpha}{180*4096} = \frac{360\pi}{180*4096}
// offset = - \frac{\pi}{18000}\alpha*\beta = -0.000174528*\beta
struct __LinearParams {
  double scale;
  double offset;
};

//PdNode::PdNode(const std::string& __l)
//  : SWNode(__l), leg_(LegType::UNKNOWN_LEG), td_(nullptr),
//    jnt_mode_(JointManager::instance()->getJointCommandMode()) {
//  for (auto& c : jnt_cmds_)
//    c = nullptr;
//}


PdNode::PdNode()
  : SWNode(),
    jnt_mode_(JointManager::instance()->getJointCommandMode()) {
  ;
}

PdNode::~PdNode() {
  jnts_by_name_.clear();

  for (auto& leg : params_by_name_) {
    delete leg.second;
    leg.second = nullptr;
  }
  params_by_name_.clear();
}

bool PdNode::auto_init() {
  if (!SWNode::auto_init())     return false;
  auto cfg = CfgReader::instance();

  cfg->foreachTag(getLabel(), [&](const std::string& p) {
    std::string label;
    cfg->get_value_fatal(p, "label", label);
    MiiPtr<Joint> jnt = Label::getHardwareByName<Joint>(label);
    if (nullptr == jnt) {
      LOG_ERROR << "Can't get the joint '" << label
          << "' pointer from LabelSystem, or the motor within joint.";
      return;
    }

    auto param  = new __LinearParams;
    double scale = 0, offset = 0;
    cfg->get_value_fatal(p, "scale",  scale);
    cfg->get_value_fatal(p, "offset", offset);
    param->scale  = scale;
    param->offset = offset;

    // LOG_INFO << jnt->joint_name() << ": " << scale << ", " << offset;
    jnts_by_name_[jnt->joint_name()]   = jnt;
    params_by_name_[jnt->joint_name()] = param;
  });

  return true;
}

void PdNode::handleMsg(const Packet& pkt) {
  if (pkt.node_id != node_id_ || pkt.bus_id != bus_id_) {
    // LOG_ERROR << "Wrong match id between Packet and Joint";
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
  default:
    SWNode::handleMsg(pkt);
  }
}

void PdNode::__parse_heart_beat_1(const unsigned char* __p) {
//  if (true /*&& LegType::FL == leg_*/)
//    printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//        __p[0], __p[1], __p[2], __p[3], __p[4], __p[5], __p[6], __p[7]);
  int offset  = 0;
  short count = 0;
  double pos  = 0.0;
  for (const auto& _n : {"lft-leg", "rgt-leg", "cog", "pitch"}) {
    memcpy(&count, __p + offset, sizeof(count));

    pos = params_by_name_[_n]->scale * (double)count
             + params_by_name_[_n]->offset;

    // update
    jnts_by_name_[_n]->updateJointPosition(pos);
    offset += sizeof(count);
  }
}

bool PdNode::generateCmd(std::vector<Packet>& pkts) {
  bool is_any_valid = false;
  switch (jnt_mode_) {
  case JntCmdType::CMD_POS:
    is_any_valid = __fill_pos_cmd(pkts);
    break;
//  case JntCmdType::CMD_VEL:
//    is_any_valid = __fill_vel_cmd(pkts);
//    break;
//  case JntCmdType::CMD_TOR:
//    is_any_valid = __fill_tor_cmd(pkts);
//    break;
//  case JntCmdType::CMD_POS_VEL:
//    is_any_valid = __fill_pos_vel_cmd(pkts);
//    break;
//  case JntCmdType::CMD_MOTOR_VEL:
//    is_any_valid = __fill_motor_vel_cmd(pkts);
//    break;
  default:
    LOG_ERROR << "What a fucking the command mode of joint.";
  }
  return is_any_valid;
}

bool PdNode::__fill_pos_cmd(std::vector<Packet>& pkts) {
  static std::vector<std::vector<std::string>> _s_jnts
    = {{"lft-leg",  "rgt-leg",  "cog"},
       {"lft-wing", "rgt-wing", "pitch"}};
  static unsigned char _s_msg_ids[] = {MII_MSG_COMMON_1, MII_MSG_COMMON_2};

  int  idx     = 0;
  bool has_cmd = false;
  for (const auto& _ns : _s_jnts) {
    short count        = 0;
    int   offset       = 0;
    bool  is_any_valid = false;
    Packet cmd = {bus_id_, node_id_, _s_msg_ids[idx++], 6, {0}};

    for (const auto& _n : _ns) {
      auto jnt   = jnts_by_name_[_n];
      auto param = params_by_name_[_n];

      if (jnt->new_command_) {
        has_cmd      = true;
        is_any_valid = true;
        // if ((LegType::FL == leg_) && (JntType::HIP == type))
        //   printf("LegNode: [%d] - (%d): %+01.04f\n", leg_, type, jnt_cmds_[type][0]);
        count = (jnt->joint_command() - param->offset) / param->scale;
        memcpy(cmd.data + offset, &count, sizeof(count));
        // printf("LegNode: [%s] - (%s):\t%05d\n", LEGTYPE_TOSTRING(leg_), JNTTYPE_TOSTRING(type), count);
        jnt->new_command_ = false;
      } else {
        cmd.data[offset]     = INVALID_BYTE;
        cmd.data[offset + 1] = INVALID_BYTE;
      }
      offset += sizeof(count); // Each count stand two bytes.
    }

    if (is_any_valid) pkts.push_back(cmd);
  }

  return has_cmd;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::PdNode, Label)
