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


PdNode::PdNode() : SWNode() {
  ;
}

PdNode::~PdNode() {
  jnts_by_type_.clear();

  for (auto& leg : jnt_params_) {
    for (auto& p : leg) {
      delete p;
      p = nullptr;
    }
  }
  jnt_params_.clear();
}

bool PdNode::auto_init() {
  if (!SWNode::auto_init())     return false;
  auto cfg = CfgReader::instance();

  jnts_by_type_.resize(LegType::N_LEGS);
  for (auto& leg : jnts_by_type_)
    leg.resize(JntType::N_JNTS);

  jnt_params_.resize(LegType::N_LEGS);
  for (auto& leg : jnt_params_)
    leg.resize(JntType::N_JNTS);

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

    LOG_INFO << jnt->joint_name() << ": " << scale << ", " << offset;

    jnts_by_type_[jnt->leg_type()][jnt->joint_type()] = jnt;
    jnt_params_[jnt->leg_type()][jnt->joint_type()]   = param;
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
//  if (true && LegType::FL == leg_)
//    printf("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//        __p[0], __p[1], __p[2], __p[3], __p[4], __p[5], __p[6], __p[7]);
  int offset  = 0;
  short count = 0;
  double pos  = 0.0;
  short counts[LegType::N_LEGS][JntType::N_JNTS] = {0};
  for (const auto& leg : {LegType::FL, LegType::FR}) {
    for (const auto& jnt : {JntType::HFE, JntType::KFE}) {
      memcpy(&count, __p + offset, sizeof(count));
      counts[leg][jnt] = count;

      pos = jnt_params_[leg][jnt]->scale * (double)count
               + jnt_params_[leg][jnt]->offset;

      // update
      jnts_by_type_[leg][jnt]->updateJointPosition(pos);
      offset += sizeof(count);
    }
  }

  if (false)
    printf("%+5d, %+5d, %+5d, %+5d\n",
        counts[LegType::FL][JntType::HFE], counts[LegType::FL][JntType::KFE],
        counts[LegType::FR][JntType::HFE], counts[LegType::FR][JntType::KFE]);
}

// TODO fill some command into the @pkts
bool PdNode::generateCmd(std::vector<Packet>& pkts) {
  return false;
}

} /* namespace middleware */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::PdNode, Label)
