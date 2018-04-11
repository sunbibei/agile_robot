/*
 * motor_pcan.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#include "system/platform/propagate/motor_pcan.h"
#include "foundation/cfg_reader.h"

namespace agile_robot {

const std::string FAKE_PID_THREAD = "fake-pid";


MotorPcan::MotorPcan(const std::string& l)
  : ArmPcan(l), new_command_(false), pid_hijack_(false) {
  pids_.resize(MAX_NODE_NUM);
  for (auto& pid : pids_)
    pid.resize(JntType::N_JNTS);
}

bool MotorPcan::auto_init() {
  if (!ArmPcan::auto_init()) return false;
  MiiCfgReader::instance()->get_value(getLabel(), "hijack", pid_hijack_);

  int count = 0;
  unsigned char node_id = INVALID_BYTE;
  std::string tag = Label::make_label(getLabel(), "pid_" + std::to_string(count));
  while(MiiCfgReader::instance()->get_value(tag, "node_id", node_id)) {
    auto_inst_pid(tag);
    tag = Label::make_label(getLabel(), "pid_" + std::to_string(++count));
  }
  return true;
}

MotorPcan::~MotorPcan() {
  for (auto& pids : pids_)
    for (auto& pid : pids)
      delete pid;

  pids_.clear();
}

bool MotorPcan::write(const Packet& pkt) {
  if (MII_MSG_COMMON_1 == pkt.msg_id) {
    int offset = 0;
    for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
      if ((INVALID_BYTE == pkt.data[offset]) && (INVALID_BYTE == pkt.data[offset + 1])) {
        offset += sizeof(short);
        continue;
      }

      memcpy(&(T_[pkt.node_id][type]), pkt.data + offset, sizeof(short));
      offset += sizeof(short);
      // LOG_INFO << "Set target: " << T_[pkt.node_id][type];
      pids_[pkt.node_id][type]->setTarget(T_[pkt.node_id][type]);
    }
  }
  if (false)
    printf("MotorPcan -> NODE ID:0x%02X MSG ID: 0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
          (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
          (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);

  return ( pid_hijack_ ? true : ArmPcan::write(pkt));
}

bool MotorPcan::read(Packet& pkt) {
  if (!ArmPcan::read(pkt)) return false;

  if (MII_MSG_HEARTBEAT_1 == pkt.msg_id) {
    /*memcpy(&(X_[pkt.node_id][JntType::KNEE]), pkt.data + 0, sizeof(short));
    memcpy(&(X_[pkt.node_id][JntType::HIP]),  pkt.data + 2, sizeof(short));
    memcpy(&(X_[pkt.node_id][JntType::YAW]),  pkt.data + 4, sizeof(short));*/
    int offset = 0;
    for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
      memcpy(&(X_[pkt.node_id][type]), pkt.data + offset, sizeof(short));
      offset += sizeof(short);
    }
  }

  if (pid_hijack_) updatePID(pkt.node_id);
  return true;
}

void MotorPcan::updatePID(unsigned char node_id) {
  Packet pkt = {bus_id_, node_id, MII_MSG_MOTOR_2, 6, {0}};

  int offset = 0;
  for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
    if (pids_[node_id][type]->control(X_[node_id][type], U_[node_id][type])) {
      memcpy(pkt.data + offset, U_[node_id] + type, sizeof(short));
    } else {
      memset(pkt.data + offset, 0x00, sizeof(short));
    }
    
    offset += sizeof(short);
  }
  ArmPcan::write(pkt);
}

void MotorPcan::auto_inst_pid(const std::string& __p) {
  unsigned char node_id = INVALID_BYTE;
  auto cfg = MiiCfgReader::instance();
  cfg->get_value_fatal(__p, "node_id", node_id);
  bool is_insert = false;
  for (const auto& id : node_ids_) {
    if (id == node_id) is_insert = true;
  }
  if (!is_insert) node_ids_.push_back(node_id);
  JntType jnt = JntType::UNKNOWN_JNT;
  cfg->get_value_fatal(__p, "jnt", jnt);
  pids_[node_id][jnt] = new Pid(__p);
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorPcan, Label)
