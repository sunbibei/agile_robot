/*
 * motor_can.cpp
 *
 *  Created on: May 16, 2018
 *      Author: bibei
 */

#include <platform/propagate/motor_can.h>
#include <iostream>

namespace agile_robot {

const unsigned int ID_BASE = 0x300u;

MotorCan::MotorCan()
  : CanUsb() {
  ///! setting the default value.
  for (auto& key : msgid2idx_lut_)
    key = INVALID_BYTE;

  ///! fill the actual key-value map.
  // TODO The follow line is a example
  msgid2idx_lut_[MII_MSG_HEARTBEAT_1] = 0x00004646l;
  idx2msgid_lut_[0x00004646l]         = MII_MSG_HEARTBEAT_1;
}

MotorCan::~MotorCan() {
  ; // Nothing to do here.
}

bool MotorCan::write(const Packet& pkt) {
  if (!connected_ || pkt.bus_id != bus_id_) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }

  send_can_obj_.ID = ID_BASE + pkt.node_id;
  if (INVALID_BYTE == msgid2idx_lut_[pkt.msg_id]) {
    LOG_ERROR << "Wrong format Packet";
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
      (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
      (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);

    ///! Something is wrong, try to whether handle this wrong in the base class.
    return CanUsb::write(pkt);
  }

  memcpy(send_can_obj_.Data, msgid2idx_lut_ + pkt.msg_id,
      sizeof(msgid2idx_lut_[pkt.msg_id]));
  ///! fill the data
  if (pkt.size > 0)
    memcpy(send_can_obj_.Data + sizeof(msgid2idx_lut_[pkt.msg_id]),
        pkt.data, pkt.size);

  return send_buffer_->push(send_can_obj_);
}

bool MotorCan::read(Packet& pkt)  {
  if (!connected_ || !recv_buffer_->pop(recv_can_obj_)) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }

  ///! make sure that we can found the MsgId from data.
  unsigned int idx = 0;
  memcpy(&idx, pkt.data, sizeof(idx));
  if (idx2msgid_lut_.end() == idx2msgid_lut_.find(idx)) {
    LOG_ERROR << "Wrong formate for the can obj";
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf(" -> ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)recv_can_obj_.ID, (int)recv_can_obj_.DataLen,
      (int)recv_can_obj_.Data[0], (int)recv_can_obj_.Data[1],
      (int)recv_can_obj_.Data[2], (int)recv_can_obj_.Data[3],
      (int)recv_can_obj_.Data[4], (int)recv_can_obj_.Data[5],
      (int)recv_can_obj_.Data[6], (int)recv_can_obj_.Data[7]);

    return false;
  }

  pkt.bus_id  = bus_id_;
  pkt.node_id = recv_can_obj_.ID - ID_BASE;
  pkt.msg_id  = idx2msgid_lut_[idx];
  pkt.size    = recv_can_obj_.DataLen - sizeof(idx);
  memset(pkt.data, '\0', 8 * sizeof(char));
  if (pkt.size > 0)
    memcpy(pkt.data, recv_can_obj_.Data + sizeof(idx), pkt.size * sizeof(char));

  return true;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorCan, Label)
