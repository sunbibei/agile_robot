/*
 * leg_can.cpp
 *
 *  Created on: May 16, 2018
 *      Author: bibei
 */

#include <platform/propagate/leg_can.h>
#include <iostream>

namespace agile_robot {

LegCan::LegCan()
  : CanUsb() {
  ; // Nothing to do here.
}

LegCan::~LegCan() {
  ; // Nothing to do here.
}

bool LegCan::write(const Packet& pkt) {
  if (!connected_ || pkt.bus_id != bus_id_) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }

  send_can_obj_.ID      = MII_MSG_FILL_2NODE_MSG(pkt.node_id, pkt.msg_id);
  send_can_obj_.DataLen = pkt.size;
  memcpy(send_can_obj_.Data, pkt.data, pkt.size * sizeof(pkt.data[0]));

  return send_buffer_->push(send_can_obj_);
}

bool LegCan::read(Packet& pkt)  {
  if (!connected_ || !recv_buffer_->pop(recv_can_obj_)) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }

  if (!MII_MSG_IS_2HOST(recv_can_obj_.ID)) {
    LOG_WARNING << "Error Message Id format! ";
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
  pkt.node_id = MII_MSG_SPLIT_NODEID(recv_can_obj_.ID);
  pkt.msg_id  = MII_MSG_SPLIT_MSGID(recv_can_obj_.ID);
  pkt.size    = recv_can_obj_.DataLen;
  memset(pkt.data, '\0', 8 * sizeof(char));
  memcpy(pkt.data, recv_can_obj_.Data, pkt.size * sizeof(char));
  return true;
}

} /* namespace agile_robot */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::LegCan, Label)
