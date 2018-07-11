/*
 * fake_can.cpp
 *
 *  Created on: Jul 11, 2018
 *      Author: bibei
 */

#include <platform/propagate/fake_can.h>

namespace agile_robot {

FakeCan::FakeCan()
  : Propagate("fake_can") {
  ;
}

FakeCan::~FakeCan() {
  ;
}

bool FakeCan::start() {
  return true;
}

void FakeCan::stop()  {
}

bool FakeCan::write(const Packet&) {
  return true;
}

bool FakeCan::read(Packet& pkt) {
  static short _fake_data = 0;

  pkt.bus_id  = ARM_BUS;
  pkt.node_id = 0x02;
  pkt.msg_id  = MII_MSG_HEARTBEAT_1;
  pkt.size    = 8;

  int off = 0;
  FOREACH_JNT(j) {
    memcpy(pkt.data + off, &_fake_data, sizeof(_fake_data));
    off += sizeof(_fake_data);
  }
  memcpy(pkt.data + off, &_fake_data, sizeof(_fake_data));

  ++_fake_data;
  return true;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::FakeCan, Label)
