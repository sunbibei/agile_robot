/*
 * fake_can.cpp
 *
 *  Created on: Jul 11, 2018
 *      Author: bibei
 */

#include "platform/propagate/fake_can.h"
#include "foundation/cfg_reader.h"

#include <boost/algorithm/string.hpp>

namespace agile_robot {

FakeCan::FakeCan()
  : Propagate("fake_can"), output_("null") {
  ;
}

bool FakeCan::auto_init() {
  if (!Propagate::auto_init()) return false;

  auto cfg = CfgReader::instance();
  cfg->get_value(getLabel(), "output", output_);
  boost::to_lower(output_);

  cfg->foreachTag(Label::make_label(getLabel(), "from"), [&](const std::string& tag){
    Packet pkt = {bus_id_, 0x02, 0x01, 8, {0}};
    cfg->get_value(tag, "node_id", pkt.node_id);
    cfg->get_value(tag, "msg_id",  pkt.msg_id);
    cfg->get_value(tag, "size",    pkt.size);

    pkt_from_.push_back(pkt);
  });

  return true;
}

FakeCan::~FakeCan() {
  ;
}

bool FakeCan::start() {
  return true;
}

void FakeCan::stop()  {
}

bool FakeCan::write(const Packet& pkt) {
  if (0 == output_.compare("screen")) {
    printf(" fake_can.cpp:%03d  - NODE ID:0x%02X MSG ID: 0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          __LINE__, (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
          (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
          (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
  }
  return true;
}

bool FakeCan::read(Packet& pkt) {
  if (pkt_from_.empty()) return false;

  static short _s_pkt_idx   = 0;
  static short _s_fake_data = 0;

  pkt = pkt_from_[_s_pkt_idx%pkt_from_.size()];
  memcpy(pkt.data + 0, &_s_fake_data, sizeof(_s_fake_data));
  memcpy(pkt.data + 2, &_s_fake_data, sizeof(_s_fake_data));
  memcpy(pkt.data + 4, &_s_fake_data, sizeof(_s_fake_data));
  memcpy(pkt.data + 6, &_s_fake_data, sizeof(_s_fake_data));

  ++_s_fake_data;
  ++_s_pkt_idx;
  return true;
}

} /* namespace agile_robot */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::FakeCan, Label)
