/*
 * gazebo_propagate.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#include <system/platform/propagate/gazebo_propagate.h>
#include <foundation/ipc/msg_queue.h>
#include <foundation/cfg_reader.h>

namespace middleware {

struct MsgqPacket : public MsgBase {
  Packet pkg;
};

GzPropagateP::GzPropagateP()
  : Propagate("gz"), msgq_(nullptr) {
  LOG_WARNING << "Starting to construct...";
}

bool GzPropagateP::auto_init() {
  if (!Propagate::auto_init()) return false;

  auto cfg = MiiCfgReader::instance();
  // TODO
  cfg->get_value(getLabel(), "leg_node_msgq_name", leg_node_msgq_name_);
  cfg->get_value(getLabel(), "cmd_msgq_name",      cmd_msgq_name_);

  LOG_WARNING << "Starting to auto_init...";
  return true;
}

GzPropagateP::~GzPropagateP() {
  MsgQueue::destroy_instance();
  msgq_ = nullptr;
}

bool GzPropagateP::start() {
  msgq_ = MsgQueue::create_instance();
  if (nullptr == msgq_) {
    LOG_ERROR << "Could not create the message queue for gazebo propagate.";
    return false;
  }

  if (!msgq_->create_msgq(leg_node_msgq_name_)) {
    LOG_ERROR << "Could not create the message queue named by " << leg_node_msgq_name_;
    return false;
  }

  if (!msgq_->create_msgq(cmd_msgq_name_)) {
    LOG_ERROR << "Could not create the message queue named by " << leg_node_msgq_name_;
    return false;
  }

  return true;
}

void GzPropagateP::stop() {
  msgq_->destroy_msgq(leg_node_msgq_name_);
  msgq_->destroy_msgq(cmd_msgq_name_);

  MsgQueue::destroy_instance();
}

bool GzPropagateP::write(const Packet& _pkg) {
  MsgqPacket _msg;
  _msg.msg_id = 0x01; // means from agile driver
  _msg.pkg    = _pkg;

  bool ret = msgq_->write_to_msgq(cmd_msgq_name_, &_msg, sizeof(_msg));
  if (false && ret) {
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf("  <- T  O:0x%02X NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)_msg.msg_id,      (int)_msg.pkg.node_id,
      (int)_msg.pkg.msg_id,  (int)_msg.pkg.size,
      (int)_msg.pkg.data[0], (int)_msg.pkg.data[1],
      (int)_msg.pkg.data[2], (int)_msg.pkg.data[3],
      (int)_msg.pkg.data[4], (int)_msg.pkg.data[5],
      (int)_msg.pkg.data[6], (int)_msg.pkg.data[7]);
  }
  return ret;
}

bool GzPropagateP::read (Packet& _pkg) {
  MsgqPacket _msg;
  if (!msgq_->read_from_msgq(leg_node_msgq_name_, &_msg, sizeof(_msg)))
    return false;

  if (false && 0x02u == _msg.pkg.node_id) {
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf("  -> FROM:0x%02X NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)_msg.msg_id,      (int)_msg.pkg.node_id,
      (int)_msg.pkg.msg_id,  (int)_msg.pkg.size,
      (int)_msg.pkg.data[0], (int)_msg.pkg.data[1],
      (int)_msg.pkg.data[2], (int)_msg.pkg.data[3],
      (int)_msg.pkg.data[4], (int)_msg.pkg.data[5],
      (int)_msg.pkg.data[6], (int)_msg.pkg.data[7]);
  }

  // _pkg = _msg.pkg;
  memcpy(&_pkg, &_msg.pkg, sizeof(_pkg));
  return true;
}

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::GzPropagateP, Label)
