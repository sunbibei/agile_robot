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
}

bool GzPropagateP::write(const Packet& _pkg) {
  MsgqPacket _msg;
  _msg.msg_id = 0x01;
  _msg.pkg    = _pkg;

  return msgq_->write_to_msgq(cmd_msgq_name_, &_msg, sizeof(_msg));
}

bool GzPropagateP::read (Packet& _pkg) {
  MsgqPacket _msg;
  if (!msgq_->read_from_msgq(leg_node_msgq_name_, &_msg, sizeof(_msg)))
    return false;

  _pkg = _msg.pkg;
  return true;
}

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::GzPropagateP, Label)
