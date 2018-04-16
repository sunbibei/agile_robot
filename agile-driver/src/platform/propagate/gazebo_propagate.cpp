/*
 * gazebo_propagate.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: bibei
 */

#include <platform/propagate/gazebo_propagate.h>
#include <foundation/ipc/shared_mem.h>
#include <foundation/ipc/msg_queue.h>
#include <foundation/cfg_reader.h>

#include <chrono>

namespace agile_robot {

struct MsgqPacket : public MsgBase {
  Packet  pkg;
};

GzPropagateP::GzPropagateP()
  : Propagate("gz"), ipc_(nullptr) {
#ifdef USE_SHM
  shm_r_buf_ = nullptr;
  shm_w_buf_ = nullptr;
  start_pos_ = 0;
#endif
  LOG_WARNING << "Starting to construct...";
}

bool GzPropagateP::auto_init() {
  if (!Propagate::auto_init()) return false;

  auto cfg = MiiCfgReader::instance();
  // TODO
  cfg->get_value(getLabel(), "leg_node_ipc_name", leg_node_ipc_name_);
  cfg->get_value(getLabel(), "cmd_ipc_name",      cmd_ipc_name_);
#ifdef USE_SHM
  id_2_leg_lut_.resize(MAX_NODE_NUM);
  unsigned char id = 0;
  FOR_EACH_LEG(l) {
    cfg->get_value(getLabel(), LEGTYPE_TOSTRING(l), id);
    id_2_leg_lut_[id] = l;
  }
#endif
  LOG_WARNING << "Starting to auto_init...";
  return true;
}

GzPropagateP::~GzPropagateP() {
  ipc_->destroy_instance();
  ipc_ = nullptr;
}

bool GzPropagateP::start() {
  ///! The message queue or the shared memory
#ifdef USE_SHM
  ipc_ = SharedMem::create_instance();
  if (nullptr == ipc_) {
    LOG_ERROR << "Could not create the shared memory for gazebo propagate.";
    return false;
  }
  if (!ipc_->create_shm(leg_node_ipc_name_, LegType::N_LEGS*sizeof(Packet))) {
    LOG_ERROR << "Could not create the shared memory named by " << leg_node_ipc_name_;
    return false;
  }
  shm_r_buf_ = ipc_->get_addr_from_shm(leg_node_ipc_name_);

  if (!ipc_->create_shm(cmd_ipc_name_, LegType::N_LEGS*sizeof(Packet))) {
    LOG_ERROR << "Could not create the shared memory named by " << leg_node_ipc_name_;
    return false;
  }
  shm_w_buf_ = ipc_->get_addr_from_shm(cmd_ipc_name_);
#else
  ipc_ = MsgQueue::create_instance();
  if (nullptr == ipc_) {
    LOG_ERROR << "Could not create the message queue for gazebo propagate.";
    return false;
  }
  if (!ipc_->create_msgq(leg_node_ipc_name_)) {
    LOG_ERROR << "Could not create the message queue named by " << leg_node_ipc_name_;
    return false;
  }

  if (!ipc_->create_msgq(cmd_ipc_name_)) {
    LOG_ERROR << "Could not create the message queue named by " << leg_node_ipc_name_;
    return false;
  }
#endif


  return true;
}

void GzPropagateP::stop() {
#ifndef USE_SHM
  ipc_->destroy_msgq(leg_node_ipc_name_);
  ipc_->destroy_msgq(cmd_ipc_name_);
#endif

  ipc_->destroy_instance();
}

bool GzPropagateP::write(const Packet& _pkg) {
#ifdef USE_SHM
  start_pos_ = id_2_leg_lut_[_pkg.node_id] * sizeof(Packet);
  memcpy(shm_w_buf_ + start_pos_, &_pkg, sizeof(Packet));

  if (false) {
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)_pkg.node_id,
      (int)_pkg.msg_id,  (int)_pkg.size,
      (int)_pkg.data[0], (int)_pkg.data[1],
      (int)_pkg.data[2], (int)_pkg.data[3],
      (int)_pkg.data[4], (int)_pkg.data[5],
      (int)_pkg.data[6], (int)_pkg.data[7]);
  }
  return true;
#else
  MsgqPacket _msg;
  _msg.msg_id = 0x01; // means from agile driver
  _msg.pkg    = _pkg;

  bool ret = ipc_->write_to_msgq(cmd_ipc_name_, &_msg, sizeof(_msg));
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
#endif
}

bool GzPropagateP::read (Packet& _pkg) {
#ifdef USE_SHM
  static size_t _start = 0;
  memcpy(&_pkg, shm_r_buf_ + (_start*sizeof(Packet)), sizeof(Packet));
  _start = (_start + 1) % LegType::N_LEGS;

  if (false && 0x02u == _pkg.node_id) {
    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
    printf("  -> NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)_pkg.node_id,
      (int)_pkg.msg_id,  (int)_pkg.size,
      (int)_pkg.data[0], (int)_pkg.data[1],
      (int)_pkg.data[2], (int)_pkg.data[3],
      (int)_pkg.data[4], (int)_pkg.data[5],
      (int)_pkg.data[6], (int)_pkg.data[7]);
  }
  return true;
#else
  MsgqPacket _msg;
  if (!ipc_->read_from_msgq(leg_node_ipc_name_, &_msg, sizeof(_msg)))
    return false;

  if (false && 0x02u == _msg.pkg.node_id) {
    int64_t _ts = std::chrono::time_point_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now()).time_since_epoch().count();
    printf("ts -> %ld - %ld = %ld\n", _ts, _msg.timestamp, _ts - _msg.timestamp);
  }

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
#endif
}

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::GzPropagateP, Label)
