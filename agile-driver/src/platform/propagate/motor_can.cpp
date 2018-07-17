/*
 * motor_can.cpp
 *
 *  Created on: May 16, 2018
 *      Author: bibei
 */

#include <platform/propagate/motor_can.h>

#include "foundation/thread/threadpool.h"
#include "foundation/cfg_reader.h"

#ifdef SAVE_CMD_TIME
#include "toolbox/time_control.h"
#include <fstream>
#endif

#include <iostream>

namespace agile_robot {

unsigned int MotorCan::s_sendid_base_ = 0x300u;
unsigned int MotorCan::s_recvid_base_ = 0x280u;
VCI_CAN_OBJ  MotorCan::s_enable_can_;
//VCI_CAN_OBJ  MotorCan::s_enable_can_  = {
//    .ID      = 0x00,
//    .DataLen = 2,
//    .Data    = {0x01, 0x00}
//};

MotorCan::MotorCan()
  : CanUsb(), enable_proxy_(false) {
  ///! setting the default value.
//  for (auto& key : msgid2idx_lut_)
//    key = INVALID_BYTE;
  memset(&send_can_obj_, 0x00, sizeof(send_can_obj_));
  memset(&recv_can_obj_, 0x00, sizeof(recv_can_obj_));
  memset(&s_enable_can_, 0x00, sizeof(s_enable_can_));
}

bool MotorCan::auto_init() {
  if (!CanUsb::auto_init()) return false;

//  ///! For read the current of motor
//  // msgid2idx_lut_[MII_MSG_HEARTBEAT_1] = 0x00005149;
//  msgid2idx_lut_[MII_MSG_HEARTBEAT_1] = 0x80005149;
//  idx2msgid_lut_[0x80005149]          = MII_MSG_HEARTBEAT_1;
//  ///! For read the velocity of motor
//  msgid2idx_lut_[MII_MSG_HEARTBEAT_2] = 0x80005856;
//  idx2msgid_lut_[0x00005856]          = MII_MSG_HEARTBEAT_2;
//  ///! For read the position of motor
//  msgid2idx_lut_[MII_MSG_HEARTBEAT_3] = 0x80005850;
//  idx2msgid_lut_[0x00005850]          = MII_MSG_HEARTBEAT_3;
//
//  ///! For control the current of motor
//  msgid2idx_lut_[MII_MSG_MOTOR_1] = 0x00004354;
//  idx2msgid_lut_[0x00004354]      = MII_MSG_MOTOR_1;
//  ///! For control the velocity of motor
//  msgid2idx_lut_[MII_MSG_MOTOR_2] = 0x0000564A;
//  idx2msgid_lut_[0x0000564A]      = MII_MSG_MOTOR_2;
//  ///! For control the position of motor
//  msgid2idx_lut_[MII_MSG_MOTOR_3] = 0x00005250;
//  idx2msgid_lut_[0x00005250]      = MII_MSG_MOTOR_3;
//  ///! For enable the motor
//  msgid2idx_lut_[MII_MSG_MOTOR_4] = 0x00004742;
//  idx2msgid_lut_[0x00004742]      = MII_MSG_MOTOR_4;
//  ///! For change the mode of motor
//  msgid2idx_lut_[MII_MSG_MOTOR_4] = 0x00005453;
//  idx2msgid_lut_[0x00005453]      = MII_MSG_MOTOR_4;
//
//  ///! For enable the motor
//  msgid2idx_lut_[MII_MSG_COMMON_1] = 0x00004F4D;
//  idx2msgid_lut_[0x00004F4D]       = MII_MSG_COMMON_1;
//  ///! For change the mode of motor
//  msgid2idx_lut_[MII_MSG_COMMON_2] = 0x00004D55;
//  idx2msgid_lut_[0x00004D55]       = MII_MSG_COMMON_2;

  s_sendid_base_ = 0x300u;
  s_recvid_base_ = 0x280u;
  s_enable_can_.ID      = 0x00;
  s_enable_can_.DataLen = 2;
  s_enable_can_.Data[0] = 0x01;
  s_enable_can_.Data[1] = 0x00;

  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "proxy",      enable_proxy_);
  cfg->get_value(getLabel(), "motor_list", proxy_list_);

#ifdef SAVE_CMD_TIME
  try_len     = 100;
  objs_timer_ = new TimeControl(true);
  send_objs_.reserve(try_len);
  recv_objs_.reserve(try_len);
#endif

  return true;
}

MotorCan::~MotorCan() {
  ; // Nothing to do here.
}

bool MotorCan::start() {
  if (!CanUsb::start()) return false;

  ///! enable the CAN of motor
  send_buffer_->push(s_enable_can_);
  if (enable_proxy_)
    ThreadPool::instance()->add("motor_state_ticker", &MotorCan::state_tick, this);

  return true;
}

bool MotorCan::write(const Packet& pkt) {
  if (!connected_ || pkt.bus_id != bus_id_) {
    LOG_WARNING << "The pcan has not been launched, or initialized fail.";
    return false;
  }
#ifdef SAVE_CMD_TIME
  return true;
#endif
//  send_can_obj_.ID = s_sendid_base_ + pkt.node_id;
//  if (INVALID_BYTE == msgid2idx_lut_[pkt.msg_id]) {
//    LOG_ERROR << "Wrong format Packet";
//    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
//    printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//      (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
//      (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
//      (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
//
//    ///! Something is wrong, try to whether handle this wrong in the base class.
//    return CanUsb::write(pkt);
//  }
//
//  memcpy(send_can_obj_.Data, msgid2idx_lut_ + pkt.msg_id,
//      sizeof(msgid2idx_lut_[pkt.msg_id]));
  ///! fill the data
//  if (pkt.size > 0)969
//    memcpy(send_can_obj_.Data + sizeof(msgid2idx_lut_[pkt.msg_id]),
//        pkt.data, pkt.size);
  if(pkt.data[0] == 0x01){
    send_can_obj_.ID      = 0x0000;
  }else{
    send_can_obj_.ID      = s_sendid_base_ + pkt.node_id;
  }
  send_can_obj_.DataLen = pkt.size;
  memcpy(send_can_obj_.Data, pkt.data, pkt.size);
  return send_buffer_->push(send_can_obj_);
}

bool MotorCan::read(Packet& pkt)  {
  if (!connected_ || !recv_buffer_->pop(recv_can_obj_)) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }

#ifdef SAVE_CMD_TIME
  static int tmp = 0;
  ++tmp;
  if (tmp > 2) {
    recv_objs_.push_back(objs_timer_->span());
    LOG_INFO << "recv: " << recv_objs_.size();

    if (try_len == recv_objs_.size()) {
      LOG_INFO << "Write the package...";

      std::ofstream ofd("/home/robot/ControlData/cmds");
      for (int i = 0; i < try_len; ++i) {
        ofd << send_objs_[i] << " " << recv_objs_[i] << std::endl;
      }
      ofd.close();
    }
  }

#endif

  ///! make sure that we can found the MsgId from data.
//  unsigned int idx = 0;
//  memcpy(&idx, recv_can_obj_.Data, sizeof(idx));
//  if (idx2msgid_lut_.end() == idx2msgid_lut_.find(idx)) {
//    LOG_ERROR << "Wrong formate for the can obj";
//    std::cout << std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) << ":";
//    printf(" -> ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
//      (int)recv_can_obj_.ID, (int)recv_can_obj_.DataLen,
//      (int)recv_can_obj_.Data[0], (int)recv_can_obj_.Data[1],
//      (int)recv_can_obj_.Data[2], (int)recv_can_obj_.Data[3],
//      (int)recv_can_obj_.Data[4], (int)recv_can_obj_.Data[5],
//      (int)recv_can_obj_.Data[6], (int)recv_can_obj_.Data[7]);
//
//    return false;
//  }

//  pkt.bus_id  = bus_id_;
//  pkt.node_id = recv_can_obj_.ID - s_recvid_base_;
//  pkt.msg_id  = idx2msgid_lut_[idx];
//  pkt.size    = recv_can_obj_.DataLen - sizeof(idx);
//  memset(pkt.data, '\0', 8 * sizeof(char));
//  if (pkt.size > 0)
//    memcpy(pkt.data, recv_can_obj_.Data + sizeof(idx), pkt.size * sizeof(char));
  pkt.bus_id  = bus_id_;
  pkt.node_id = recv_can_obj_.ID - s_recvid_base_;
  pkt.msg_id  = MII_MSG_MOTOR_RESERVE;
  pkt.size    = recv_can_obj_.DataLen;
  memcpy(pkt.data, recv_can_obj_.Data, recv_can_obj_.DataLen);
  return true;
}

void MotorCan::state_tick() {
  TICKER_INIT(std::chrono::milliseconds);

  VCI_CAN_OBJ req_p, req_dp, req_ddp;
  memset(&req_ddp, 0x00, sizeof(req_ddp));
  req_ddp.DataLen = 4;
  req_ddp.Data[0] = 0x49; req_ddp.Data[1] = 0x51;
  req_ddp.Data[2] = 0x00; req_ddp.Data[3] = 0x80;

  memset(&req_dp, 0x00, sizeof(req_dp));
  req_dp.DataLen = 4;
  req_dp.Data[0] = 0x56; req_dp.Data[1] = 0x58;
  req_dp.Data[2] = 0x00; req_dp.Data[3] = 0x00;

  memset(&req_p, 0x00, sizeof(req_p));
  req_p.DataLen = 4;
  req_p.Data[0] = 0x50; req_p.Data[1] = 0x58;
  req_p.Data[2] = 0x00; req_p.Data[3] = 0x00;

#ifdef SAVE_CMD_TIME
  req_p.ID   = s_sendid_base_ + proxy_list_[0];

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  LOG_INFO << "Start to send objects...";
  for (int i = 0; i < try_len; ++i) {
    send_buffer_->push(req_p);

    send_objs_.push_back(objs_timer_->span());

    LOG_WARNING << "Send: " << send_objs_.size();
    TICKER_CONTROL(1000)
  }

  return;
#endif

  while (connected_) {
    for (const auto& proxy : proxy_list_) {
//      req_ddp.ID = s_sendid_base_ + proxy;
//      send_buffer_->push(req_ddp);

      req_dp.ID  = s_sendid_base_ + proxy;
      send_buffer_->push(req_dp);

//      req_p.ID   = s_sendid_base_ + proxy;
//      send_buffer_->push(req_p);
    }

    TICKER_CONTROL(5, std::chrono::milliseconds);
  }
}

} /* namespace agile_robot */

// #include <class_loader/class_loader_register_macro.h>
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(agile_robot::MotorCan, Label)
