/*
 * pcan_fake.cpp
 *
 *  Created on: Sep 7, 2017
 *      Author: bibei
 */

#include "system/platform/propagate/pcan_fake.h"
#include "foundation/cfg_reader.h"

#include <stdio.h>
#include <chrono>
#include <ctime>
#include <iomanip>

namespace middleware {

FILE*   g_r_fd   = nullptr;
FILE*   g_w_fd   = nullptr;
bool    g_repeat = false;

#define INVALID_BYTE  (0x88)
#define ONLY_ONE_PKT  (0x11)
#define PACKET_W_FORMAT ("%4d-%02d-%02d, %02d:%02d:%02d\tID: 0x%03X, LEN: %d, DATA: 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n")
#define PACKET_R_FORMAT ("ID: 0x%x, LEN: %d, DATA: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n")

#define PACKAGE_W_PRINT ("\033[33;1m -> %02d:%02d:%02d ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\033[0m\n")
#define PACKAGE_R_PRINT ("\033[35;1m <- %02d:%02d:%02d ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\033[0m\n")

PcanFake::PcanFake(const std::string&)
  : swap_buffer_(nullptr) {
  r_time_t_ = nullptr;
  w_time_t_ = nullptr;
  r_tm_    = nullptr;
  w_tm_    = nullptr;
}
bool PcanFake::auto_init() {
  auto cfg = MiiCfgReader::instance();

  std::string filename;
  if (cfg->get_value(getLabel(), "output", filename))
    g_w_fd = fopen(filename.c_str(), "w");
  if (cfg->get_value(getLabel(), "input",  filename)) {
    g_r_fd = fopen(filename.c_str(), "r");
    cfg->get_value(getLabel(),   "repeat", g_repeat);
  }

  r_time_t_ = new time_t;
  w_time_t_ = new time_t;
//  r_tm_     = new tm;
//  w_tm_     = new tm;

  swap_buffer_ = new boost::lockfree::queue<TPCANMsg>(1024);
  return PcanPropagate::auto_init();
}

PcanFake::~PcanFake() {
  delete r_time_t_;
  delete w_time_t_;
  delete swap_buffer_;
  r_time_t_    = nullptr;
  w_time_t_    = nullptr;
  swap_buffer_ = nullptr;

  if (nullptr != g_r_fd) {
    fclose(g_r_fd);
    g_r_fd = nullptr;
  }
  if (nullptr != g_w_fd) {
    fclose(g_w_fd);
    g_w_fd = nullptr;
  }
}

bool PcanFake::start() {
  LOG_INFO << "PcanChannelFake start. ";
  LOG_INFO << "Input  FD: " << g_r_fd;
  LOG_INFO << "Output FD: " << g_w_fd;
  return true;
}

bool PcanFake::write(const Packet& pkt) {
  // static int g_w_err_count = 0;
  // if (!connected_) { return connected_; }
  // This code aims to compatible with the old protocol
  // TODO It should be updated.
  send_msg_.MSGTYPE = PCAN_MESSAGE_STANDARD;
  send_msg_.ID      = MII_MSG_FILL_TO_NODE_MSG(pkt.node_id, pkt.msg_id);
  send_msg_.LEN     = pkt.size;
  memset(send_msg_.DATA, '\0', 8 * sizeof(BYTE));
  memcpy(send_msg_.DATA, pkt.data, send_msg_.LEN * sizeof(BYTE));

  time(w_time_t_);
  w_tm_ = std::localtime(w_time_t_);

  if (false)
  printf(PACKAGE_W_PRINT,
      w_tm_->tm_hour, w_tm_->tm_min, w_tm_->tm_sec, send_msg_.ID, (int)send_msg_.LEN,
      send_msg_.DATA[0], send_msg_.DATA[1], send_msg_.DATA[2],
      send_msg_.DATA[3], send_msg_.DATA[4], send_msg_.DATA[5],
      send_msg_.DATA[6], send_msg_.DATA[7]);

  if (g_w_fd)
    fprintf(g_w_fd, PACKET_W_FORMAT, w_tm_->tm_year + 1900, w_tm_->tm_mon,
        w_tm_->tm_mday, w_tm_->tm_hour, w_tm_->tm_min, w_tm_->tm_sec,
        send_msg_.ID, (int)send_msg_.LEN, send_msg_.DATA[0],
        send_msg_.DATA[1], send_msg_.DATA[2], send_msg_.DATA[3],
        send_msg_.DATA[4], send_msg_.DATA[5], send_msg_.DATA[6],
        send_msg_.DATA[7]);

  if (MII_MSG_COMMON_DATA_1 == pkt.msg_id) {
    send_msg_.ID  = MII_MSG_FILL_TO_HOST_MSG(pkt.node_id, MII_MSG_HEARTBEAT_MSG_1);
    send_msg_.LEN = 0x08;
    // buf_lock_.lock();
    swap_buffer_->push(send_msg_);
    // swap_buffer_.push_back(send_msg_);
    // buf_lock_.unlock();
  }

  return true;
}

bool PcanFake::read(Packet& pkt) {
  if (g_r_fd) {
    if (feof(g_r_fd)) {
      if (g_repeat) {
        fseek(g_r_fd, 0, SEEK_SET);
      } else {
        fclose(g_r_fd);
        g_r_fd = nullptr;
      }
    }
    fscanf(g_r_fd, PACKET_R_FORMAT,
        &recv_msg_.ID, (int*)&recv_msg_.LEN,
         (int*)(recv_msg_.DATA + 0), (int*)(recv_msg_.DATA + 1),
         (int*)(recv_msg_.DATA + 2), (int*)(recv_msg_.DATA + 3),
         (int*)(recv_msg_.DATA + 4), (int*)(recv_msg_.DATA + 5),
         (int*)(recv_msg_.DATA + 6), (int*)(recv_msg_.DATA + 7));
  } else if (!swap_buffer_->empty()) {
    if (!swap_buffer_->pop(recv_msg_))
      return false;
    // recv_msg_ = swap_buffer_.front();
    // swap_buffer_.pop_front();
  } else {
    return false;
  }

  time(r_time_t_);
  r_tm_ = std::localtime(r_time_t_);
  if (false)
    printf(PACKAGE_R_PRINT,
      r_tm_->tm_hour, r_tm_->tm_min, r_tm_->tm_sec, recv_msg_.ID, (int)recv_msg_.LEN,
      recv_msg_.DATA[0], recv_msg_.DATA[1], recv_msg_.DATA[2], recv_msg_.DATA[3],
      recv_msg_.DATA[4], recv_msg_.DATA[5], recv_msg_.DATA[6], recv_msg_.DATA[7]);

  pkt.bus_id  = bus_id_;
  pkt.node_id = MII_MSG_EXTRACT_NODE_ID(recv_msg_.ID);
  pkt.msg_id  = MII_MSG_EXTRACT_MSG_ID(recv_msg_.ID);
  pkt.size    = recv_msg_.LEN;
  memset(pkt.data, '\0', 8 * sizeof(char));
  memcpy(pkt.data, recv_msg_.DATA, pkt.size * sizeof(char));
  return true;
}

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::PcanFake, Label)
CLASS_LOADER_REGISTER_CLASS(middleware::PcanFake, middleware::Propagate)
