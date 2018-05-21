/*
 * can_usb.cpp
 *
 *  Created on: Apr 26, 2018
 *      Author: bibei
 */

#include <platform/propagate/can_usb.h>

#include "foundation/thread/threadpool.h"
#include "foundation/cfg_reader.h"
#include "foundation/utf.h"

namespace agile_robot {

static bool  g_is_startup_device = false;
static DWORD g_device_type       = VCI_USBCAN2;
static DWORD g_device_idx        = 0;

#define MAX_TRY_TIMES     (10)

CanUsb::CanUsb()
  : Propagate("CanUsb"),
    recv_buffer_(nullptr), send_buffer_(nullptr),
    connected_(false),   recv_buf_size_(2048),
    recv_msgs_(nullptr), send_msgs_(nullptr) {
  ;
}

bool CanUsb::auto_init() {
  if (!Propagate::auto_init()) return false;
  // TODO
  auto cfg = MiiCfgReader::instance();

  // cfg->get_value(getLabel(), "device_type", g_device_type);
  // cfg->get_value(getLabel(), "device_idx",  g_device_idx);

  config_.AccCode = 0;
  config_.AccMask = 0xFFFFFFFF;
  config_.Filter  = 1;
  config_.Timing0 = 0x00;
  config_.Timing1 = 0x1C;
  config_.Mode    = 0;

  ///! Setting the size of receive buffer
  int N_SWAP_BUF = 2048;
  cfg->get_value(getLabel(), "recv_buf_size", N_SWAP_BUF);
  recv_msgs_     = new VCI_CAN_OBJ[N_SWAP_BUF];
  recv_buf_size_ = N_SWAP_BUF;

  ///! Setting the size of sending buffer.
  N_SWAP_BUF = 2048;
  cfg->get_value(getLabel(), "send_buf_size", N_SWAP_BUF);
  send_msgs_ = new VCI_CAN_OBJ[N_SWAP_BUF];

  ///! Setting the size of swap buffer
  N_SWAP_BUF = 2048;
  cfg->get_value(getLabel(), "swap_buf_size", N_SWAP_BUF);
  recv_buffer_ = new boost::lockfree::queue<VCI_CAN_OBJ>(N_SWAP_BUF);
  send_buffer_ = new boost::lockfree::queue<VCI_CAN_OBJ>(N_SWAP_BUF);
  return true;
}

CanUsb::~CanUsb() {
//  int count = 0;
//  VCI_CAN_OBJ tmp;
//  while (!send_buffer_->empty())
//    send_buffer_->pop(tmp);
//  LOG_WARNING << "Send Buffer Residue: " << count;
//
//  count = 0;
//  while (!recv_buffer_->empty())
//    recv_buffer_->pop(tmp);
//  LOG_WARNING << "Recv Buffer Residue: " << count;

  delete recv_msgs_;
  delete send_msgs_;
  delete recv_buffer_;
  delete send_buffer_;
}

bool CanUsb::start() {
  // try to 10 times
  for (int i = 0; i < MAX_TRY_TIMES; ++i) {
    connected_ = true; // the default value is true.
    // LOG_WARNING << "Try to open the usb_can device.";
    if (!g_is_startup_device &&
        STATUS_OK != VCI_OpenDevice(g_device_type, g_device_idx, 0)) {
      LOG_INFO << "Initialize CAN FAIL!!! No such " << g_device_type
          << " or such bus " << g_device_idx;
      connected_          = false;
    }
    g_is_startup_device = true;
    // LOG_WARNING << "Open the usb_can device successful.";

    LOG_INFO << "Try to initialize the can bus " << (int)bus_id_;
    if (connected_ &&
        STATUS_OK != VCI_InitCAN(g_device_type, g_device_idx, bus_id_, &config_)) {
      LOG_INFO << "Initialize the CAN Configure fail!";
//      VCI_CloseDevice(g_device_type, g_device_idx);
//      g_is_startup_device = false;
      connected_          = false;
    }
    // LOG_WARNING << "Initialize the can bus successful!";

    // LOG_WARNING << "Try to start the can bus " << (int)bus_id_;
    if(connected_ &&
        STATUS_OK != VCI_StartCAN(g_device_type, g_device_idx, bus_id_)) {
      LOG_INFO << "Starting CAN error";
//      VCI_CloseDevice(VCI_USBCAN2,0);
//      g_is_startup_device = false;
      connected_          = false;
    }
    // LOG_WARNING << "Start the can bus successful!";

    if (!connected_) {
      LOG_DEBUG << "(" << i + 1 << "/" << MAX_TRY_TIMES
          << ") Initialize CAN FAIL, Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_INFO << "Initialize CAN OK!";

      ///! It will be launch two thread by each Bus inherit from CanUsb
      ThreadPool::instance()->add(propa_name_ + "-r",
          &CanUsb::do_exchange_r, this);
      ThreadPool::instance()->add(propa_name_ + "-w",
          &CanUsb::do_exchange_w, this);

      return connected_;
    }
  }

  LOG_ERROR << "Initialize CAN FAIL!!!";
  return false;
}

void CanUsb::stop()  {
  VCI_CloseDevice(g_device_type, g_device_idx);
  g_is_startup_device = false;
  connected_          = false;
}

//bool CanUsb::write(const Packet& pkt) {
//  if (!connected_) {
//    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
//    return false;
//  }
//
//  return send_buffer_->push(pkt);
//}
//
//bool CanUsb::read(Packet& pkt)  {
//  if (!connected_ || recv_buffer_->empty()) {
//    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
//    return false;
//  }
//
//  return recv_buffer_->pop(pkt);
//}

void CanUsb::do_exchange_w() {
  TIMER_INIT

  int    __n_msg    = 0;
  while (connected_) {
    if (!send_buffer_->empty()) {
      __n_msg    = 0;
      send_buffer_->consume_all( [&] (const VCI_CAN_OBJ& pkt) {
        memcpy(send_msgs_ + __n_msg, &pkt, sizeof(VCI_CAN_OBJ));
//        send_msgs_[__n_msg].ID      = MII_MSG_FILL_2NODE_MSG(pkt.node_id, pkt.msg_id);
//        send_msgs_[__n_msg].DataLen = pkt.size;
//        memcpy(send_msgs_[__n_msg].Data, pkt.data, pkt.size * sizeof(pkt.data[0]));
        ++__n_msg;
      });

      if (false) {
        printf("%s [%03d]: \n", std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1).c_str(), __LINE__);
        for (int i = 0; i < __n_msg; ++i) {
          printf(" <- [%d] ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
            i, (int)send_msgs_[i].ID, (int)send_msgs_[i].DataLen,
            (int)send_msgs_[i].Data[0], (int)send_msgs_[i].Data[1],
            (int)send_msgs_[i].Data[2], (int)send_msgs_[i].Data[3],
            (int)send_msgs_[i].Data[4], (int)send_msgs_[i].Data[5],
            (int)send_msgs_[i].Data[6], (int)send_msgs_[i].Data[7]);
        }
        printf("\n");
      }

      if (VCI_Transmit(g_device_type, g_device_idx, bus_id_, send_msgs_, __n_msg) < 0) {
        LOG_ERROR << "Write CAN FAIL!!!";
      }
    }

    TIMER_CONTROL(5)
  }
}

void CanUsb::do_exchange_r() {
  TIMER_INIT

  // TimeControl timer;
  int recv_off  = 0;
  int recv_size = 0;
  while (connected_) {
    recv_size = VCI_Receive(g_device_type, g_device_idx, bus_id_,
        recv_msgs_, recv_buf_size_, 0);
    // LOG_WARNING<< "bus_id=" << (int)bus_id_ << " -- [ " << recv_size
    //     << " / " << recv_buf_size_ << " ]";
    recv_off  = 0;
    while (recv_off < recv_size) {

      if (false) {
        printf("%s [%03d]: ", std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1).c_str(), __LINE__);
        printf(" -> ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)recv_msgs_[recv_off].ID, (int)recv_msgs_[recv_off].DataLen,
          (int)recv_msgs_[recv_off].Data[0], (int)recv_msgs_[recv_off].Data[1],
          (int)recv_msgs_[recv_off].Data[2], (int)recv_msgs_[recv_off].Data[3],
          (int)recv_msgs_[recv_off].Data[4], (int)recv_msgs_[recv_off].Data[5],
          (int)recv_msgs_[recv_off].Data[6], (int)recv_msgs_[recv_off].Data[7]);
      }

      recv_buffer_->push(recv_msgs_[recv_off]);
      ++recv_off;
    } // end while (recv_off < recv_size)

    TIMER_CONTROL(10)
  }
}

} /* namespace agile_robot */

//#include <class_loader/class_loader_register_macro.h>
//CLASS_LOADER_REGISTER_CLASS(agile_robot::CanUsb, Label)
