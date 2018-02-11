/*
 * imu_usb.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#include "system/platform/propagate/imu_usb_jy901.h"
#include "foundation/cfg_reader.h"

namespace middleware {

const int MAX_BUF_SIZE  = 1024;

ImuJY901::ImuJY901(const MiiString& l)
  : UsbPropagate(l), read_status_(-1), read_buf_(new char[MAX_BUF_SIZE]),
    buf_top_(read_buf_), buf_btm_(read_buf_),
    BUF_EOF_(read_buf_ + MAX_BUF_SIZE) {
  ;
}

ImuJY901::~ImuJY901() {
  stop();
  if (nullptr != read_buf_) {
    delete read_buf_;
    read_buf_ = nullptr;
  }
}

///! The helper method calculates the sum byte for up message.
inline bool __check_sum(const char* data) {
  char sum = data[0];
  // every message contains 11 bytes.
  const char* DATA_EOF = data + USB_UP_MESSAGE_SIZE - 1;
  while (++data != DATA_EOF) sum += *data;
  return (*data == sum);
}

bool ImuJY901::read(Packet& pkt) {
  if (!opened_) {
    // LOG_FIRST_N(WARNING, 10000) << "The USB has not been launched, or initialized fail.";
    return false;
  }
  pkt.bus_id  = bus_id_;
  pkt.node_id = node_id_;

  while (true) { // loop until parse an message or read error!
    char* offset = buf_btm_;
    while ((buf_top_ - offset) > USB_UP_MESSAGE_SIZE) {
      if ( (MII_USB_UP_HEADER != *offset++)
          || (*offset < MII_USB_UP_ID_TIME)
          || (*offset > MII_USB_UP_ID_ACCURACY)
          || (!__check_sum(offset - 1)) ) {
        // LOG_WARNING << "The USB message has error!";
        continue;
      }

      pkt.msg_id = *offset++;
      pkt.size   = USB_UP_MESSAGE_DATA_SIZE;
      memcpy(pkt.data, offset, USB_UP_MESSAGE_DATA_SIZE);
      // one byte sum and then is the new data, so plus one
      offset  += (USB_UP_MESSAGE_DATA_SIZE + 1);
      buf_btm_ = offset;
      // printf("1. 0x%02X : 0x%02X : 0x%02X : 0x%02X\n", read_buf_, buf_btm_, buf_top_, BUF_EOF_);
      return true;
    } // end while (offset < buf_top_)

    // buf_btm_ = offset;
    // printf("2. 0x%02X 0x%02X 0x%02X 0x%02X\n", read_buf_, buf_btm_, buf_top_, BUF_EOF_);
    if ((BUF_EOF_ - buf_top_) < 2*USB_UP_MESSAGE_SIZE) {
      int __size = buf_top_ - buf_btm_;
      memcpy(read_buf_, buf_btm_, __size);
      buf_btm_ = read_buf_;
      buf_top_ = read_buf_ + __size;
    }

    read_status_ = ::read(usb_fd_, buf_top_, BUF_EOF_ - buf_top_);
    if (read_status_ <= 0) return false;
    buf_top_ += read_status_;
    // printf("3: %d, 0x%02X vs 0x%02X\n", read_status_, buf_btm_, buf_top_);
  } // end while(true)
}

bool ImuJY901::write(const Packet&) { return false; }

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::ImuJY901, Label)
CLASS_LOADER_REGISTER_CLASS(middleware::ImuJY901, middleware::Propagate)
