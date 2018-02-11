/*
 * imu_usb_jy61p.cpp
 *
 *  Created on: Dec 26, 2017
 *      Author: robot
 */

#include "system/platform/propagate/imu_usb_jy61p.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

namespace middleware {

const int MAX_BUF_SIZE  = 1024;

ImuJY61P::ImuJY61P(const MiiString& l)
: UsbPropagate(l), read_status_(-1), read_buf_(new char[MAX_BUF_SIZE]),
  buf_top_(read_buf_), buf_btm_(read_buf_),
  BUF_EOF_(read_buf_ + MAX_BUF_SIZE) {
;
}

ImuJY61P::~ImuJY61P() {
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
  printf("SUM: 0x%02X - 0x%02X\n", sum, *data);
  return (*data == sum);
}

bool ImuJY61P::read(Packet& pkt) {
  if (!opened_) {
    // LOG_FIRST_N(WARNING, 10000) << "The USB has not been launched, or initialized fail.";
    return false;
  }
  if ((BUF_EOF_ - buf_top_) < 2*USB_UP_MESSAGE_SIZE) {
    int __size = buf_top_ - buf_btm_;
    memcpy(read_buf_, buf_btm_, __size);
    buf_btm_ = read_buf_;
    buf_top_ = read_buf_ + __size;
  }

  read_status_ = ::read(usb_fd_, buf_top_, BUF_EOF_ - buf_top_);
  if (read_status_ > 0) buf_top_ += read_status_;
  else return false;

  if (false) {
    char* offset1 = buf_btm_;
    printf("read: %d\n", read_status_);
    while (offset1 != buf_top_) {
      printf("%02X ", *offset1++);
    }
    printf("\n");
  }

  char* offset = buf_btm_;
  while ((buf_top_ - offset) > USB_UP_MESSAGE_SIZE) {
    if (MII_USB_UP_HEADER != *offset++) continue;

    if ( (*offset < MII_USB_UP_ID_TIME)
        || (*offset > MII_USB_UP_ID_ACCURACY) ) {
      LOG_WARNING << "Message id is wrong!";
      continue;
    }
    if (!__check_sum(offset - 1)) {
      buf_btm_ = offset + USB_UP_MESSAGE_SIZE;
      LOG_WARNING << "The message has error sum!";
      printf("0x%02X\n", *buf_btm_);
      continue;
    } else {
      LOG_WARNING << "The message has correct sum!";
    }

    // printf("2. 0x%02X 0x%02X 0x%02X 0x%02X\n", read_buf_, buf_btm_, buf_top_, BUF_EOF_);
    pkt.bus_id  = bus_id_;
    pkt.node_id = node_id_;
    pkt.msg_id  = *offset++;
    pkt.size    = USB_UP_MESSAGE_DATA_SIZE;
    memcpy(pkt.data, offset, USB_UP_MESSAGE_DATA_SIZE);
    // one byte sum and then is the new data, so plus one
    offset  += (USB_UP_MESSAGE_DATA_SIZE + 1);
    buf_btm_ = offset;
    return true;
  } // end while (offset < buf_top_)

  return false;
  // buf_btm_ = offset;

    // printf("3: %d, 0x%02X vs 0x%02X\n", read_status_, buf_btm_, buf_top_);
}

bool ImuJY61P::write(const Packet&) { return false; }

} /* namespace middleware */


#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::ImuJY61P, Label)
CLASS_LOADER_REGISTER_CLASS(middleware::ImuJY61P, middleware::Propagate)
