/*
 * usb.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#include "system/platform/propagate/usb.h"
#include "foundation/cfg_reader.h"

#include <fcntl.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>

namespace middleware {

int       g_counter     = 0;
const int MAX_TRY_TIMES = 10;

const int BAUD_DEF[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300};
const int BAUD_MAP[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300};

void __set_speed(int fd, int speed){
  int   status;
  struct termios Opt;
  tcgetattr(fd, &Opt);

  int idx = 0;
  for (; idx < sizeof(BAUD_MAP)/sizeof(int); ++idx) {
    if (speed == BAUD_MAP[idx]) break;
  }
  if (sizeof(BAUD_MAP)/sizeof(int) == idx) {
    LOG_ERROR << "The error baud settings";
    return;
  }

  tcflush(fd, TCIOFLUSH);
  cfsetispeed(&Opt, BAUD_DEF[idx]);
  cfsetospeed(&Opt, BAUD_DEF[idx]);
  status = tcsetattr(fd, TCSANOW, &Opt);
  if  (0 != status) {
    LOG_ERROR << ("baud setting fail！");
  }

  tcflush(fd,TCIOFLUSH);
}

void __set_others(int fd, const UsbPropagate::UsbConfig& cfg) {
  struct termios options;
  if  ( tcgetattr(fd, &options)  !=  0) {
    LOG_ERROR << ("SetupSerial 1");
    return;
  }
  options.c_cflag &= ~CSIZE;
  switch (cfg.data_bit)
  {
  case 7:
    options.c_cflag |= CS7;
    break;
  case 8:
    options.c_cflag |= CS8;
    break;
  default:
    LOG_WARNING << "Unsupported data size, using the default value: 8";
    options.c_cflag |= CS8;
    break;
  }

  if (0 == cfg.parity.compare("none")) {
    options.c_cflag &= ~PARENB;   /* Clear parity enable */
    options.c_iflag &= ~INPCK;     /* Enable parity checking */
  } else if (0 == cfg.parity.compare("odd")) {
    options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
    options.c_iflag |= INPCK;             /* Disnable parity checking */
  } else if (0 == cfg.parity.compare("even")) {
    options.c_cflag |= PARENB;     /* Enable parity */
    options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
    options.c_iflag |= INPCK;       /* Disnable parity checking */
  } else if (0 == cfg.parity.compare("space")) {
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
  } else {
    LOG_WARNING << "Unsupported parity, using the default value none.";
    options.c_cflag &= ~PARENB;   /* Clear parity enable */
    options.c_iflag &= ~INPCK;     /* Enable parity checking */
  }

  /* 设置停止位*/
  switch (cfg.stop_bit) {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
      LOG_WARNING << "Unsupported stop bits, using the default value 1";
      options.c_cflag &= ~CSTOPB;
      break;
  }
  /* Set input parity option */
  if (0 != cfg.parity.compare("none"))
    options.c_iflag |= INPCK;

  tcflush(fd, TCIFLUSH);
  options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
  options.c_cc[VMIN] = 0;    /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0) {
    LOG_ERROR << ("SetupSerial 3");
  }

  options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
  options.c_oflag  &= ~OPOST;   /*Output*/
}

void __config_usb(int fd, const UsbPropagate::UsbConfig& cfg) {
  if (fd < 0) return;
  LOG_INFO << "Ready to configure the USB.";
  __set_speed (fd, cfg.baud_rate);
  __set_others(fd, cfg);
}

UsbPropagate::UsbPropagate(const std::string& l)
: Propagate(l), opened_(false),
  usb_fd_(-1), node_id_(0xFF) {
  ; // Nothing to do here.
}

bool UsbPropagate::auto_init() {
  if (!Propagate::auto_init()) return false;

  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "channel",  usb_config_.file_name);
  cfg->get_value(getLabel(), "baud",     usb_config_.baud_rate);
  cfg->get_value(getLabel(), "parity",   usb_config_.parity);
  cfg->get_value(getLabel(), "stop_bit", usb_config_.stop_bit);
  cfg->get_value(getLabel(), "data_bit", usb_config_.data_bit);

  cfg->get_value(getLabel(), "node_id", node_id_);
  return true;
}

UsbPropagate::~UsbPropagate() {
  stop();
}

bool UsbPropagate::start() {
  opened_ = false;
  // try to 10 times
  for (g_counter = 0; g_counter < MAX_TRY_TIMES; ++g_counter) {
    usb_fd_ = open(usb_config_.file_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    opened_ = (-1 != usb_fd_);
    if (!opened_) {
      LOG_DEBUG << "(" << g_counter + 1 << "/" << MAX_TRY_TIMES
          << ") Initialize USB FAIL, error code: " << errno << "("
          << strerror(errno) << "), Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      __config_usb(usb_fd_, usb_config_);
      LOG_DEBUG << "Initialize USB OK!";
      return opened_;
    }
  }

  LOG_ERROR << "Initialize USB FAIL!!!";
  return opened_;
}

void UsbPropagate::stop() {
  if ((-1 == usb_fd_) || !opened_) return;

  // try to 10 times
  for (g_counter = 0; g_counter < MAX_TRY_TIMES; ++g_counter) {
    int err = close(usb_fd_);
    if (-1 != err){
      LOG_DEBUG << "(" << g_counter + 1 << "/" << MAX_TRY_TIMES
          << ") Stopping USB FAIL, error code: " << errno << "("
          << strerror(errno) << "), Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      usb_fd_ = -1;
      opened_ = false;
      LOG_INFO << "Stopping USB OK!";
      return;
    }
  }

  LOG_ERROR << "Stopping USB FAIL!!!";
}

} /* namespace middleware */
