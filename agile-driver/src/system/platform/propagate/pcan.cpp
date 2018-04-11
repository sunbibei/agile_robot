/*
 * propagate_imp_pcan.cpp
 *
 *  Created on: Dec 2, 2016
 *      Author: silence
 */

#include "system/platform/propagate/pcan.h"
#include "system/platform/proto/agile_proto.h"
#include "foundation/cfg_reader.h"

namespace agile_robot {

const unsigned int MAX_TRY_TIMES = 10;
unsigned int       g_times_count = 0;

PCanPropa::PCanPropa(const std::string& l)
  : Propagate(l), connected_(false),
    tmp_pcan_status_(PCAN_ERROR_OK) {
  pcan_config_ = {PCAN_USBBUS1, PCAN_BAUD_500K, 0, 0, 0};

}

PCanPropa::~PCanPropa() {
  stop();
  ;//  Nothing need to dealloc
}

bool PCanPropa::auto_init() {
  if (!Propagate::auto_init()) return false;

  auto cfg = MiiCfgReader::instance();
  unsigned char c = PCAN_USBBUS1;
  cfg->get_value(getLabel(), "channel",   c);
  pcan_config_.channel = c;

  cfg->get_value(getLabel(), "baud_rate", pcan_config_.baud_rate);
  cfg->get_value(getLabel(), "type",      pcan_config_.type);
  cfg->get_value(getLabel(), "port",      pcan_config_.port);
  cfg->get_value(getLabel(), "interrupt", pcan_config_.interrupt);
  return true;
}

bool PCanPropa::start() {
  connected_ = false;
  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    // g_status_ = CAN_Initialize(g_channel, g_baud_rate, g_type, g_port, g_interrupt);
    tmp_pcan_status_ = CAN_Initialize(pcan_config_.channel, pcan_config_.baud_rate,
        pcan_config_.type, pcan_config_.port, pcan_config_.interrupt);
    connected_ = (PCAN_ERROR_OK == tmp_pcan_status_);
    if (!connected_) {
      LOG_DEBUG << "(" << g_times_count + 1 << "/" << MAX_TRY_TIMES
          << ") Initialize CAN FAIL, status code: " << tmp_pcan_status_
          << ", Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      LOG_DEBUG << "Initialize CAN OK!";
      return connected_;
    }
  }

  LOG_ERROR << "Initialize CAN FAIL!!!";
  return connected_;
}

void PCanPropa::stop() {
  if (!connected_) return;

  // try to 10 times
  for (g_times_count = 0; g_times_count < MAX_TRY_TIMES; ++g_times_count) {
    tmp_pcan_status_ = CAN_Uninitialize(pcan_config_.channel);
    if (PCAN_ERROR_OK != tmp_pcan_status_){
      LOG_DEBUG << "(" << g_times_count + 1 << "/" << MAX_TRY_TIMES
          << ") Uninitialize CAN FAIL, status code: " << tmp_pcan_status_
          << ", Waiting 500ms... ...";
      // Waiting 500ms
      usleep(500000);
    } else {
      connected_ = false;
      LOG_INFO << "Uninitialize CAN OK!";
      return;
    }
  }

  LOG_ERROR << "Uninitialize CAN FAIL!!!";
}

} /* namespace agile_robot */

/*#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::PcanPropagate, middleware::Label)
CLASS_LOADER_REGISTER_CLASS(middleware::PcanPropagate, middleware::Propagate)*/

