/*
 * propagate.cpp
 *
 *  Created on: Jan 9, 2017
 *      Author: silence
 */

#include "platform/propagate/propagate_manager.h"

#include <foundation/utf.h>
#include "foundation/cfg_reader.h"

#include "platform/propagate/propagate.h"

namespace agile_robot {

#define PROPA_TAG_NAME  ("propagates")

Propagate::Propagate(const std::string& l)
  : Label(l), bus_id_(0x00), propa_name_("") {
  PropagateManager::instance()->add(this);
}

bool Propagate::auto_init() {
  std::string __p;
  Label::split_label(label_, __p, propa_name_);
  MiiCfgReader::instance()->get_value(getLabel(), "bus", bus_id_);
  return true;
}

bool Propagate::write(const Packet&) {
  LOG_ERROR << "Call the base method 'write'";
  return false;
}

bool Propagate::read(Packet&) {
  LOG_ERROR << "Call the base method 'write'";
  return false;
}

bool Propagate::start() {
  LOG_WARNING << "This method (Propagate::start()) should not be called.";
  return false;
}

void Propagate::stop() {
  LOG_WARNING << "This method (Propagate::stop()) should not be called.";
}

} /* namespace middleware */
