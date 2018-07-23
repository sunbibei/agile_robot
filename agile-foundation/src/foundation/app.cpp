/*
 * app.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: bibei
 */

#include "foundation/app.h"
#include "foundation/utf.h"
#include "foundation/auto_instor.h"
#include "foundation/thread/threadpool.h"

int MiiApp::s_N_priorities_ = 5;

CfgReader::Callback1 MiiApp::s_auto_inst_cb_ = [](const std::string& __p, const std::string& __type) {
  if (!AutoInstor::instance()->make_instance(__p, __type)) {
    LOG_ERROR << "Create instance(" << __type << " " << __p << ") fail!";
    return false;
  }

  LOG_INFO << "INST: " << __type << ": " << __p;
  return true;
};
//  = std::bind(&MiiApp::auto_inst_cb, std::placeholders::_1, std::placeholders::_2);

MiiApp::MiiApp() {
  ;
}

MiiApp::~MiiApp() {
  ThreadPool::destroy_instance();
  CfgReader::destroy_instance();
  AutoInstor::destroy_instance();
}

void MiiApp::start() {
  ///! THE CREATING STAGE.
  // create the all of singleton
  create_system_singleton();
  // create the all of instance
  auto_inst();
  // initialize the system.
  if (!init()) LOG_FATAL << "FAIL to launch the APP";

  ///! THE RUNNING STAGE.
  // run the system.
  run();
}

void MiiApp::create_system_singleton() {
  if (nullptr == ThreadPool::create_instance())
    LOG_FATAL << "Create the singleton 'ThreadPool' has failed.";

  if (nullptr == CfgReader::create_instance())
    LOG_FATAL << "Create the singleton 'CfgReader' has failed.";

  if (nullptr == AutoInstor::create_instance())
    LOG_FATAL << "Create the singleton 'AutoInstor' has failed.";
}

void MiiApp::mv_auto_inst_cb(CfgReader::Callback1& cb) {
  s_auto_inst_cb_ = cb;
}

bool MiiApp::auto_inst_cb(const std::string& __p, const std::string& __type) {
  if (!AutoInstor::instance()->make_instance(__p, __type)) {
    LOG_ERROR << "Create instance(" << __type << " " << __p << ") fail!";
    return false;
  }

  // LOG_INFO << "INST: " << __type << ": " << __p;
  return true;
}

void MiiApp::auto_inst() {
  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
//  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
//  CfgReader::instance()->regAttrCb("auto_inst", s_auto_inst_cb_);
//  LOG_DEBUG << "AutoInst has finished.";

  auto cfg = CfgReader::instance();
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  std::vector<std::vector<std::string>> __obj_labels(s_N_priorities_);
  cfg->regAttrCb("auto_inst",
      [&](const std::string& _l, const std::string& _t) -> bool {
    int priority = s_N_priorities_ - 1;
    cfg->get_value(_l, "priority", priority);
    __obj_labels[priority].push_back(_l);
    return true;
  });

  for (const auto& level : __obj_labels) {
    for (const auto& l : level) {
      std::string t;
      if (cfg->get_value(l, "auto_inst", t))
        s_auto_inst_cb_(l, t); ///! make instance
    }
  }

  LOG_DEBUG << "AutoInst has finished.";
}

bool MiiApp::run() {
  return ThreadPool::instance()->start();
}

