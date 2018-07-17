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

CfgReader::Callback1 MiiApp::s_auto_inst_cb_
  = std::bind(&MiiApp::auto_inst_cb, std::placeholders::_1, std::placeholders::_2);

MiiApp::MiiApp() {
  ;
}

MiiApp::~MiiApp() {
  ThreadPool::destroy_instance();
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
}

void MiiApp::mv_auto_inst_cb(CfgReader::Callback1& cb) {
  s_auto_inst_cb_ = cb;
}

bool MiiApp::auto_inst_cb(const std::string& __p, const std::string& __type) {
  if (!AutoInstor::instance()->make_instance(__p, __type)) {
    LOG_ERROR << "Create instance(" << __type << " " << __p << ") fail!";
    return false;
  }

  return true;
}

void MiiApp::auto_inst() {
  // All of the objects mark with "auto_inst" in the configure file
  // will be instanced here.
  LOG_DEBUG << "Now, We are ready to auto_inst object in the configure file.";
  CfgReader::instance()->regAttrCb("auto_inst", s_auto_inst_cb_);
  LOG_DEBUG << "AutoInst has finished.";
}

bool MiiApp::run() {
  return ThreadPool::instance()->start();
}

