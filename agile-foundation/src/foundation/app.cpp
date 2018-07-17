/*
 * app.cpp
 *
 *  Created on: Jul 17, 2018
 *      Author: bibei
 */

#include "foundation/app.h"
#include "foundation/utf.h"
#include "foundation/thread/threadpool.h"

MiiApp::MiiApp() {
  ;
}

MiiApp::~MiiApp() {
  ;
}

void MiiApp::start() {
  ///! THE CREATING STAGE.
  // create the all of singleton
  create_system_instance();
  // initialize the system.
  if (!init()) LOG_FATAL << "FAIL to launch the APP";

  ///! THE RUNNING STAGE.
  // run the system.
  run();
}

bool MiiApp::run() {
  return ThreadPool::instance()->start();
}

