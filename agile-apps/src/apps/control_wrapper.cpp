/*
 * control_wrapper.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: bibei
 */

#include "apps/control_wrapper.h"
#include "apps/internal/setup_env.h"

#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/thread/threadpool.h"

SINGLETON_IMPL(ControlWrapper)

ControlWrapper::ControlWrapper()
  : MiiControl(), alive_(true) {
  ///! Setup the root tag of Wrapper and Robot.
  if (!ros::param::get("~namespaces", param_ns_))
    LOG_FATAL << "Wrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  std::string cfg_root;
  if (!ros::param::get(param_ns_ + "/prefix", cfg_root))
    LOG_FATAL << "Wrapper can't find the 'prefix' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  root_wrapper_ = Label::make_label(cfg_root,      "wrapper");
  root_control_ = Label::make_label(root_wrapper_, "control");

  ///! Setup the ENV for our system
  internal::__setup_env();
}

ControlWrapper::~ControlWrapper() {
  alive_ = false;
  // agile_control::MiiControl::instance()->destroy_instance();
  // AutoInstor::destroy_instance();
  CfgReader::destroy_instance();
}

bool ControlWrapper::init() {
  bool debug = false;
  ros::param::get("~debug", debug);
  google::SetStderrLogging(debug ? google::GLOG_INFO : google::GLOG_WARNING);

  if (!MiiControl::init())
    LOG_FATAL << "Initialization the 'MiiControl' has failed.";

  std::string str;
  if (!nh_.getParam("gait_topic", str)) {
  // if (!ros::param::get("~gait_topic", str)) {
    LOG_INFO << "No 'gait_topic' parameter, using the default name of topic"
        << " -- gait_control";
    str = "gait_control";
  }
  gait_ctrl_sub_ = nh_.subscribe<std_msgs::String>(str, 1,
      &ControlWrapper::gaitControlCb, this);

  // For debug
  #ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float32>("debug", 100,
      &ControlWrapper::cbForDebug, this);
  #endif

  return true;
}

void ControlWrapper::create_system_singleton() {
  //! class this method in the base class.
  MiiControl::create_system_singleton();

  internal::__setup_sys(param_ns_);
}

void ControlWrapper::gaitControlCb(const std_msgs::String::ConstPtr& msg) {
  if (msg->data.compare("NULL") || msg->data.compare("null"))
    activate("null");

  activate(msg->data);
}

#ifdef DEBUG_TOPIC
void ControlWrapper::cbForDebug(const std_msgs::Float32ConstPtr& msg) {
  LOG_INFO << "Debug Callback Completed!";
}
#endif

#define  WRAPPER ControlWrapper
#include "apps/internal/main.cpp"
