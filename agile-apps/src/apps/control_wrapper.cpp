/*
 * control_wrapper.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: bibei
 */

#include "apps/control_wrapper.h"

#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/thread/threadpool.h"

SINGLETON_IMPL_NO_CREATE(ControlWrapper)

ControlWrapper* ControlWrapper::create_instance(const std::string& __tag) {
  if (nullptr != s_inst_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    s_inst_ = new ControlWrapper(Label::make_label(__tag, "wrapper"));
  }
  return s_inst_;
}

ControlWrapper::ControlWrapper(const std::string& _prefix)
  : MiiControl(Label::make_label(_prefix, "wrapper")), nh_("agile_control"),
    root_tag_(_prefix), alive_(true) {
  ; // Nothing to do here.
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

void ControlWrapper::create_system_instance() {
  std::string prefix;
  std::vector<std::string> cfgs;
  if (!nh_.getParam("configure/prefix", prefix)
      || !nh_.getParam("configure/file", cfgs)) {
    LOG_FATAL << "RosWapper can't find the 'configure' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }

  ros::param::get(prefix, prefix);
  if (nullptr == CfgReader::create_instance())
    LOG_FATAL << "Create the singleton 'CfgReader' has failed.";
  for (size_t i = 0; i < cfgs.size(); ++i)
    CfgReader::instance()->add_config(prefix + "/" + cfgs[i]);

  if (!nh_.getParam("library/prefix", prefix)
      || !nh_.getParam("library/file", cfgs)) {
    LOG_FATAL << "RosWapper can't find the 'library' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }

  ros::param::get(prefix, prefix);
  if (nullptr == AutoInstor::create_instance())
    LOG_FATAL << "Create the singleton 'AutoInstor' has failed.";
  for (size_t i = 0; i < cfgs.size(); ++i)
    AutoInstor::instance()->add_library(prefix + "/" + cfgs[i]);

  MiiControl::create_system_instance();
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
