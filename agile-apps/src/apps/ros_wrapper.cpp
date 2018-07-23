/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */
#include "apps/ros_wrapper.h"
#include "apps/internal/setup_env.h"

#include "repository/motor.h"
#include "repository/imu_sensor.h"
#include "repository/force_sensor.h"
#include "repository/joint_manager.h"

#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/registry/registry.h"
#include "foundation/thread/threadpool.h"

#include <rospack/rospack.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>


SINGLETON_IMPL(RosWrapper)

RosWrapper::RosWrapper()
  : MiiRobot(), MiiControl(),
    alive_(true), rt_duration_(1000/1000) {
  ///! Setup the root tag of Wrapper and Robot.
  if (!ros::param::get("~namespaces", param_ns_))
    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  std::string cfg_root;

  if (!ros::param::get(param_ns_ + "/prefix", cfg_root))
    LOG_FATAL << "RosWrapper can't find the 'prefix' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  root_wrapper_ = Label::make_label(cfg_root,      "wrapper");
  root_robot_   = Label::make_label(root_wrapper_, "robot");
  ///! Setup the ENV for our system
  internal::__setup_env();
}

RosWrapper::~RosWrapper() {
  alive_ = false;
  // agile_control::MiiControl::instance()->destroy_instance();
  // AutoInstor::destroy_instance();
  CfgReader::destroy_instance();
}

void RosWrapper::create_system_singleton() {
  if (nullptr == CfgReader::create_instance())
    LOG_FATAL << "Create the singleton 'CfgReader' has failed.";
  if (nullptr == AutoInstor::create_instance())
    LOG_FATAL << "Create the singleton 'AutoInstor' has failed.";

  ///! For each namespace (agile_robot or agile_control) given in the bringup.launch
  for (const auto& pns : {"~robot_ns", "~control_ns"}) {
    std::string ns, path, alias;
    std::vector<std::string> files;
    if (!ros::param::get(pns, ns)) {
      LOG_FATAL << "RosWapper can't find the '" << pns << "' parameter "
          << "in the parameter server. Did you forget define this parameter.";
    }

    ///! Get the alias of the configure of path.
    if ( !ros::param::get(ns + "/configure/prefix", alias)
      || !ros::param::get(ns + "/configure/file",   files)
      || !ros::param::get(alias, path)) {
      LOG_FATAL << "RosWapper can't find the '" << ns << "/configure/prefix "
          << "(or file)' or '" << alias << "' parameter in the parameter "
          << "server. Did you forget define this parameter.";
    }

    CfgReader::add_path(path);
    for (const auto& cfg : files)
      CfgReader::instance()->add_config(cfg);

    if ( !ros::param::get(ns + "/library/prefix", alias)
      || !ros::param::get(ns + "/library/file",   files)
      || !ros::param::get(alias, path)) {
      LOG_FATAL << "RosWapper can't find the '" << ns << "/library/prefix "
          << "(or file)' or '" << alias << "' parameter in the parameter "
          << "server. Did you forget define this parameter.";
    }

    ///! Got from ENV
    AutoInstor::add_path(path);
    for (const auto& lib : files)
      AutoInstor::instance()->add_library(lib);
  }

  MiiRobot::create_system_singleton();
  MiiControl::create_system_singleton();
}

//void RosWrapper::auto_inst() {
//  MiiApp::auto_inst();
//}

bool RosWrapper::init() {
  bool verbose = false;
  ros::param::get("~verbose", verbose);
  google::SetStderrLogging(verbose ? google::GLOG_INFO : google::GLOG_WARNING);

  if (!MiiRobot::init())
    LOG_FATAL << "Robot initializes fail!";
  LOG_INFO << "MiiRobot   initialization has completed.";

  if (!MiiControl::init())
    LOG_FATAL << "Launched the mii-control has completed.";
  LOG_INFO << "MiiControl initialization has completed.";

  std::string ns;
  ros::param::get("~robot_ns", ns);
  double frequency = 50.0;
  ros::param::get(ns + "/rt_frequency", frequency);
  if (frequency > 0)
    rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

  ros::param::get("~control_ns", ns);
  std::string str;
  if (!ros::param::get(ns + "/gait_topic", str)) {
  // if (!ros::param::get("~gait_topic", str)) {
    LOG_INFO << "No 'gait_topic' parameter, using the default name of topic"
        << " -- gait_control";
    str = "gait_control";
  }
  gait_ctrl_sub_ = nh_.subscribe<std_msgs::String>(str, 1,
      &RosWrapper::gaitControlCb, this);

  ThreadPool::instance()->add("rt_puber", &internal::__pub_rt_msg, alive_, rt_duration_);

// For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float32>("debug", 100,
      &RosWrapper::cbForDebug, this);
#endif

  return true;
}

void RosWrapper::gaitControlCb(const std_msgs::String::ConstPtr& msg) {
  if (msg->data.compare("null") || msg->data.compare("NULL"))
    activate("null");

  activate(msg->data);
}

#ifdef DEBUG_TOPIC
void RosWrapper::cbForDebug(const std_msgs::Float32ConstPtr& msg) {
  auto hfe = JointManager::instance()->getJointHandle(LegType::FL, JntType::HFE);
  auto kfe = JointManager::instance()->getJointHandle(LegType::FL, JntType::KFE);
  LOG_INFO << "Jnt: " << hfe->joint_name();

  double lim_hfe[] = {0,  1.3};
  double lim_kfe[] = {-2.0, -1.5};
//  double lim_hfe[] = {hfe->joint_position_min(), hfe->joint_position_max()};
//  double lim_kfe[] = {kfe->joint_position_min(), kfe->joint_position_max()};
  std::string type = "phase";

  //hfe->updateJointCommand(lim_hfe[0]);
  //LOG_INFO << "Go to initialize position.";
  //sleep(2); // in s

  ///! sin
  if (0 == type.compare("sin")) {
    for (double _x = 0; _x < 10 * 3.14; _x += 0.01) {
      // _y.push_back((limits[1] - limits[0])*sin(_x) + limits[0]);
      double tmp = 0.5*(lim_hfe[1] - lim_hfe[0])*sin(_x) + 0.5*(lim_hfe[1] + lim_hfe[0]);
      hfe->updateJointCommand(tmp);
      double tmp1 = 0.5*(lim_kfe[1] - lim_kfe[0])*sin(_x) + 0.5*(lim_kfe[1] + lim_kfe[0]);
      kfe->updateJointCommand(tmp1);
      LOG_INFO << "Add the target: " << tmp << ", " << tmp1;
      std::this_thread::sleep_for(std::chrono:: milliseconds((int)msg->data));
      //return;
    }
  } else if (0 == type.compare("linear")) {
    for (double _x = 0; _x <= 1; _x += 0.01) {
      double tmp = (lim_hfe[1] - lim_hfe[0])*_x + lim_hfe[0];
      //double tmp = 1.3;
      hfe->updateJointCommand(tmp);
      double tmp1 = (lim_kfe[1] - lim_kfe[0])*_x + lim_kfe[0];
      //double tmp1 = -1.5;
      kfe->updateJointCommand(tmp1);
      LOG_INFO << "Add the target: " << tmp1 << ", " << tmp1;
      std::this_thread::sleep_for(std::chrono:: milliseconds((int)msg->data));
      //return;
    }
  } else if (0 == type.compare("quadratic")) {
    for (double _x = 0; _x < 1; _x += 0.01) {
      // _y.push_back((limits[1] - limits[0])*sin(_x) + limits[0]);
      double tmp = (lim_hfe[1] - lim_hfe[0])*_x*_x + lim_hfe[0];
      kfe->updateJointCommand(tmp);
      LOG_INFO << "Add the target: " << tmp;
      std::this_thread::sleep_for(std::chrono:: milliseconds((int)msg->data));
      //return;
    }
  } else if (0 == type.compare("phase")) {
    hfe->updateJointCommand(msg->data);
    kfe->updateJointCommand(-2.0);
  } else if (0 == type.compare("square")) {
    for (int i = 0; i < (int)msg->data; ++i) {
      kfe->updateJointCommand(lim_kfe[i%2]);
      std::this_thread::sleep_for(std::chrono::seconds(4));
    }
  } else {
    ;
  }

  LOG_INFO << "Debug Callback Completed!";
}
#endif
