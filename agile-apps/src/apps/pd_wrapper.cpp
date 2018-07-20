/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */
#include "apps/pd_wrapper.h"

#include "repository/joint.h"
#include "repository/motor.h"
#include "repository/imu_sensor.h"
#include "repository/force_sensor.h"
#include "repository/joint_manager.h"

#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/registry/registry.h"
#include "foundation/registry/registry2.h"
#include "foundation/thread/threadpool.h"

#include <rospack/rospack.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>

using agile_robot::MiiRobot;
using agile_robot::Joint;
using agile_robot::JointManager;

/*!
 * @brief Setup the env for the agile-system.
 *        Add the devel_lib_root:  the/path/to/devel/lib
 */
void __setup_env() {
  std::string pkg_name;
  if (!ros::param::get("pkg_name", pkg_name)) {
    printf("\033[0;31mNo such parameters with named pkg_name, using the default"
        " value 'agile_apps'...\033[0m\n");
    pkg_name = "agile_apps";
  }

  rospack::Rospack rp;
  std::vector<std::string> search_path;
  if(!rp.getSearchPathFromEnv(search_path)) {
    printf("\033[0;31mCan't search the path from ENV...\033[0m\n");
    exit(-1);
  }
  // We crawl here because profile (above) does its own special crawl.
  rp.crawl(search_path, false);

  std::string apps_root;
  if (!rp.find(pkg_name, apps_root)) {
    printf("\033[0;31mCan't find the named '%s' package...\033[0m\n", pkg_name.c_str());
    exit(-1);
  }

  std::string pkgs_root = apps_root.substr(0, apps_root.rfind('/'));
  std::string libs_root = pkgs_root.substr(0, pkgs_root.rfind('/'));
  libs_root  = libs_root.substr(0, libs_root.rfind('/'));
  libs_root += "/devel/lib";

  ///! Setting the alias of paths
  ros::param::set("apps_root", apps_root);
  ros::param::set("pkgs_root", pkgs_root);
  ros::param::set("libs_root", libs_root);

  printf("\n");
  printf("\033[1;36;43mENV: \n");
  printf("    libs_root:  %s\n",        libs_root.c_str());
  printf("    pkgs_root:  %s\n",        pkgs_root.c_str());
  printf("    apps_root:  %s\033[0m\n", apps_root.c_str());
  printf("\n");

  bool verbose = true;
  ros::param::get("~verbose", verbose);
  LOG_ERROR << "VERBOSE: " << (verbose ? "true" : "false");
  google::SetStderrLogging(verbose ? google::GLOG_INFO : google::GLOG_WARNING);
}


SINGLETON_IMPL(PdWrapper)

PdWrapper::PdWrapper()
  : MiiRobot(), root_wrapper_(""), alive_(true),
    rt_interval_(1000/1000) {
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
  __setup_env();
}

PdWrapper::~PdWrapper() {
  alive_ = false;
  jnt_lft_leg_  = nullptr;
  jnt_rgt_leg_  = nullptr;
  jnt_lft_wing_ = nullptr;
  jnt_rgt_wing_ = nullptr;
  jnt_cog_      = nullptr;
  jnt_pitch_    = nullptr;
  // AutoInstor will destroy in the base class(MiiApp).
  // AutoInstor::destroy_instance();
  // CfgReader::destroy_instance();
}

void PdWrapper::create_system_singleton() {
  //! class this method in the base class.
  MiiRobot::create_system_singleton();

  std::string nss_str;
  std::vector<std::string> nss;
  // if (nh_.getParam("namespaces", nss)) {
  if (!ros::param::get("~namespaces", nss_str)) {
    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  } else {
    std::stringstream ss;
    ss << nss_str;
    std::string ns;
    while (ss >> ns) nss.push_back(ns);
  }

  ///! For each namespace (agile_robot or agile_control) given in the bringup.launch
  for (const auto& ns : nss) {
    // LOG_INFO << "namespace: " << ns;
    std::string path, alias;
    std::vector<std::string> files;
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
}

//void PdWrapper::auto_inst() {
//  MiiApp::auto_inst();
//}

bool PdWrapper::init() {
  if (!MiiRobot::init())
    LOG_FATAL << "MiiRobot initializes fail!";
  LOG_INFO << "MiiRobot initialization has completed.";

  ///! Get joint handle from JointManager.
  jnt_lft_leg_  = JointManager::instance()->getJointHandle(LegType::FL, JntType::HFE);
  jnt_rgt_leg_  = JointManager::instance()->getJointHandle(LegType::FL, JntType::KFE);
  jnt_lft_wing_ = JointManager::instance()->getJointHandle(LegType::FR, JntType::HFE);
  jnt_rgt_wing_ = JointManager::instance()->getJointHandle(LegType::FR, JntType::KFE);
  jnt_cog_      = JointManager::instance()->getJointHandle(LegType::HL, JntType::HFE);
  jnt_pitch_    = JointManager::instance()->getJointHandle(LegType::HL, JntType::KFE);

  ///! Initialize the Wrapper.
  auto cfg = CfgReader::instance();

  double frequency = 50.0;
  cfg->get_value(root_wrapper_, "rt_freq", frequency);
  rt_interval_ = std::chrono::milliseconds((int)(1000.0 / frequency));

  frequency = 500;
  cfg->get_value(root_wrapper_, "ctrl_freq", frequency);
  tick_interval_ = std::chrono::microseconds((int)(1000000.0/frequency));

// For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float32>("debug", 100,
      &PdWrapper::cbForDebug, this);
#endif

  std::string ctrl_topic = "policy_control";
  cfg->get_value(root_wrapper_, "policy_topic", ctrl_topic);
  ctrl_sub_ = nh_.subscribe<std_msgs::String>(ctrl_topic, 1,
      &PdWrapper::cbForControl, this);
  // FOR DEBUG
  Label::printfEveryInstance();

  // registry the thread.
  ThreadPool::instance()->add("rt_puber", &PdWrapper::publishRTMsg, this);
  ThreadPool::instance()->add("control",  &PdWrapper::controlRobot, this);
  return true;
}

void PdWrapper::cbForControl(const std_msgs::StringConstPtr& msg) {
  LOG_INFO << msg->data;
}

void PdWrapper::controlRobot() {
  TICKER_INIT(std::chrono::microseconds);
  while (alive_ && ros::ok()) {
//    JointManager::instance()->foreach([](MiiPtr<Joint>& jnt){
//      printf("[pd_wrapper.cpp: %03d]%8s: %+7.04f %+7.04f %+7.04f\n",
//          __LINE__, jnt->joint_name().c_str(), jnt->joint_position(),
//          jnt->joint_velocity(), jnt->joint_torque());
//    });
//    printf("\n");

    JointManager::instance()->foreach([](MiiPtr<Joint>& jnt){
      jnt->updateJointCommand(1);
    });
    TICKER_CONTROL(tick_interval_, std::chrono::microseconds);
  }
}

void PdWrapper::publishRTMsg() {
  ros::Publisher jnt_puber
      = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);

  TICKER_INIT(std::chrono::milliseconds);
  while (alive_ && ros::ok()) {
    if (jnt_puber.getNumSubscribers()) {
      sensor_msgs::JointState msg;
      JointManager::instance()->foreach([&msg](MiiPtr<Joint>& jnt){
        msg.position.push_back(((int) (jnt->joint_position()*1000000))/1000000.0);
        msg.velocity.push_back(((int) (jnt->joint_velocity()*1000000))/1000000.0);
        msg.effort.push_back  (((int) (jnt->joint_torque()  *1000000))/1000000.0);
        msg.name.push_back    (jnt->joint_name());
      });
      msg.header.stamp = ros::Time::now();

      jnt_puber.publish(msg);
    }

    TICKER_CONTROL(rt_interval_, std::chrono::milliseconds);
  }

  alive_ = false;
}


#ifdef DEBUG_TOPIC
void PdWrapper::cbForDebug(const std_msgs::Float32ConstPtr& msg) {
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
