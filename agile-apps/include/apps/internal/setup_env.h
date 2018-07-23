/*
 * setup_env.h
 *
 *  Created on: Jul 23, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_APPS_INTERNAL_SETUP_ENV_H_
#define INCLUDE_APPS_INTERNAL_SETUP_ENV_H_

#include <ros/ros.h>
#include <rospack/rospack.h>
#include <sensor_msgs/JointState.h>

#include "foundation/utf.h"
#include "repository/joint_manager.h"

namespace internal {

/*!
 * @brief Setup the env for the agile-system using the 'pkg_name' and 'verbose'
 *        parameter.
 *        Add the libs_root:  the/path/to/devel/lib
 *        Add the apps_root:  the/path/to/src/agile_robot/agile_apps
 *        Add the pkgs_root:  the/path/to/src/agile_robot
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

void __setup_sys() {
//  std::string nss_str;
//  std::vector<std::string> nss;
//  // if (nh_.getParam("namespaces", nss)) {
//  if (!ros::param::get("~namespaces", nss_str)) {
//    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
//        << "in the parameter server. Did you forget define this parameter.";
//  } else {
//    std::stringstream ss;
//    ss << nss_str;
//    std::string ns;
//    while (ss >> ns) nss.push_back(ns);
//  }
  std::string ns;
  if (!ros::param::get("~namespaces", ns)) {
    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }
  ///! For each namespace (agile_robot or agile_control) given in the bringup.launch
  // for (const auto& ns : nss) {
  if (!ns.empty()) {
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

///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
void __pub_rt_msg(const bool& alive, std::chrono::milliseconds rt_interval) {
  ros::NodeHandle _nh;
  ros::Publisher jnt_puber = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  TICKER_INIT(std::chrono::milliseconds);
  while (alive && ros::ok()) {
    if (jnt_puber.getNumSubscribers()) {
      sensor_msgs::JointState msg;
      agile_robot::JointManager::instance()->foreach([&msg](MiiPtr<Joint>& jnt){
        msg.position.push_back(((int) (jnt->joint_position()*1000000))/1000000.0);
        msg.velocity.push_back(((int) (jnt->joint_velocity()*1000000))/1000000.0);
        msg.effort.push_back  (((int) (jnt->joint_torque()  *1000000))/1000000.0);
        msg.name.push_back    (jnt->joint_name());
      });
      msg.header.stamp = ros::Time::now();

      jnt_puber.publish(msg);
    }

    TICKER_CONTROL(rt_interval, std::chrono::milliseconds);
  }

}

} /* namespace internal */

#endif /* INCLUDE_APPS_INTERNAL_SETUP_ENV_H_ */