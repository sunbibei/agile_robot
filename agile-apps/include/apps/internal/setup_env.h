/*
 * setup_env.h
 *
 *  Created on: Jul 23, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_APPS_INTERNAL_SETUP_ENV_H_
#define INCLUDE_APPS_INTERNAL_SETUP_ENV_H_

#include <thread>
#include <ros/ros.h>
#include <rospack/rospack.h>
#include <sensor_msgs/JointState.h>

#include "foundation/utf.h"
#include "foundation/auto_instor.h"

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

  ///! Setting the alias of paths, The parameter in the ROS PARAMETER SERVER
  ///! is shared over each process, so we only need to set once.
  if (!ros::param::has("apps_root"))
    ros::param::set("apps_root", apps_root);

  if (!ros::param::has("pkgs_root"))
    ros::param::set("pkgs_root", pkgs_root);

  if (!ros::param::has("libs_root"))
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

void __setup_sys(const std::string& ns) {
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
//  std::string ns;
//  if (!ros::param::get("~namespaces", ns)) {
//    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
//        << "in the parameter server. Did you forget define this parameter.";
//  }
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
      LOG_WARNING << "RosWapper can't find the '" << ns << "/configure/prefix "
          << "(or file)' or '" << alias << "' parameter in the parameter "
          << "server. Did you forget define this parameter.";
    } else {
      CfgReader::add_path(path);
      for (const auto& cfg : files)
        CfgReader::instance()->add_config(cfg);
    }

    if ( !ros::param::get(ns + "/library/prefix", alias)
      || !ros::param::get(ns + "/library/file",   files)
      || !ros::param::get(alias, path)) {
      LOG_WARNING << "RosWapper can't find the '" << ns << "/library/prefix "
          << "(or file)' or '" << alias << "' parameter in the parameter "
          << "server. Did you forget define this parameter.";
    } else {
      AutoInstor::add_path(path);
      for (const auto& lib : files)
        AutoInstor::instance()->add_library(lib);
    }
  }
}

} /* namespace internal */

#endif /* INCLUDE_APPS_INTERNAL_SETUP_ENV_H_ */
