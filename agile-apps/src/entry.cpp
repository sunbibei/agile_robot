/*
 * main.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: bibei
 */

#include <string>
#include <vector>

#include <fcntl.h>
#include <ros/ros.h>
#include <rospack/rospack.h>

#include <sys/wait.h>

#include "foundation/cfg_reader.h"

static bool g_is_alive = true;
static std::vector<std::string> g_apps_list
  = std::vector<std::string>{"robot", "control", "monitor", "log"};

void termial(int signo) {
  g_is_alive = false;
}

/*!
 * @brief Setup the env for the agile-system.
 *        Add the devel_lib_root:  the/path/to/devel/lib
 */
void setup_env() {
  std::string cfg;
  if (!ros::param::get("~configure", cfg)) {
    printf("\033[0;31mNo parameter with named configure or prefix!\033[0m\n");
    exit(-1);
  }
  if (nullptr == MiiCfgReader::create_instance(cfg)) {
    printf("\033[0;31mCreate the CfgReader fail!\033[0m\n");
    exit(-1);
  }
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

  ros::param::set("apps_root", apps_root);
  ros::param::set("pkgs_root", pkgs_root);
  ros::param::set("libs_root", libs_root);

  printf("\033[1;36;43m\n");
  printf("ENV: \n");
  printf("    libs_root:  %s\n", libs_root.c_str());
  printf("    pkgs_root:  %s\n", pkgs_root.c_str());
  printf("    apps_root:  %s\n", apps_root.c_str());
  printf("\033[0m\n");
}

void apps_launcher() {
  auto cfg = MiiCfgReader::instance();
  std::string prefix = "";
  ros::param::get("~prefix", prefix);
  std::string root = Label::make_label(prefix, "launcher");

  std::string screen;
  cfg->get_value(root, "screen", screen);
  std::string output;
  cfg->get_value(root, "output", output);

  printf("screen: %s\noutput: %s\n", screen.c_str(), output.c_str());
  bool enable = false;
  ///! launch each process
  for (const auto& app : g_apps_list) {
    // if (0 == output.compare(app)) continue;
    std::string tag = Label::make_label(root, app);
    if (!cfg->get_value(tag, "enable", enable) || !enable)
      continue;

    std::vector<std::string> strs;
    cfg->get_value_fatal(tag, "argv", strs);
    char** argv  = new char*[strs.size() + 1];
    char** pargv = argv;
    for (const auto& arg : strs) {
      *pargv = new char[arg.size() + 1];
      memcpy(*pargv, arg.c_str(), arg.size());
      (*pargv)[arg.size()] = '\0';

      ++pargv;
    }
    *pargv = nullptr;

//    int in, out;
    pid_t pid = fork();
    if (0 == pid) { // child process
      ///! cancel the output redirection.
//      char _buffer[128] = {0};
//      time_t _time;
//      time(&_time);
//      tm* _tm = std::localtime(&_time);
//      sprintf(_buffer, "%s_%4d-%02d-%02d_%02d-%02d-%02d", app.c_str(),
//          _tm->tm_year + 1900, _tm->tm_mon, _tm->tm_mday,
//          _tm->tm_hour, _tm->tm_min, _tm->tm_sec);
//
//      output += ("/" + std::string(_buffer) + "");
//      out = open(output.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0664);
//      dup2(out, STDOUT_FILENO);
//      close(out);
      ///! execute the sub-process.
      pid_t cpid = fork();
      if (0 == cpid) {
        std::string params;
        if (cfg->get_value(tag, "params", params)) {
          std::string command = "rosparam load " + params;
          system(command.c_str());
        }

        execvp(argv[0], argv);
        ///! What fucking! failed...
        pargv = argv;
        fprintf(stderr, "Failed to execute ");
        while (nullptr != pargv) fprintf(stderr, "%s ", *pargv);
        fprintf(stderr, "\n");
        exit(-1);/* end child process */
      }

      // waiting for the new child-process exiting.
      int status = 0;
      wait(&status);

      printf("\033[1;31;43mThe %s exited!\033[0m\n", app.c_str());
      exit(-1);
    }


  }
}

int main(int argc, char* argv[]) {
  // signal(SIGINT, termial);
  ros::init(argc, argv, "system_entry");
  ros::NodeHandle nh("agile_apps");

  // setup the ENV
  setup_env();
  // launch the each APPS
  apps_launcher();

  // Waiting for shutdown by user
  ros::waitForShutdown();

  // waiting for the all of child-process exiting.
//  int status = 0;
//  wait(&status);
  sleep(2);

  // destroy the CfgReader.
  MiiCfgReader::destroy_instance();
  printf("\033[1;31;43mThe apps exited!\033[0m\n");
  return 0;
}


