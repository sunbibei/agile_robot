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
#include "apps/internal/setup_env.h"

static bool g_is_alive = true;
void termial(int) { g_is_alive = false; }

void __launch_app(const std::string& tag) {
  // printf("%s\n", tag.c_str());
  auto cfg = CfgReader::instance();

  bool enable = false;
  if (!cfg->get_value(tag, "enable", enable) || !enable)
    return;

  std::vector<std::string> strs;
  cfg->get_value_fatal(tag, "command", strs);
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
        ///! Load the parameters to THE ROS PARAMETER SERVER
        system(command.c_str());
      }

      ///! EXECUTE this process
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

    printf("\033[1;31;43mThe %-21s exited!\033[0m\n", tag.c_str());
    exit(-1);
  }
  ///! In the main process, returned immediately.
  return;
}

void launcher() {
  std::string param_ns;
  if (!ros::param::get("~namespaces", param_ns))
    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  auto cfg = CfgReader::create_instance();
  if (nullptr == cfg) {
    printf("\033[0;31mCreate the CfgReader fail!\033[0m\n");
    exit(-1);
  }
  internal::__setup_sys(param_ns);

  std::string prefix = "";
  ros::param::get(param_ns + "/prefix", prefix);
  std::string root = Label::make_label( prefix, "launcher");

  std::string screen;
  cfg->get_value(root, "screen", screen);
  std::string output;
  cfg->get_value(root, "output", output);

  printf("screen: %s\noutput: %s\n", screen.c_str(), output.c_str());
  ///! launch each process
  printf("%s\n", root.c_str());
  cfg->foreachTag(root, __launch_app);
}

int main(int argc, char* argv[]) {
  // signal(SIGINT, termial);
  google::InitGoogleLogging("agile_apps");
  google::FlushLogFiles(google::GLOG_INFO);
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "system_entry");
  ros::NodeHandle nh;

  // setup the ENV
  internal::__setup_env();

  // launch the each APPS
  launcher();

  // Waiting for shutdown by user
  ros::waitForShutdown();

  // waiting for the all of child-process exiting.
//  int status = 0;
//  wait(&status);
  sleep(2);

  // destroy the CfgReader.
  CfgReader::destroy_instance();
  printf("\033[1;31;43mThe %-21s exited!\033[0m\n", "apps");
  return 0;
}


