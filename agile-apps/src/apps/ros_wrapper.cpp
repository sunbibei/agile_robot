/*
 * ros_wrapper.cpp
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
  std::string nss_str;
  std::vector<std::string> nss;
  if (!ros::param::get("~namespaces", nss_str)) {
    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";
  }

  std::stringstream ss;
  ss << nss_str;
  for (int i = APP_ROBOT; i < N_APPTYPE; ++i) {
    if (!(ss >> param_nss_[i]))
      LOG_FATAL << "You should be filled " << N_APPTYPE << "'s namespaces.";
    const auto& ns = param_nss_[i];

    std::string cfg_root;
    if (!ros::param::get(ns + "/prefix", cfg_root))
      LOG_FATAL << "RosWrapper can't find the 'prefix' parameter "
          << "in the parameter server. Did you forget define this parameter.";

    root_wrapper_[i] = Label::make_label(cfg_root,   "wrapper");
  }

  ///! Initialize the MiiRobot::root_robot_
  root_robot_   = Label::make_label(root_wrapper_[APP_ROBOT],   "robot");
  ///! Initialize the MiiControl::root_control_
  root_control_ = Label::make_label(root_wrapper_[APP_CONTROL], "control");

  ///! Setup the ENV for our system
  internal::__setup_env();
}

RosWrapper::~RosWrapper() {
  alive_ = false;
  // AutoInstor and CfgReader will destroy in the base class(MiiApp).
  // AutoInstor::destroy_instance();
  // CfgReader::destroy_instance();
}

void RosWrapper::create_system_singleton() {
  //! class this method in the base class.
  MiiRobot  ::create_system_singleton();
  MiiControl::create_system_singleton();

  for (const auto& ns : param_nss_)
    internal::__setup_sys(ns);
}

bool RosWrapper::init() {
  if (!MiiRobot::init())
    LOG_FATAL << "Robot initializes fail!";
  LOG_INFO << "MiiRobot   initialization has completed.";

  if (!MiiControl::init())
    LOG_FATAL << "Launched the mii-control has completed.";
  LOG_INFO << "MiiControl initialization has completed.";

  double frequency = 50.0;
  ros::param::get(param_nss_[APP_ROBOT] + "/rt_frequency", frequency);
  if (frequency > 0)
    rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

  std::string str;
  if (!ros::param::get(param_nss_[APP_CONTROL] + "/gait_topic", str)) {
  // if (!ros::param::get("~gait_topic", str)) {
    LOG_INFO << "No 'gait_topic' parameter, using the default name of topic"
        << " -- gait_control";
    str = "gait_control";
  }
  gait_ctrl_sub_ = nh_.subscribe<std_msgs::String>(str, 1,
      &RosWrapper::gaitControlCb, this);

  ThreadPool::instance()->add("rt_puber", &RosWrapper::pub_rt_msg, this);

// For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float32>("debug", 100,
      &RosWrapper::cbForDebug, this);
#endif

  return true;
}

///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
void RosWrapper::pub_rt_msg() {
  ros::NodeHandle _nh;
  ros::Publisher jnt_puber = _nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  TICKER_INIT(std::chrono::milliseconds);
  while (alive_ && ros::ok()) {
    if (jnt_puber.getNumSubscribers()) {
      sensor_msgs::JointState msg;
      agile_robot::JointManager::instance()->foreach([&msg](MiiPtr<Joint>& jnt){
        msg.position.push_back(((int) (jnt->joint_position()*1000000))/1000000.0);
        msg.velocity.push_back(((int) (jnt->joint_velocity()*1000000))/1000000.0);
        msg.effort.push_back  (((int) (jnt->joint_torque()  *1000000))/1000000.0);
        msg.name.push_back            (jnt->joint_name());
      });
      msg.header.stamp = ros::Time::now();

      jnt_puber.publish(msg);
    }

    TICKER_CONTROL(rt_duration_, std::chrono::milliseconds);
  }

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

#define WRAPPER RosWrapper
#include "apps/internal/main.cpp"
