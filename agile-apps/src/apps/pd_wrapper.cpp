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

#include "apps/internal/setup_env.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>

using agile_robot::MiiRobot;
using agile_robot::Joint;
using agile_robot::JointManager;


SINGLETON_IMPL(PdWrapper)

PdWrapper::PdWrapper()
  : MiiRobot(), nh_("~"), root_wrapper_(""), alive_(true),
    rt_interval_(1000/1000) {
  ///! Setup the root tag of Wrapper and Robot.
  if (!ros::param::get("~namespaces", param_ns_))
    LOG_FATAL << "PdWrapper can't find the 'namespaces' parameter "
        << "in the ros parameter server. Did you forget define this parameter.";

  std::string cfg_root;
  if (!ros::param::get(param_ns_ + "/prefix", cfg_root))
    LOG_FATAL << "RosWrapper can't find the 'prefix' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  root_wrapper_ = Label::make_label(cfg_root,      "wrapper");
  root_robot_   = Label::make_label(root_wrapper_, "robot");
  ///! Setup the ENV for our system
  internal::__setup_env();
}

PdWrapper::~PdWrapper() {
  alive_ = false;
  jnt_lft_leg_  = nullptr;
  jnt_rgt_leg_  = nullptr;
  jnt_lft_wing_ = nullptr;
  jnt_rgt_wing_ = nullptr;
  jnt_cog_      = nullptr;
  jnt_pitch_    = nullptr;
  // AutoInstor and CfgReader will destroy in the base class(MiiApp).
  // AutoInstor::destroy_instance();
  // CfgReader::destroy_instance();
}

void PdWrapper::create_system_singleton() {
  //! class this method in the base class.
  MiiRobot::create_system_singleton();

  internal::__setup_sys(param_ns_);
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
  ThreadPool::instance()->add("rt_puber", &PdWrapper::pub_rt_msg,   this);
  ThreadPool::instance()->add("control",  &PdWrapper::controlRobot, this);
  return true;
}

///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
void PdWrapper::pub_rt_msg() {
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
        msg.name.push_back    (jnt->joint_name());
      });
      msg.header.stamp = ros::Time::now();

      jnt_puber.publish(msg);
    }

    TICKER_CONTROL(rt_interval_, std::chrono::milliseconds);
  }

}

// TODO
void PdWrapper::cbForControl(const std_msgs::StringConstPtr& msg) {
  LOG_INFO << msg->data;
}

// TODO
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

#define WRAPPER PdWrapper
#include "apps/internal/main.cpp"

