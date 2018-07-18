/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */
#include "apps/ros_wrapper.h"

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

// #define MII_CTRL_THREAD ("mii_control")
// #define RT_PUB_THREAD   ("rt_publisher")

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

  printf("\033[1;36;43m\n");
  printf("ENV: \n");
  printf("    libs_root:  %s\n", libs_root.c_str());
  printf("    pkgs_root:  %s\n", pkgs_root.c_str());
  printf("    apps_root:  %s\n", apps_root.c_str());
  printf("\033[0m\n");

}

SINGLETON_IMPL_NO_CREATE(RosWrapper)

RosWrapper* RosWrapper::create_instance(const std::string& __tag1, const std::string& __tag2) {
  if (nullptr != s_inst_) {
    LOG_WARNING << "This method 'create_instance()' is called twice.";
  } else {
    s_inst_ = new RosWrapper(Label::make_label(__tag1, "wrapper"),
        Label::make_label(__tag2, "wrapper"));
  }
  return s_inst_;
}

RosWrapper::RosWrapper(const std::string& _robot_tag, const std::string& _ctrl_tag)
  : MiiRobot(Label::make_label(_robot_tag,  "robot")),
    MiiControl(Label::make_label(_ctrl_tag, "control")),
    robot_root_(_robot_tag), control_root_(_ctrl_tag),
    alive_(true), rt_duration_(1000/1000) {
  // LOG_DEBUG << "Enter the roswrapper construction";
  // google::InitGoogleLogging("qr_driver");
  // google::SetLogDestination(google::GLOG_INFO, "/path/to/log/INFO_");
  // FLAGS_colorlogtostderr = true;
  // google::FlushLogFiles(google::GLOG_INFO);
  // LOG_DEBUG << "Leave the roswrapper construction";
  __setup_env();
  ; // Nothing to do here, all of variables initialize in the method @start()
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

  ThreadPool::instance()->add("rt_puber", &RosWrapper::publishRTMsg, this);

// For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float32>("debug", 100,
      &RosWrapper::cbForDebug, this);
#endif

  return true;
}

inline void __fill_jnt_data(sensor_msgs::JointState& to, JointManager* from) {
  to.name.clear();
  to.position.clear();
  to.velocity.clear();
  to.effort.clear();
  to.name.clear();
  for (const auto& jnt : *from) {
    to.position.push_back(((int) (jnt->joint_position()*1000000))/1000000.0);
    to.velocity.push_back(((int) (jnt->joint_velocity()*1000000))/1000000.0);
    to.effort.push_back  (((int) (jnt->joint_torque()  *1000000))/1000000.0);
    to.name.push_back(jnt->joint_name());
  }
  to.header.stamp = ros::Time::now();
}

inline void __fill_motor_data(sensor_msgs::JointState& to, JointManager* from) {
  to.name.clear();
  to.position.clear();
  to.velocity.clear();
  to.effort.clear();
  to.name.clear();
  for (const auto& jnt : *from) {
    to.position.push_back(jnt->joint_motor()->motor_position());
    to.velocity.push_back(jnt->joint_motor()->motor_velocity());
    to.effort.push_back(jnt->joint_motor()->motor_torque());
    to.name.push_back(jnt->joint_motor()->motor_name());
  }
  to.header.stamp = ros::Time::now();
}

inline void __fill_imu_data(sensor_msgs::Imu& to, ImuSensor* from) {
  to.orientation.x = from->orientation_const_pointer()[0];
  to.orientation.y = from->orientation_const_pointer()[1];
  to.orientation.z = from->orientation_const_pointer()[2];
  to.orientation.w = from->orientation_const_pointer()[3];
  for (size_t i = 0; i < to.orientation_covariance.size(); ++i)
    to.orientation_covariance[i] = from->orientation_covariance_const_pointer()[i];

  to.angular_velocity.x = from->angular_velocity_const_pointer()[0];
  to.angular_velocity.y = from->angular_velocity_const_pointer()[1];
  to.angular_velocity.z = from->angular_velocity_const_pointer()[2];
  for (size_t i = 0; i < to.angular_velocity_covariance.size(); ++i)
    to.angular_velocity_covariance[i] = from->angular_velocity_covariance_const_pointer()[i];

  to.linear_acceleration.x = from->linear_acceleration_const_pointer()[0];
  to.linear_acceleration.y = from->linear_acceleration_const_pointer()[1];
  to.linear_acceleration.z = from->linear_acceleration_const_pointer()[2];
  for (size_t i = 0; i < to.linear_acceleration_covariance.size(); ++i)
    to.linear_acceleration_covariance[i] = from->linear_acceleration_covariance_const_pointer()[i];

  to.header.stamp = ros::Time::now();
}

inline void __fill_force_data(std_msgs::Int32MultiArray& to, std::vector<ForceSensor*>& from) {
  for (const LegType& leg : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    // LOG_DEBUG << "enter: " << leg << " " << to.data.size() << " " << from.size();// << " " << from[leg];
    to.data[leg] = (nullptr != from[leg]) ? from[leg]->force_data() : NAN;
  }
}

inline void __fill_cmd_data(std_msgs::Float64MultiArray& __cmd_msg, Eigen::VectorXd** cmds) {
  __cmd_msg.data.clear();
  for (const auto& l : {LegType::FL, LegType::FR, LegType::HL, LegType::HR}) {
    const auto& cmd = *(cmds[l]);
    for (const auto& j : {JntType::KFE, JntType::HFE, JntType::HAA}) {
      __cmd_msg.data.push_back(cmd(j));
    }
  }
}

void RosWrapper::publishRTMsg() {
  ros::Publisher jnt_puber
      = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher motor_puber
        = nh_.advertise<sensor_msgs::JointState>("motor_states", 1);
  ros::Publisher imu_puber
      = nh_.advertise<sensor_msgs::Imu>("imu", 1);
  ros::Publisher force_puber
      = nh_.advertise<std_msgs::Int32MultiArray>("foot_forces", 1);

  // TODO
  ros::Publisher hip_puber
        = nh_.advertise<std_msgs::Float32>("hip_jnt", 1);

  Joint* hip = JointManager::instance()->getJointHandle(LegType::FL, JntType::HFE);

// TODO Redesigned the Command publisher.
//  ros::Publisher cmd_puber;
//  if (!use_ros_control_)
//    cmd_puber = nh_.advertise<std_msgs::Float64MultiArray>("/dragon/joint_commands", 10);

  sensor_msgs::JointState     __jnt_msg;
  sensor_msgs::JointState     __motor_msg;
  sensor_msgs::Imu            __imu_msg;
  std_msgs::Int32MultiArray   __f_msg;
  std_msgs::Float64MultiArray __cmd_msg;

  // TODO
  std_msgs::Float32 __hip_msg;

  __imu_msg.header.frame_id = "imu";

  __f_msg.data.resize(4);
  __f_msg.layout.data_offset = 0;
  for (const std::string& l : {"fl", "fr", "hl", "hr"}) {
    std_msgs::MultiArrayDimension dim;
    dim.label = "foot_" + l;
    dim.size  = 1;
    dim.stride= 1;
    __f_msg.layout.dim.push_back(dim);
  }

//  Eigen::VectorXd* _sub_cmd[LegType::N_LEGS];
//  if (!use_ros_control_) {
//    auto cfg = CfgReader::instance();
//    MiiVector<MiiString> cmds;
//    cfg->get_value(Label::make_label(root_tag_, "roswrapper"), "cmds", cmds);
//    FOR_EACH_LEG(l) {
//     _sub_cmd[l] = GET_COMMAND_NO_FLAG(cmds[l], Eigen::VectorXd*);
//    }
//  }

  TICKER_INIT(std::chrono::milliseconds);

  while (alive_ && ros::ok()) {
    if (jnt_puber.getNumSubscribers()) {
      __fill_jnt_data(__jnt_msg, jnt_manager_);
      jnt_puber.publish(__jnt_msg);
    }

    if (motor_puber.getNumSubscribers()) {
      __fill_motor_data(__motor_msg, jnt_manager_);
      motor_puber.publish(__motor_msg);
    }

    if (imu_puber.getNumSubscribers()) {
      __fill_imu_data(__imu_msg, imu_sensor_);
      imu_puber.publish(__imu_msg);
    }

    if (force_puber.getNumSubscribers()) {
      __fill_force_data(__f_msg, td_list_by_type_);
      force_puber.publish(__f_msg);
    }

    if (hip_puber.getNumSubscribers()) {
      __hip_msg.data = hip->joint_position();
      hip_puber.publish(__hip_msg);
    }

//    if (!use_ros_control_ && cmd_puber.getNumSubscribers()) {
//      __fill_cmd_data(__cmd_msg, _sub_cmd);
//      cmd_puber.publish(__cmd_msg);
//    }

    TICKER_CONTROL(rt_duration_, std::chrono::milliseconds);
  }

  alive_ = false;
}

void RosWrapper::gaitControlCb(const std_msgs::String::ConstPtr& msg) {
  if (msg->data.compare("null") || msg->data.compare("NULL"))
    activate("null");

  activate(msg->data);
}

#ifdef DEBUG_TOPIC
void RosWrapper::cbForDebug(const std_msgs::Float32ConstPtr& msg) {
  auto hfe = jnt_manager_->getJointHandle(LegType::FL, JntType::HFE);
  auto kfe = jnt_manager_->getJointHandle(LegType::FL, JntType::KFE);
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
