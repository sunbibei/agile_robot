/*
 * qr_ros_wrapper.cpp
 *
 *  Created on: Dec 5, 2016
 *      Author: silence
 */
#include "apps/robot_wrapper.h"
#include "repository/motor.h"
#include "repository/imu_sensor.h"
#include "repository/force_sensor.h"
#include "repository/joint_manager.h"

#include "foundation/cfg_reader.h"
#include "foundation/auto_instor.h"
#include "foundation/thread/threadpool.h"
#include "foundation/registry/registry.h"
#include "foundation/registry/registry2.h"

#include "apps/internal/setup_env.h"

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int32MultiArray.h>

#include <Eigen/Dense>

#define UN_REG ((char)0x00)
#define REG_1  ((char)0x01)
#define REG_2  ((char)0x02)
struct __RegsStu {
  char reg_type                  = UN_REG;
  bool reg_legs[LegType::N_LEGS] = {false};
  std::atomic_bool* joint_command_flags[LegType::N_LEGS] = {nullptr};
  Eigen::VectorXd*  joint_commands     [LegType::N_LEGS] = {nullptr};
  Eigen::MatrixXd*  joint_limitss      [LegType::N_LEGS] = {nullptr};
  Eigen::VectorXd*  joint_positions    [LegType::N_LEGS] = {nullptr};
  Eigen::VectorXd*  joint_velocities   [LegType::N_LEGS] = {nullptr};
  Eigen::VectorXd*  joint_torques      [LegType::N_LEGS] = {nullptr};

  ~__RegsStu() {
    for (auto& cmd : joint_commands)   delete cmd;
    for (auto& lim : joint_limitss)    delete lim;
    for (auto& pos : joint_positions)  delete pos;
    for (auto& vel : joint_velocities) delete vel;
    for (auto& tor : joint_torques)    delete tor;

    if (REG_2 == reg_type)
      for (auto& flag : joint_command_flags) delete flag;
  }
};

SINGLETON_IMPL(RobotWrapper)

RobotWrapper::RobotWrapper()
  : MiiRobot(), root_wrapper_(""), alive_(true),
    rt_duration_(1000/50), regs_stu_(nullptr) {
  ///! Setup the root tag of Wrapper and Robot.
  if (!ros::param::get("~namespaces", param_ns_))
    LOG_FATAL << "Wrapper can't find the 'namespaces' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  std::string cfg_root;
  if (!ros::param::get(param_ns_ + "/prefix", cfg_root))
    LOG_FATAL << "Wrapper can't find the 'prefix' parameter "
        << "in the parameter server. Did you forget define this parameter.";

  root_wrapper_ = Label::make_label(cfg_root,      "wrapper");
  root_robot_   = Label::make_label(root_wrapper_, "robot");

  ///! Setup the ENV for our system
  internal::__setup_env();
}

RobotWrapper::~RobotWrapper() {
  alive_    = false;

  delete regs_stu_;
  regs_stu_ = nullptr;
  // agile_control::MiiControl::instance()->destroy_instance();
  // AutoInstor::destroy_instance();
  // CfgReader::destroy_instance();
}

void RobotWrapper::create_system_singleton() {
  //! class this method in the base class.
  MiiRobot::create_system_singleton();

  internal::__setup_sys(param_ns_);
}

bool RobotWrapper::init() {
  if (!MiiRobot::init())
    LOG_FATAL << "Robot initializes fail!";
  LOG_INFO << "MiiRobot initialization has completed.";

  // Label::printfEveryInstance();
  double frequency = 50.0;
  nh_.getParam("rt_frequency", frequency);
  if (frequency > 0)
    rt_duration_ = std::chrono::milliseconds((int)(1000.0 / frequency));

  reg_robot_states();
  // registry the thread
  ThreadPool::instance()->add("rt-pub",    &RobotWrapper::pub_rt_msg,       this);
  ThreadPool::instance()->add("reg-robot", &RobotWrapper::support_registry, this);

// For debug
#ifdef DEBUG_TOPIC
  cmd_sub_ = nh_.subscribe<std_msgs::Float32>("debug", 100,
      &RobotWrapper::cbForDebug, this);
#endif

  Registry2::instance()->print();
  return true;
}

void RobotWrapper::reg_robot_states() {
  regs_stu_   = new __RegsStu;

  auto jnts   = JointManager::instance();
  auto cfg    = CfgReader::instance();
  auto label  = Label::make_label(root_wrapper_, "registry");
  bool enable = true;
  if (cfg->get_value(label, "enable", enable) && !enable) {
    LOG_WARNING << "The registry is disenabled!";
    return;
  }

  // The default value is registry(exchange between thread)
  std::string reg_type = "registry";
  cfg->get_value(label, "type", reg_type);
  ///! If it is registry2
  if (0 == reg_type.compare("registry2")) {
    regs_stu_->reg_type = REG_2;
    cfg->foreachTag(label, [&](const std::string& _bs) {
      LegType leg = LegType::UNKNOWN_LEG;
      if (!cfg->get_value(_bs, "leg", leg) || (LegType::UNKNOWN_LEG == leg)) {
        LOG_ERROR << "wrong format of " << _bs << ", because NO attr named 'leg'";
        return;
      }

      std::string type, name;
      auto reg = Registry2::instance();
      cfg->foreachTag(_bs, [&](const std::string& _p) {
        if (cfg->get_value(_p, "enable", regs_stu_->reg_legs[leg]) && regs_stu_->reg_legs[leg]
            && cfg->get_value(_p, "type", type)  && cfg->get_value(_p, "name", name)) {
          ///! register the position
          if (0 == type.compare("position")) {
            regs_stu_->joint_positions[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->publish(name, regs_stu_->joint_positions [leg]);
            ///! register the velocities
          } else if (0 == type.compare("velocity")) {
            regs_stu_->joint_velocities[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->publish(name, regs_stu_->joint_velocities [leg]);
            ///! register the torques
          } else if (0 == type.compare("torque")) {
            regs_stu_->joint_torques[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->publish(name, regs_stu_->joint_torques [leg]);
            ///! register the tdlo sensor
          } else if (0 == type.compare("data_tdlo")) {
            std::string l;
            cfg->get_value(_p, "label", l);
            auto td = Label::getHardwareByName<ForceSensor>(l);
            if (nullptr == td) {
              LOG_ERROR << "Can't find the TDLO sensor from Label's System using"
                  << " the given label '" << l << "'";
              return;
            }
            reg->publish(name, td->force_data_const_pointer());
            ///! register the joint command
          } else if (0 == type.compare("command")) {
            regs_stu_->joint_command_flags[leg] = new std::atomic_bool(false);
            regs_stu_->joint_commands[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->subscribe(name, regs_stu_->joint_commands[leg], regs_stu_->joint_command_flags[leg]);
            ///! register the joint limit
          } else if (0 == type.compare("limit")) {
            regs_stu_->joint_limitss[leg] = new Eigen::MatrixXd((int)JntType::N_JNTS, 2);
            jnts->foreach(leg, [&](MiiPtr<Joint>& jnt) {
              regs_stu_->joint_limitss[leg]->row((int)jnt->joint_type())
                  << jnt->joint_position_min(), jnt->joint_position_max();
            });
            reg->publish(name, regs_stu_->joint_limitss[leg]);
          } else {
            ; // Nothing to do here.
          }
        }
        // ;
      });
    });
  } else { ///! The others mean using the registry.
    regs_stu_->reg_type = REG_1;
    cfg->foreachTag(label, [&](const std::string& _bs) {
      LegType leg = LegType::UNKNOWN_LEG;
      if (!cfg->get_value(_bs, "leg", leg) || (LegType::UNKNOWN_LEG == leg)) {
        LOG_ERROR << "wrong format of " << _bs << ", because NO attr named 'leg'";
        return;
      }

      std::string type, name;
      auto reg = Registry2::instance();
      cfg->foreachTag(_bs, [&](const std::string& _p) {
        if (cfg->get_value(_p, "enable", regs_stu_->reg_legs[leg]) && regs_stu_->reg_legs[leg]
            && cfg->get_value(_p, "type", type)  && cfg->get_value(_p, "name", name)) {
          ///! register the position
          if (0 == type.compare("position")) {
            regs_stu_->joint_positions[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->publish(name, regs_stu_->joint_positions [leg]);
            ///! register the velocities
          } else if (0 == type.compare("velocity")) {
            regs_stu_->joint_velocities[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->publish(name, regs_stu_->joint_velocities [leg]);
            ///! register the torques
          } else if (0 == type.compare("torque")) {
            regs_stu_->joint_torques[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->publish(name, regs_stu_->joint_torques [leg]);
            ///! register the tdlo sensor
          } else if (0 == type.compare("data_tdlo")) {
            std::string l;
            cfg->get_value(_p, "label", l);
            auto td = Label::getHardwareByName<ForceSensor>(l);
            if (nullptr == td) {
              LOG_ERROR << "Can't find the TDLO sensor from Label's System using"
                  << " the given label '" << l << "'";
              return;
            }
            reg->publish(name, td->force_data_const_pointer());
            ///! register the joint command
          } else if (0 == type.compare("command")) {
            regs_stu_->joint_command_flags[leg] = new std::atomic_bool(false);
            regs_stu_->joint_commands[leg] = new Eigen::VectorXd((int)JntType::N_JNTS);
            reg->subscribe(name, regs_stu_->joint_commands[leg], regs_stu_->joint_command_flags[leg]);
            ///! register the joint limit
          } else if (0 == type.compare("limit")) {
            regs_stu_->joint_limitss[leg] = new Eigen::MatrixXd((int)JntType::N_JNTS, 2);
            jnts->foreach(leg, [&](MiiPtr<Joint>& jnt) {
              regs_stu_->joint_limitss[leg]->row((int)jnt->joint_type())
                  << jnt->joint_position_min(), jnt->joint_position_max();
            });
            reg->publish(name, regs_stu_->joint_limitss[leg]);
          } else {
            ; // Nothing to do here.
          }
        }
        // ;
      });
    });
//    cfg->foreachTag(label, [&](const std::string& _p) {
//      bool enable = true;
//      std::string type, name;
//      cfg->foreachTag(_bs, [&](const std::string& _p) {
//        if (cfg->get_value(_p, "enable", enable) && enable
//            && cfg->get_value(_p, "type", type)
//            && cfg->get_value(_p, "name", name)) {
//          if (0 == type.compare("data_pos")) {
//            REG_RESOURCE(name, joint_status + JntDataType::POS);
//          } else if (0 == type.compare("data_vel")) {
//            REG_RESOURCE(name, joint_status + JntDataType::VEL);
//          } else if (0 == type.compare("data_tor")) {
//            REG_RESOURCE(name, joint_status + JntDataType::TOR);
//          } else if (0 == type.compare("data_tdlo")) {
//            ; // REG_RESOURCE(name, );
//          } else if (0 == type.compare("command")) {
//            REG_COMMAND(name, &joint_command, &joint_command_flag);
//          } else if (0 == type.compare("limit")) {
//            ;
//          } else {
//            ; // Nothing to do here.
//          }
//        }
//        // ;
//      });
//    });
  }
}

void RobotWrapper::support_registry() {
  auto jnts = JointManager::instance();

  TICKER_INIT(std::chrono::microseconds);
  while (alive_ && ros::ok()) {
    FOREACH_LEG(l) {
      if (!regs_stu_->reg_legs[l]) continue;
      ///! update the pos/vel/tor
      jnts->foreach(l, [&](MiiPtr<Joint>& jnt) {
        (*regs_stu_->joint_positions[l])((int)jnt->joint_type())  = jnt->joint_position();
        (*regs_stu_->joint_velocities[l])((int)jnt->joint_type()) = jnt->joint_velocity();
        (*regs_stu_->joint_torques[l])((int)jnt->joint_type())    = jnt->joint_torque();
      });

      ///! update the command
      if (regs_stu_->joint_command_flags[l]->load()) {
        jnts->foreach(l, [&](MiiPtr<Joint>& jnt) {
          jnt->updateJointCommand((*regs_stu_->joint_commands[l])[jnt->joint_type()]);
        });

        regs_stu_->joint_command_flags[l]->store(false);
      }
    }

    TICKER_CONTROL(80, std::chrono::microseconds);
  }
}

///! This method publish the real-time message, e.g. "/joint_states", "imu", "foot_force"
void RobotWrapper::pub_rt_msg() {
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

    TICKER_CONTROL(rt_duration_, std::chrono::milliseconds);
  }

}

#ifdef DEBUG_TOPIC
void RobotWrapper::cbForDebug(const std_msgs::Float32ConstPtr& msg) {
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

#define  WRAPPER RobotWrapper
#include "apps/internal/main.cpp"
