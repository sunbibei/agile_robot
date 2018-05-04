/*
 * gz_agile_leg_plugin.cpp
 *
 *  Created on: Mar 7, 2018
 *      Author: bibei
 */

#include <gz_agile_leg_plugin.h>

#include <foundation/utf.h>
#include <foundation/ipc/msg_queue.h>
#include <foundation/ipc/shared_mem.h>

#include <toolbox/time_control.h>
#include <toolbox/pid.h>

#include <atomic>

#define GZ_R_THREAD_NAME   ("gz_r")
#define GZ_W_THREAD_NAME   ("gz_w")
#define GZ_PID_THREAD_NAME ("gz_pid")

// #define SAVE_MSG_TO_FILE
#ifdef  SAVE_MSG_TO_FILE
FILE* _msg_fd = nullptr;
#endif

double _g_init_pose[] = {0, 0.50851, -1.14050};

using namespace gazebo;
namespace agile_gazebo {
void __parse_jnt_name(const std::string& _n, JntType& _j);

struct MsgqPacket : public MsgBase {
  Packet pkg;
};

struct LinearParams {
  double scale;
  double offset;
};

struct JntTarget {
  JntCmdType       type;
  double           value;
  std::atomic_bool is_new;
};

GzAgileLegPlugin::GzAgileLegPlugin()
  : gazebo::ModelPlugin(),
    world_(nullptr), model(nullptr),
    thread_alive_(false), ipc_(nullptr),
#ifdef USE_SHM
    shm_r_buf_(nullptr),     shm_w_buf_(nullptr),
#endif
    swap_r_buffer_(nullptr), swap_w_buffer_(nullptr),
    leg_type_(LegType::UNKNOWN_LEG), leg_id_(INVALID_BYTE),
    linear_params_(nullptr) {
  ///! initialize the glog.
//  google::InitGoogleLogging("qr_driver");
//  google::FlushLogFiles(google::GLOG_INFO);
//  FLAGS_colorlogtostderr = true;
  std::cout << "Create the GzAgilePlugin... ..." << std::endl;
}

GzAgileLegPlugin::~GzAgileLegPlugin() {
  std::cout << "Deconstructing the GzAgilePlugin... ..." << std::endl;
  thread_alive_ = false;
  agile_robot::ThreadPool::instance()->stop();
  agile_robot::ThreadPool::destroy_instance();

  ipc_->destroy_instance();
//  Packet pkg;
//  int count = 0;
//  while (!swap_w_buffer_->empty()) {
//    swap_w_buffer_->pop(pkg);
//    ++count;
//  }
//  printf("W SIZE: %d\n", count);
//
//  count = 0;
//  while (!swap_r_buffer_->empty()) {
//    swap_r_buffer_->pop(pkg);
//    ++count;
//  }
//  printf("R SIZE: %d\n", count);

  for (auto& p : joints_control_) {
    delete p;
    p = nullptr;
  }

  for (auto& p : joints_target_) {
    delete p;
    p = nullptr;
  }

  delete swap_r_buffer_;
  delete swap_w_buffer_;
  swap_r_buffer_ = nullptr;
  swap_w_buffer_ = nullptr;

  delete linear_params_;
  linear_params_ = nullptr;

  // Disconnect from gazebo events
  updateConnection.reset();
  std::cout << "Deconstructed  the GzAgilePlugin... ..." << std::endl;
  // google::ShutdownGoogleLogging();
}

void GzAgileLegPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // std::cout << "Loading... 0.0.3" << std::endl;
  // Store the pointer to the model
  model   = _parent;
  world_  = _parent->GetWorld();

  joints_.resize(JntType::N_JNTS);
  JntType jnt = JntType::UNKNOWN_JNT;
  auto _js_ptr = model->GetJoints();
  for (auto& _j : _js_ptr) {
    __parse_jnt_name(_j->GetName(), jnt);
    if (JntType::UNKNOWN_JNT == jnt) {
      std::cout << "IT NOT JOINT NAME OF LEG (" << _j->GetName() << "), "
        << "ignore the joint..." << std::endl;
      continue;
    }

    std::cout << "Parse the joint[" << JNTTYPE2STR(jnt) << "]" << std::endl;
    joints_[jnt] = _j;
  }

  if (!_sdf->HasElement("config")) {
    std::cout << "Could not found the 'config' tag from _sdf" << std::endl;
    // gzthrow("Could not found the 'config' tag from _sdf");
    return;
  }
  std::cout << "Found the 'config' tag." << std::endl;
  auto cfg = _sdf->GetElement("config");
  // initialize id_2_leg_lut_  from _sdf
  // initialize leg_2_id_lut_  from _sdf
  std::string _tag = "leg_type";
  auto _attr = cfg->GetElement(_tag);
  if (nullptr == _attr) {
    std::cout << "Could not found the '" << _tag << "' tag from _sdf" << std::endl;
    gzthrow("Could not found the '" << _tag << "' tag from _sdf");
    return;
  }
  std::string _val = _attr->Get<std::string>();

  if (std::string::npos != _val.find("FL")) {
    leg_type_ = LegType::FL;
  } else if (std::string::npos != _val.find("FR")) {
    leg_type_ = LegType::FR;
  } else if (std::string::npos != _val.find("HL")) {
    leg_type_ = LegType::HL;
  } else if (std::string::npos != _val.find("HR")) {
    leg_type_ = LegType::HR;
  } else {
    leg_type_ = LegType::UNKNOWN_LEG;
  }

  _tag = "node_id";
  _attr = cfg->GetElement(_tag);
  if (nullptr == _attr) {
    std::cout << "Could not found the '" << _tag << "' tag from _sdf" << std::endl;
    gzthrow("Could not found the '" << _tag << "' tag from _sdf");
    return;
  }
  _val = _attr->Get<std::string>();
  unsigned int _val_id = 0;
  if (('0' == _val[0]) && ('x' == _val[1])) {
    // Hex to id
    sscanf(_val.c_str(), "0x%x", &_val_id);
  } else {
    sscanf(_val.c_str(), "%d",   &_val_id);
  }
  leg_id_ = _val_id;

  // initialize linear_params_ from _sdf
  linear_params_ = new LinearParams[JntType::N_JNTS];
  FOREACH_JNT(j) {
    _tag = JNTTYPE2STR(j);
    double alpha = cfg->GetElement(_tag + "_linear_scale")->Get<double>();
    double beta  = cfg->GetElement(_tag + "_linear_offset")->Get<double>();
    linear_params_[j].scale  = alpha * 0.001533981;
    linear_params_[j].offset = alpha * beta * -0.000174528;
  }

  joints_control_.resize(JntType::N_JNTS);
  joints_target_.resize(JntType::N_JNTS);
  // for (const auto& j : {JntType::HIP, JntType::KNEE}){
  FOREACH_JNT(j) {
    _tag = JNTTYPE2STR(j);
    auto gains_str = cfg->GetElement(_tag + "_pid_gains")->Get<std::string>();
    std::stringstream ss;
    ss << gains_str;
    double kp, ki, kd;
    ss >> kp >> ki >> kd;

    joints_target_[j]  = new JntTarget;
    joints_control_[j] = new Pid(_tag + "_pid");
    joints_control_[j]->gains(kp, ki, kd);
  }

  if (false) {
    std::cout << "The configure of joint list as follow:" << std::endl;
    printf("%s[0x%02X]:\n", LEGTYPE2STR(leg_type_), leg_id_);
    FOREACH_JNT(j) {
      printf("\t%s\t-> %+11.8f : %+8.5f\n", joints_[j]->GetName().c_str(),
          linear_params_[j].scale, linear_params_[j].offset);
    }
  }
  // initialize the msgq names
  cmd_ipc_name_        = cfg->GetElement("cmd_ipc_name")->Get<std::string>();
  leg_node_ipc_name_   = cfg->GetElement("leg_node_ipc_name")->Get<std::string>();
  std::cout << "The names of IPCs: " << cmd_ipc_name_ << " " << leg_node_ipc_name_ << std::endl;
  ///! launch to the read/write thread.
  swap_r_buffer_   = new boost::lockfree::queue<Packet>(1024);
  swap_w_buffer_   = new boost::lockfree::queue<Packet>(1024);
  thread_alive_ = true;
  ///! Choice the style of IPC
#ifdef USE_SHM
  ///! The follow is Shared Memory.
  ipc_ = SharedMem::create_instance();
  ipc_->create_shm(leg_node_ipc_name_, LegType::N_LEGS*sizeof(Packet));
  ipc_->create_shm(cmd_ipc_name_,      LegType::N_LEGS*sizeof(Packet));
  shm_r_buf_ = ipc_->get_addr_from_shm(cmd_ipc_name_);
  shm_w_buf_ = ipc_->get_addr_from_shm(leg_node_ipc_name_);
#else
  ///! The follow is Message Queue.
  ipc_            = MsgQueue::create_instance();
  ipc_->create_msgq(leg_node_ipc_name_);
  ipc_->create_msgq(cmd_ipc_name_);
#endif

  ///! Launch threads.
  ///! The msg_r_update send the robot states to the soft system.
  ///! The msg_w_update receive the command from the soft system.
  ///! The joint_control run the pid control to arrival the target position.
  auto thread_pool = agile_robot::ThreadPool::create_instance();
  thread_pool->add(GZ_R_THREAD_NAME,   &GzAgileLegPlugin::msg_r_update,  this);
  thread_pool->add(GZ_W_THREAD_NAME,   &GzAgileLegPlugin::msg_w_update,  this);
  thread_pool->add(GZ_PID_THREAD_NAME, &GzAgileLegPlugin::joint_control, this);
  thread_pool->start(GZ_R_THREAD_NAME);
  thread_pool->start(GZ_W_THREAD_NAME);
  thread_pool->start(GZ_PID_THREAD_NAME);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GzAgileLegPlugin::event_update, this));

#ifdef SAVE_MSG_TO_FILE
  _msg_fd = fopen("/home/bibei/Workspaces/agile_ws/src/agile_robot/agile-apps/config/gz", "w+");
#endif
  std::cout << "It has Load the GzPropagateG... ..." << std::endl;
}

// Called by the world update start event
void GzAgileLegPlugin::event_update() {
  // Apply a small linear velocity to the model.
//  this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
//  return;

//  static size_t count = 0;
//  if (0 == ++count%10000)
//    std::cout << "update ..." << std::endl;
//  FOR_EACH_JNT(j) {
//    joints_[j]->SetPosition(0, _g_init_pose[j]);
//    // joints_[j]->SetPosition(0, -0.32);
//  }
//  ///! The frequency control
//  ///! The 1000/5=200 Hz is the highest frequency of normal communication between agile-driver.
//  static TimeControl _s_post_tick(true);
//  static int64_t     _s_sum_interval  = 0;
//  static int64_t     _s_tick_interval = 5;
//  _s_sum_interval += _s_post_tick.dt();
//  if (_s_sum_interval < _s_tick_interval) return;
//  _s_sum_interval = 0;
//
//  ///! send the robot states.
//  __update_robot_stats();
//  return;

  if (!thread_alive_) return;

  ///! The frequency of responding command is the highest.
  if (read(pkt_rw_))
    __parse_command_pkg(pkt_rw_);

  ///! The frequency control
  ///! The 1000/5=200 Hz is the highest frequency of normal communication between agile-driver.
  static TimeControl _s_post_tick(true);
  static int64_t     _s_sum_interval  = 0;
  static int64_t     _s_tick_interval = 5;
  _s_sum_interval += _s_post_tick.dt();
  if (_s_sum_interval < _s_tick_interval) return;
  _s_sum_interval = 0;

  ///! send the robot states.
  __update_robot_stats();
}

void GzAgileLegPlugin::joint_control() {
  TIMER_INIT

  double _u = 0;
  while (thread_alive_) {
    FOREACH_JNT(j) {
      if ((nullptr == joints_target_[j]) || (nullptr == joints_control_[j]))
        continue;

      joints_[j]->SetPosition(0, joints_target_[j]->value);

      // TODO need to support the velocity and torque command
//      if (joints_target_[j]->is_new.load()) {
//        joints_control_[j]->target(joints_target_[j]->value);
//        joints_target_[j]->is_new.store(false);
//      }
//
//      _u = 0;
//      joints_control_[j]->control(joints_[j]->Position(), _u);
//      joints_[j]->SetVelocity(0, _u);
      // joints_[j]->SetForce(0, _u);
    }

    TIMER_CONTROL(2);
  }
}

void GzAgileLegPlugin::msg_w_update() {
#ifdef USE_SHM
  Packet _pkg;
  size_t _start = 0;
#else
  MsgqPacket _pkg;
  _pkg.msg_id = 0x02; // means from gazebo plugin
#endif

  TIMER_INIT
  while (thread_alive_) {
    if (!swap_w_buffer_->empty()) {
#ifdef USE_SHM
      swap_w_buffer_->pop(_pkg);
      _start = id_2_leg_lut_[_pkg.node_id] * sizeof(Packet);
      memcpy(shm_w_buf_ + _start, &_pkg, sizeof(Packet));
//      if (ipc_->write_to_shm(leg_node_msgq_name_, _pkg, start)) {
//        std::cout << "Send the command fail." << std::endl;
//      }
      if (false && 0x02u == _pkg.node_id)
        printf("  -> NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)_pkg.node_id,
          (int)_pkg.msg_id,  (int)_pkg.size,
          (int)_pkg.data[0], (int)_pkg.data[1],
          (int)_pkg.data[2], (int)_pkg.data[3],
          (int)_pkg.data[4], (int)_pkg.data[5],
          (int)_pkg.data[6], (int)_pkg.data[7]);
#else
      swap_w_buffer_->pop(_pkg.pkg);
      if (!ipc_->write_to_msgq(leg_node_ipc_name_, &_pkg, sizeof(_pkg))) {
        std::cout << "Send the command fail." << std::endl;
      }
      if (false)
        printf("  <- T  O:0x%02X NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)_pkg.msg_id,      (int)_pkg.pkg.node_id,
          (int)_pkg.pkg.msg_id,  (int)_pkg.pkg.size,
          (int)_pkg.pkg.data[0], (int)_pkg.pkg.data[1],
          (int)_pkg.pkg.data[2], (int)_pkg.pkg.data[3],
          (int)_pkg.pkg.data[4], (int)_pkg.pkg.data[5],
          (int)_pkg.pkg.data[6], (int)_pkg.pkg.data[7]);
#endif
    }

    TIMER_CONTROL(1)
  }
}

void GzAgileLegPlugin::msg_r_update() {
#ifdef USE_SHM
  Packet _pkg;
  size_t _start = 0;
#else
  MsgqPacket _pkg;
#endif

  TIMER_INIT
  while (thread_alive_) {
#ifdef USE_SHM
    FOR_EACH_LEG(l) {
      _start = l * sizeof(Packet);
      memcpy(&_pkg, shm_r_buf_ + _start, sizeof(Packet));
      swap_r_buffer_->push(_pkg);
      if (false)
        printf("  <- NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)_pkg.node_id,
          (int)_pkg.msg_id,  (int)_pkg.size,
          (int)_pkg.data[0], (int)_pkg.data[1],
          (int)_pkg.data[2], (int)_pkg.data[3],
          (int)_pkg.data[4], (int)_pkg.data[5],
          (int)_pkg.data[6], (int)_pkg.data[7]);
    }
#else
    if (ipc_->read_from_msgq(cmd_ipc_name_, &_pkg, sizeof(_pkg))) {
      swap_r_buffer_->push(_pkg.pkg);
      if (false)
        printf("  <- FROM:0x%02X NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)_pkg.msg_id,      (int)_pkg.pkg.node_id,
          (int)_pkg.pkg.msg_id,  (int)_pkg.pkg.size,
          (int)_pkg.pkg.data[0], (int)_pkg.pkg.data[1],
          (int)_pkg.pkg.data[2], (int)_pkg.pkg.data[3],
          (int)_pkg.pkg.data[4], (int)_pkg.pkg.data[5],
          (int)_pkg.pkg.data[6], (int)_pkg.pkg.data[7]);
    }
#endif

    TIMER_CONTROL(10)
  }
}

/**
 * @brief This method completes the function that convert the Packet
 *        to the form of message which be needed by the specific communication
 *        type, and writing the converted message to robot.
 * @param pkt[in] The Packet object need to convert and write
 * @return Return true if successful, or return false
 */
inline bool GzAgileLegPlugin::write(const Packet& pkt) {
  return swap_w_buffer_->push(pkt);
}
/**
 * @brief This method must be implemented by subclass, and convert the message
 *        which received from the specific communication type to the form of Packet,
 * @param pkt[out] The Packet object converted from the message
 * @return Return true if read successful, or return false
 */
inline bool GzAgileLegPlugin::read(Packet& pkt) {
  return swap_r_buffer_->pop(pkt);
}


//const unsigned char g_TEST_MSG[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
//const short g_TEST_COUNT[] = {+2142, +2797, +1638};
void GzAgileLegPlugin::__update_robot_stats() {
  const static unsigned char GZ_BUS_ID             = 0x01u;
  const static unsigned char ROBOT_STATES_PKG_SIZE = 0x08u;
  double poss[JntType::N_JNTS]  = {0};
//  short counts[JntType::N_JNTS] = {0};

  Packet pkt = {GZ_BUS_ID, leg_id_, MII_MSG_HEARTBEAT_1, ROBOT_STATES_PKG_SIZE, {0}};
  double pos  = 0;
  short count = 0;
  int offset  = 0;
  for (const auto& jnt : {JntType::KFE, JntType::HFE, JntType::HAA}) {
    pos   = joints_[jnt]->Position();
    if (ignition::math::isnan(pos)) pos = 0;
    count = (pos - linear_params_[jnt].offset) / linear_params_[jnt].scale;
    //if (true && LegType::FL == leg) count = g_TEST_COUNT[jnt];
    memcpy(pkt.data + offset, &count, sizeof(short));

    poss[jnt]   = pos;
//    counts[jnt] = count;
    offset += sizeof(short);
  }
//  if (false && LegType::FL == leg)
//    memcpy(pkt.data, g_TEST_MSG, sizeof(g_TEST_MSG)*sizeof(unsigned char));
  if (false) {
    printf("%s", (std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) + ":" + std::to_string(__LINE__)).c_str());
    printf(" NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)pkt.node_id,
      (int)pkt.msg_id,  (int)pkt.size,
      (int)pkt.data[0], (int)pkt.data[1],
      (int)pkt.data[2], (int)pkt.data[3],
      (int)pkt.data[4], (int)pkt.data[5],
      (int)pkt.data[6], (int)pkt.data[7]);
  }

#ifdef SAVE_MSG_TO_FILE
  if (false)
    fprintf(_msg_fd, "%s - %+5d, %+5d, %+5d\n", LEGTYPE2STR(leg),
        counts[JntType::KFE], counts[JntType::HFE], counts[JntType::HAA]);
#else
//    if (false)
//      printf("%s - %+5d, %+5d, %+5d\n", LEGTYPE_TOSTRING(leg),
//          counts[JntType::KNEE], counts[JntType::HIP], counts[JntType::YAW]);
#endif

  if (false)
    printf(" - [%s]%+8.5f, [%s]%+8.5f, [%s]%+8.5f\n",
        JNTTYPE2STR(JntType::KFE), poss[JntType::KFE],
        JNTTYPE2STR(JntType::HFE), poss[JntType::HFE],
        JNTTYPE2STR(JntType::HAA), poss[JntType::HAA]);
  ///! write the robot state into the message queue.
  write(pkt);
}

void GzAgileLegPlugin::__parse_command_pkg(const Packet& pkt) {
  if (false) {
    printf("%s", (std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) + ":" + std::to_string(__LINE__)).c_str());
    printf(" -> NODE ID:0x%02X MSG ID: 0%02ou LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
          (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
          (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
  }

  switch (pkt.msg_id) {
  case MII_MSG_COMMON_1: // JOINT POS CMD
  case MII_MSG_COMMON_2: // JOINT VEL CMD
  case MII_MSG_COMMON_3: // JOINT TOR CMD
    __write_command_to_sim(pkt);
    break;
  case MII_MSG_COMMON_4: // POS-VEL CMD (knee and hip)
  case MII_MSG_COMMON_5: // POS-VEL CMD (yaw)
  case MII_MSG_MOTOR_1:   // MOTOR VEL CMD
  case MII_MSG_MOTOR_2:   // MOTOR VEL CMD
  case MII_MSG_MOTOR_3:   // MOTOR TOR CMD
    LOG_ERROR << "Using the NO IMPLEMENTED mode of control.";
    break;
  }
}

void GzAgileLegPlugin::__write_command_to_sim(const Packet& pkt) {
  if (pkt.node_id != leg_id_) return;

  static const unsigned int JNT_CMD_SIZE = 6;
  if (JNT_CMD_SIZE != pkt.size) {
    gzerr << "The data size of MII_MSG_COMMON_DATA_3 message does not match!"
        << ", the expect size is 6, but the real size is " << pkt.size << "\n";
    return;
  }

  double  cmd = 0;
  short count = 0;
  int offset  = 0;

  double cmds[JntType::N_JNTS]  = {0};
  short counts[JntType::N_JNTS] = {0};
  for (const auto& type : {JntType::KFE, JntType::HFE, JntType::HAA}) {
    if ((INVALID_BYTE == pkt.data[offset]) && (INVALID_BYTE == pkt.data[offset + 1])) {
      offset += sizeof(count);
      continue;
    }

    memcpy(&count, pkt.data + offset, sizeof(count));
    cmd = linear_params_[type].scale * count + linear_params_[type].offset;
    cmds[type]   = cmd;
    counts[type] = count;
    switch (pkt.msg_id) {
    case MII_MSG_COMMON_1: joints_target_[type]->type = JntCmdType::CMD_POS; break;
    case MII_MSG_COMMON_2: joints_target_[type]->type = JntCmdType::CMD_VEL; break;
    case MII_MSG_COMMON_3: joints_target_[type]->type = JntCmdType::CMD_TOR; break;
    default:
      gzerr << "ERROR msg_id";
      joints_target_[type]->type = JntCmdType::UNKNOWN_CMD_TYPE;
    }
    joints_target_[type]->value  = cmd;
    joints_target_[type]->is_new.store(true);
    // joints_[type]->SetPosition(0, cmd);
    offset += sizeof(count);
  }
  if (false)
    printf(" - %+5d, %+5d, %+5d\n",
        counts[JntType::KFE], counts[JntType::HFE], counts[JntType::HAA]);

  if (true)
    printf("\n -    HAA  ,    HFE  ,    KFE  \n - %+8.5f, %+8.5f, %+8.5f\n\n",
        cmds[JntType::HAA], cmds[JntType::HFE], cmds[JntType::KFE]);
}

inline void __parse_jnt_name(const std::string& _n, JntType& _j) {
    if (std::string::npos != _n.find("haa")) {
        _j = JntType::HAA;
    } else if (std::string::npos != _n.find("hfe")) {
        _j = JntType::HFE;
    } else if (std::string::npos != _n.find("kfe")) {
        _j = JntType::KFE;
    } else {
        _j = JntType::UNKNOWN_JNT;
    }
}

} /* end namespace gazebo */

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(agile_gazebo::GzAgileLegPlugin)

