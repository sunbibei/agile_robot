/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gz_agile_plugin.h>

#include <foundation/utf.h>
#include <foundation/ipc/msg_queue.h>
#include <foundation/ipc/shared_mem.h>

#include <toolbox/time_control.h>

#define GZ_R_THREAD_NAME ("gz_r")
#define GZ_W_THREAD_NAME ("gz_w")

// #define SAVE_MSG_TO_FILE
#ifdef  SAVE_MSG_TO_FILE
FILE* _msg_fd = nullptr;
#endif

double _g_init_pose[][JntType::N_JNTS] = {
    {+0.00000, +0.50851, -1.14050},
    {+0.00000, +0.50851, -1.14050},
    {+0.00000, -0.50851, +1.14050},
    {+0.00000, -0.50851, +1.14050}
};

using namespace gazebo;
namespace agile_gazebo {
void __parse_jnt_name(const std::string& _n, LegType& _l, JntType& _j);

struct MsgqPacket : public MsgBase {
  Packet pkg;
};

struct LinearParams {
  double scale;
  double offset;
};

GzAgilePlugin::GzAgilePlugin()
  : gazebo::ModelPlugin(),
    world_(nullptr), model(nullptr),
    rw_thread_alive_(false), ipc_(nullptr),
#ifdef USE_SHM
    shm_r_buf_(nullptr),     shm_w_buf_(nullptr),
#endif
    swap_r_buffer_(nullptr), swap_w_buffer_(nullptr) {
  ///! initialize the glog.
//  google::InitGoogleLogging("qr_driver");
//  google::FlushLogFiles(google::GLOG_INFO);
//  FLAGS_colorlogtostderr = true;
  std::cout << "Create the GzAgilePlugin... ..." << std::endl;

  id_2_leg_lut_.resize(MAX_NODE_NUM);
  for (auto& lut : id_2_leg_lut_) {
    lut = LegType::UNKNOWN_LEG;
  }

  leg_2_id_lut_.resize(LegType::N_LEGS);
  for (auto& lut : leg_2_id_lut_) {
    lut = INVALID_BYTE;
  }

  for (auto& p : linear_params_) {
    p = nullptr;
  }
}

GzAgilePlugin::~GzAgilePlugin() {
  std::cout << "Deconstructing the GzAgilePlugin... ..." << std::endl;
  // Disconnect from gazebo events
  updateConnection.reset();

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



  rw_thread_alive_ = false;
  middleware::ThreadPool::instance()->stop();
  middleware::ThreadPool::destroy_instance();

  delete swap_r_buffer_;
  delete swap_w_buffer_;
  swap_r_buffer_ = nullptr;
  swap_w_buffer_ = nullptr;

  for (auto& p : linear_params_) {
    delete[] p;
    p = nullptr;
  }

  std::cout << "Deconstructed  the GzAgilePlugin... ..." << std::endl;
  // google::ShutdownGoogleLogging();
}

void GzAgilePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // std::cout << "Loading... 0.0.3" << std::endl;
  // Store the pointer to the model
  model   = _parent;
  world_  = _parent->GetWorld();

  joints_.resize(LegType::N_LEGS);
  for (auto& leg : joints_)
    leg.resize(JntType::N_JNTS);

  LegType leg = LegType::UNKNOWN_LEG;
  JntType jnt = JntType::UNKNOWN_JNT;
  auto _js_ptr = model->GetJoints();
  for (auto& _j : _js_ptr) {
    __parse_jnt_name(_j->GetName(), leg, jnt);
    if ((LegType::UNKNOWN_LEG == leg) || (JntType::UNKNOWN_JNT == jnt)) {
      std::cout << "ERROR JOINT NAME (" << _j->GetName() << "), "
        << "ignore the joint..." << std::endl;
      continue;
    }

    std::cout << "Parse the joint[" << JNTTYPE_TOSTRING(jnt)
      << "]\tfrom leg[" << LEGTYPE_TOSTRING(leg) << "]." << std::endl;
    joints_[leg][jnt] = _j;
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
  id_2_leg_lut_.resize(MAX_NODE_NUM);
  leg_2_id_lut_.resize(LegType::N_LEGS);
  FOR_EACH_LEG(l) {
    std::string _tag = LEGTYPE_TOSTRING(l);
    _tag += "_node_id";
    auto _attr = cfg->GetElement(_tag);
    if (nullptr == _attr) {
      std::cout << "Could not found the '" << _tag << "' tag from _sdf" << std::endl;
      gzthrow("Could not found the '" << _tag << "' tag from _sdf");
      return;
    }

    std::string _val = _attr->Get<std::string>();
    // std::cout << _val << std::endl;
    unsigned int _val_id = 0;
    if (('0' == _val[0]) && ('x' == _val[1])) {
      // Hex to id
      sscanf(_val.c_str(), "0x%x", &_val_id);
    } else {
      sscanf(_val.c_str(), "%d",   &_val_id);
    }
    id_2_leg_lut_[_val_id] = l;
    leg_2_id_lut_[l]       = _val_id;
  }

  // initialize linear_params_ from _sdf
  FOR_EACH_LEG(l) {
    linear_params_[l] = new LinearParams[JntType::N_JNTS];
    FOR_EACH_JNT(j) {
      std::string _tag = LEGTYPE_TOSTRING(l);
      _tag += "_";
      _tag += JNTTYPE_TOSTRING(j);
      double alpha = cfg->GetElement(_tag + "_linear_scale")->Get<double>();
      double beta  = cfg->GetElement(_tag + "_linear_offset")->Get<double>();
      linear_params_[l][j].scale  = alpha * 0.001533981;
      linear_params_[l][j].offset = alpha * beta * -0.000174528;
    }
  }

  if (false) {
    std::cout << "The configure of joint list as follow:" << std::endl;
    FOR_EACH_LEG(l) {
      printf("%s[0x%02X]:\n", LEGTYPE_TOSTRING(l), leg_2_id_lut_[l]);
      FOR_EACH_JNT(j) {
        printf("\t%s\t-> %+11.8f : %+8.5f\n", joints_[l][j]->GetName().c_str(),
            linear_params_[l][j].scale, linear_params_[l][j].offset);
      }
    }
  }
  // initialize the msgq names
  cmd_ipc_name_        = cfg->GetElement("cmd_ipc_name")->Get<std::string>();
  leg_node_ipc_name_   = cfg->GetElement("leg_node_ipc_name")->Get<std::string>();
  std::cout << "The names of IPCs: " << cmd_ipc_name_ << " " << leg_node_ipc_name_ << std::endl;
  ///! launch to the read/write thread.
  swap_r_buffer_   = new boost::lockfree::queue<Packet>(1024);
  swap_w_buffer_   = new boost::lockfree::queue<Packet>(1024);
  rw_thread_alive_ = true;
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
  auto thread_pool = middleware::ThreadPool::create_instance();
  thread_pool->add(GZ_R_THREAD_NAME, &GzAgilePlugin::msg_r_update, this);
  thread_pool->add(GZ_W_THREAD_NAME, &GzAgilePlugin::msg_w_update, this);
  thread_pool->start(GZ_R_THREAD_NAME);
  thread_pool->start(GZ_W_THREAD_NAME);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GzAgilePlugin::event_update, this));

#ifdef SAVE_MSG_TO_FILE
  _msg_fd = fopen("/home/bibei/Workspaces/agile_ws/src/agile_robot/agile-apps/config/gz", "w+");
#endif
  std::cout << "It has Load the GzPropagateG... ..." << std::endl;
}

// Called by the world update start event
void GzAgilePlugin::event_update() {
  // Apply a small linear velocity to the model.
//  this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
//  return;

//  static size_t count = 0;
//  if (0 == ++count%10000)
//    std::cout << "update ..." << std::endl;
  FOR_EACH_LEG(l) {
    FOR_EACH_JNT(j) {
      joints_[l][j]->SetPosition(0, _g_init_pose[l][j]);
    }
  }
  return;

  if (!rw_thread_alive_) return;

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

void GzAgilePlugin::msg_w_update() {
#ifdef USE_SHM
  Packet _pkg;
  size_t _start = 0;
#else
  MsgqPacket _pkg;
  _pkg.msg_id = 0x02; // means from gazebo plugin
#endif

  TIMER_INIT
  while (rw_thread_alive_) {
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

void GzAgilePlugin::msg_r_update() {
#ifdef USE_SHM
  Packet _pkg;
  size_t _start = 0;
#else
  MsgqPacket _pkg;
#endif

  TIMER_INIT
  while (rw_thread_alive_) {
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
inline bool GzAgilePlugin::write(const Packet& pkt) {
  return swap_w_buffer_->push(pkt);
}
/**
 * @brief This method must be implemented by subclass, and convert the message
 *        which received from the specific communication type to the form of Packet,
 * @param pkt[out] The Packet object converted from the message
 * @return Return true if read successful, or return false
 */
inline bool GzAgilePlugin::read(Packet& pkt) {
  return swap_r_buffer_->pop(pkt);
}


//const unsigned char g_TEST_MSG[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
//const short g_TEST_COUNT[] = {+2142, +2797, +1638};
void GzAgilePlugin::__update_robot_stats() {

  double poss[JntType::N_JNTS]  = {0};
//  short counts[JntType::N_JNTS] = {0};

  Packet pkt;
  FOR_EACH_LEG(leg) {
    pkt = {INVALID_BYTE, leg_2_id_lut_[leg], MII_MSG_HEARTBEAT_MSG_1, 8, {0}};
    double pos  = 0;
    short count = 0;
    int offset  = 0;
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      pos   = joints_[leg][jnt]->Position();
      count = (pos - linear_params_[leg][jnt].offset) / linear_params_[leg][jnt].scale;
      //if (true && LegType::FL == leg) count = g_TEST_COUNT[jnt];
      memcpy(pkt.data + offset, &count, sizeof(short));

      poss[jnt]   = pos;
//      counts[jnt] = count;
      offset += sizeof(short);
    }
//    if (false && LegType::FL == leg)
//      memcpy(pkt.data, g_TEST_MSG, sizeof(g_TEST_MSG)*sizeof(unsigned char));
    if (false && LegType::FL == leg) {
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
    if (false && LegType::FL == leg)
      fprintf(_msg_fd, "%s - %+5d, %+5d, %+5d\n", LEGTYPE_TOSTRING(leg),
          counts[JntType::KNEE], counts[JntType::HIP], counts[JntType::YAW]);
#else
//    if (false && LegType::FL == leg)
//      printf("%s - %+5d, %+5d, %+5d\n", LEGTYPE_TOSTRING(leg),
//          counts[JntType::KNEE], counts[JntType::HIP], counts[JntType::YAW]);
#endif

    if (true && LegType::FL == leg)
      printf("%s - %+8.5f, %+8.5f, %+8.5f\n", LEGTYPE_TOSTRING(leg),
          poss[JntType::KNEE], poss[JntType::HIP], poss[JntType::YAW]);
    ///! write the robot state into the message queue.
    write(pkt);
  }
}

void GzAgilePlugin::__parse_command_pkg(const Packet& pkt) {
  if (false) {
    printf("%s", (std::string(__FILE__).substr(std::string(__FILE__).rfind('/')+1) + ":" + std::to_string(__LINE__)).c_str());
    printf(" -> NODE ID:0x%02X MSG ID: 0%02ou LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)pkt.node_id, (int)pkt.msg_id,  (int)pkt.size,
          (int)pkt.data[0], (int)pkt.data[1], (int)pkt.data[2], (int)pkt.data[3],
          (int)pkt.data[4], (int)pkt.data[5], (int)pkt.data[6], (int)pkt.data[7]);
  }

  switch (pkt.msg_id) {
  case MII_MSG_COMMON_DATA_1: // JOINT POS CMD
  case MII_MSG_COMMON_DATA_2: // JOINT VEL CMD
  case MII_MSG_COMMON_DATA_3: // JOINT TOR CMD
    __write_command_to_sim(pkt);
    break;
  case MII_MSG_COMMON_DATA_4: // POS-VEL CMD (knee and hip)
  case MII_MSG_COMMON_DATA_5: // POS-VEL CMD (yaw)
  case MII_MSG_MOTOR_CMD_1:   // MOTOR VEL CMD
  case MII_MSG_MOTOR_CMD_2:   // MOTOR VEL CMD
  case MII_MSG_MOTOR_CMD_3:   // MOTOR TOR CMD
    LOG_ERROR << "Using the NO IMPLEMENTED mode of control.";
    break;
  }
}

void GzAgilePlugin::__write_command_to_sim(const Packet& pkt) {
  static const unsigned int JNT_CMD_SIZE = 6;
  if (JNT_CMD_SIZE != pkt.size) {
    gzerr << "The data size of MII_MSG_COMMON_DATA_3 message does not match!"
        << ", the expect size is 6, but the real size is " << pkt.size << "\n";
    return;
  }

  double  cmd = 0;
  short count = 0;
  int offset  = 0;
  LegType leg = id_2_leg_lut_[pkt.node_id];

  double cmds[JntType::N_JNTS]  = {0};
  short counts[JntType::N_JNTS] = {0};
  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    memcpy(&count, pkt.data + offset, sizeof(count));
    cmd = linear_params_[leg][type].scale * count + linear_params_[leg][type].offset;
    cmds[type]   = cmd;
    counts[type] = count;
    switch (pkt.msg_id) {
    case MII_MSG_COMMON_DATA_1: joints_[leg][type]->SetPosition(0, cmd); break;
    case MII_MSG_COMMON_DATA_2: joints_[leg][type]->SetVelocity(0, cmd); break;
    case MII_MSG_COMMON_DATA_3: joints_[leg][type]->SetForce(0,    cmd); break;
    default:
      gzerr << "ERROR msg_id";
    }

    offset += sizeof(count);
  }
  if (false && LegType::FL == leg)
    printf("%s - %+5d, %+5d, %+5d\n", LEGTYPE_TOSTRING(leg),
        counts[JntType::KNEE], counts[JntType::HIP], counts[JntType::YAW]);

  if (false && LegType::FL == leg)
    printf("%s - %+8.5f, %+8.5f, %+8.5f\n", LEGTYPE_TOSTRING(leg),
        cmds[JntType::KNEE], cmds[JntType::HIP], cmds[JntType::YAW]);
}

inline void __parse_jnt_name(const std::string& _n, LegType& _l, JntType& _j) {
    if (std::string::npos != _n.find("fl")) {
        _l = LegType::FL;
    } else if (std::string::npos != _n.find("fr")) {
        _l = LegType::FR;
    } else if (std::string::npos != _n.find("hl")) {
        _l = LegType::HL;
    } else if (std::string::npos != _n.find("hr")) {
        _l = LegType::HR;
    } else {
        _l = LegType::UNKNOWN_LEG;
    }

    if (std::string::npos != _n.find("yaw")) {
        _j = JntType::YAW;
    } else if (std::string::npos != _n.find("hip")) {
        _j = JntType::HIP;
    } else if (std::string::npos != _n.find("knee")) {
        _j = JntType::KNEE;
    } else {
        _j = JntType::UNKNOWN_JNT;
    }
}

} /* end namespace gazebo */

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(agile_gazebo::GzAgilePlugin)
