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

#define GZ_R_THREAD_NAME ("gz_r")
#define GZ_W_THREAD_NAME ("gz_w")

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
    rw_thread_alive_(false), msgq_(nullptr),
    swap_r_buffer_(nullptr),
    swap_w_buffer_(nullptr) {
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
  rw_thread_alive_ = false;
  middleware::ThreadPool::instance()->stop(GZ_R_THREAD_NAME);
  middleware::ThreadPool::instance()->stop(GZ_W_THREAD_NAME);

  // Disconnect from gazebo events
  updateConnection.reset();

  for (auto& p : linear_params_) {
    delete[] p;
    p = nullptr;
  }

  MsgQueue::destroy_instance();
  std::cout << "Deconstructed  the GzAgilePlugin... ..." << std::endl;
  // google::ShutdownGoogleLogging();
}

void GzAgilePlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  std::cout << "Loading... 0.0.3" << std::endl;
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
      linear_params_[l][j].scale  = cfg->GetElement(_tag + "_linear_scale")->Get<double>();
      linear_params_[l][j].offset = cfg->GetElement(_tag + "_linear_offset")->Get<double>();
    }
  }

  std::cout << "The configure of joint list as follow:" << std::endl;
  FOR_EACH_LEG(l) {
    printf("%s[0x%02X]:\n", LEGTYPE_TOSTRING(l), leg_2_id_lut_[l]);
    FOR_EACH_JNT(j) {
      printf("\t%s\t-> %+5.1f : %+8.1f\n", joints_[l][j]->GetName().c_str(),
          linear_params_[l][j].scale, linear_params_[l][j].offset);
    }
  }
  // initialize the msgq names
  cmd_msgq_name_        = cfg->GetElement("cmd_msgq_name")->Get<std::string>();
  leg_node_msgq_name_   = cfg->GetElement("leg_node_msgq_name")->Get<std::string>();
  std::cout << "The names of msgq: " << cmd_msgq_name_ << " " << leg_node_msgq_name_ << std::endl;
  ///! launch to the read/write thread.
  swap_r_buffer_   = new boost::lockfree::queue<Packet>(1024);
  swap_w_buffer_   = new boost::lockfree::queue<Packet>(1024);
  rw_thread_alive_ = true;
  msgq_            = MsgQueue::create_instance();
  auto thread_pool = middleware::ThreadPool::create_instance();
  msgq_->create_msgq(leg_node_msgq_name_);
  msgq_->create_msgq(cmd_msgq_name_);
  thread_pool->add(GZ_R_THREAD_NAME, &GzAgilePlugin::msg_r_update, this);
  thread_pool->add(GZ_W_THREAD_NAME, &GzAgilePlugin::msg_w_update, this);
  thread_pool->start(GZ_R_THREAD_NAME);
  thread_pool->start(GZ_W_THREAD_NAME);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GzAgilePlugin::event_update, this));

  std::cout << "Has Load the GzPropagateG... ..." << std::endl;
}

// Called by the world update start event
void GzAgilePlugin::event_update() {
  // Apply a small linear velocity to the model.
//  this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
//  return;

  static size_t count = 0;
  if (0 == ++count%1000)
    std::cout << "update ..." << std::endl;

//  Packet pkt;
//  if (read(pkt)) {
//    __parse_command_pkg(pkt);
//  }

  __update_robot_stats();
}

void GzAgilePlugin::msg_w_update() {
  MsgqPacket _pkg;
  _pkg.msg_id = 0x02; // means from gazebo plugin

  TIMER_INIT
  while (rw_thread_alive_) {
    if (!swap_w_buffer_->empty()) {
      swap_w_buffer_->pop(_pkg.pkg);
      if (false)
        printf("  -> FROM:0x%02X NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)_pkg.msg_id,      (int)_pkg.pkg.node_id,
          (int)_pkg.pkg.msg_id,  (int)_pkg.pkg.size,
          (int)_pkg.pkg.data[0], (int)_pkg.pkg.data[1],
          (int)_pkg.pkg.data[2], (int)_pkg.pkg.data[3],
          (int)_pkg.pkg.data[4], (int)_pkg.pkg.data[5],
          (int)_pkg.pkg.data[6], (int)_pkg.pkg.data[7]);

      if (msgq_->write_to_msgq(leg_node_msgq_name_, &_pkg, sizeof(_pkg))) {
        std::cout << "Send the command fail." << std::endl;
      }
    }

    TIMER_CONTROL(10)
  }
}

void GzAgilePlugin::msg_r_update() {
  MsgqPacket _pkg;

  TIMER_INIT
  while (rw_thread_alive_) {
    if (msgq_->read_from_msgq(cmd_msgq_name_, &_pkg, sizeof(_pkg))) {
      swap_r_buffer_->push(_pkg.pkg);
      if (true)
        printf("  <- FROM:0x%02X NODE_ID:0x%02X MSG_ID:0x%02X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)_pkg.msg_id,      (int)_pkg.pkg.node_id,
          (int)_pkg.pkg.msg_id,  (int)_pkg.pkg.size,
          (int)_pkg.pkg.data[0], (int)_pkg.pkg.data[1],
          (int)_pkg.pkg.data[2], (int)_pkg.pkg.data[3],
          (int)_pkg.pkg.data[4], (int)_pkg.pkg.data[5],
          (int)_pkg.pkg.data[6], (int)_pkg.pkg.data[7]);
    }

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
bool GzAgilePlugin::write(const Packet& pkt) {
  swap_w_buffer_->push(pkt);
  return true;
}
/**
 * @brief This method must be implemented by subclass, and convert the message
 *        which received from the specific communication type to the form of Packet,
 * @param pkt[out] The Packet object converted from the message
 * @return Return true if read successful, or return false
 */
bool GzAgilePlugin::read(Packet& pkt) {
  if (swap_r_buffer_->empty()) return false;

  swap_r_buffer_->pop(pkt);
  return true;
}

void GzAgilePlugin::__update_robot_stats() {
  Packet pkt;
  FOR_EACH_LEG(leg) {
    pkt = {INVALID_BYTE, leg_2_id_lut_[leg], MII_MSG_HEARTBEAT_MSG_1, 8, {0}};
    double pos  = 0;
    short count = 0;
    int offset  = 0;
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      pos   = joints_[leg][jnt]->Position(0);
      count = (pos - linear_params_[leg][jnt].offset) / linear_params_[leg][jnt].scale;
      memcpy(pkt.data + offset, &count, sizeof(short));

      offset += sizeof(short);
    }
    ///! write the robot state into the message queue.
    write(pkt);
  }
}

void GzAgilePlugin::__parse_command_pkg(const Packet& pkt) {
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
  LegType leg = id_2_leg_lut_[pkt.node_id];
  double  cmd = 0;
  short count = 0;
  int offset  = 0;
  if (6 != pkt.size) {
    gzerr << "The data size of MII_MSG_COMMON_DATA_3 message does not match!"
        << ", the expect size is 6, but the real size is " << pkt.size << "\n";
    return;
  }

  for (const auto& type : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
    memcpy(&count, pkt.data + offset, sizeof(count));
    cmd = linear_params_[leg][type].scale * count + linear_params_[leg][type].offset;

    switch (pkt.msg_id) {
    case MII_MSG_COMMON_DATA_1: joints_[leg][type]->SetPosition(0, cmd); break;
    case MII_MSG_COMMON_DATA_2: joints_[leg][type]->SetVelocity(0, cmd); break;
    case MII_MSG_COMMON_DATA_3: joints_[leg][type]->SetForce(0, cmd);    break;
    default:
      gzerr << "ERROR msg_id";
    }

    offset += sizeof(count);
  }
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
