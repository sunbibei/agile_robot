/*
 * gazebo_propagate.cpp
 *
 *  Created on: Jan 30, 2018
 *      Author: bibei
 */

#include "gazebo_propagate.h"
#include <gazebo/sensors/sensors.hh>

#include <functional>

#include <foundation/utf.h>
#include <foundation/ipc/msg_queue.h>

#define GZ_R_THREAD ("gz_r")
#define GZ_W_THREAD ("gz_w")

void __parse_jnt_name(const std::string& _n, LegType& _l, JntType& _j);

using namespace gazebo;

namespace agile_gazebo {

struct MsgqPacket1 : public MsgBase {
  Packet pkg;
};

struct LinearParams1 {
  double scale;
  double offset;
};

GzPropagateG::GzPropagateG()
  : gazebo::ModelPlugin(),
    world_(nullptr), parent_(nullptr),
    rw_thread_alive_(false), msgq_(nullptr),
    swap_r_buffer_(nullptr),
    swap_w_buffer_(nullptr) {
  LOG_INFO << "Create the GzPropagateG... ...";

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

GzPropagateG::~GzPropagateG() {
  // Disconnect from gazebo events
  update_connection_.reset();

  for (auto& p : linear_params_) {
    delete p;
    p = nullptr;
  }
}

// Load the controller
void GzPropagateG::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  std::cout << "Loading the GzPropagateG... ..." << std::endl;

  parent_ = _parent;
  world_  = _parent->GetWorld();

  joints_.resize(LegType::N_LEGS);
  for (auto& leg : joints_)
    leg.resize(JntType::N_JNTS);

  LegType leg = LegType::UNKNOWN_LEG;
  JntType jnt = JntType::UNKNOWN_JNT;
  auto _js_ptr = parent_->GetJoints();
  for (auto& _j : _js_ptr) {
    __parse_jnt_name(_j->GetName(), leg, jnt);
    if ((LegType::UNKNOWN_LEG == leg) || (JntType::UNKNOWN_JNT == jnt))
      LOG_FATAL << "ERROR JOINT NAME...";

    joints_[leg][jnt] = _j;
  }

  if (!_sdf->HasElement("config")) {
    gzthrow("Could not found the 'config' tag from _sdf");
    return;
  }

  // initialize id_2_leg_lut_  from _sdf
  // initialize leg_2_id_lut_  from _sdf
  id_2_leg_lut_.resize(MAX_NODE_NUM);
  leg_2_id_lut_.resize(LegType::N_LEGS);
  FOR_EACH_LEG(l) {
    std::string _tag = LEGTYPE_TOSTRING(l);
    _tag += "_node_id";
    if (!_sdf->HasElement(_tag)) {
      gzthrow("Could not found the '" << _tag << "' tag from _sdf");
      return;
    }
    std::string _val = _sdf->GetElement(_tag)->Get<std::string>();
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
    linear_params_[l] = new LinearParams1;
    FOR_EACH_JNT(j) {
      std::string _tag = LEGTYPE_TOSTRING(l);
      _tag += "_";
      _tag += JNTTYPE_TOSTRING(j);
      linear_params_[l]->scale  = _sdf->GetElement(_tag + "_linear_scale")->Get<double>();
      linear_params_[l]->offset = _sdf->GetElement(_tag + "_linear_offset")->Get<double>();
    }
  }
  // initialize the msgq names
  cmd_msgq_name_        = _sdf->GetElement("cmd_msgq_name")->Get<std::string>();
  leg_node_msgq_name_   = _sdf->GetElement("leg_node_msgq_name")->Get<std::string>();

  ///! launch to the read/write thread.
  swap_r_buffer_   = new boost::lockfree::queue<Packet>(1024);
  swap_w_buffer_   = new boost::lockfree::queue<Packet>(1024);
  rw_thread_alive_ = true;
  msgq_            = MsgQueue::create_instance();
  auto thread_pool = middleware::ThreadPool::create_instance();
  msgq_->create_msgq(leg_node_msgq_name_);
  msgq_->create_msgq(cmd_msgq_name_);
  thread_pool->add(GZ_R_THREAD, &GzPropagateG::msg_r_update, this);
  thread_pool->add(GZ_W_THREAD, &GzPropagateG::msg_w_update, this);
  thread_pool->start(GZ_R_THREAD);
  thread_pool->start(GZ_W_THREAD);

  // listen to the update event (broadcast every simulation iteration)
  update_connection_ =
    event::Events::ConnectWorldUpdateBegin(
        std::bind(&GzPropagateG::event_update, this));

  LOG_INFO << "Has Load the GzPropagateG... ...";
}

/**
 * @brief This method completes the function that convert the Packet
 *        to the form of message which be needed by the specific communication
 *        type, and writing the converted message to robot.
 * @param pkt[in] The Packet object need to convert and write
 * @return Return true if successful, or return false
 */
bool GzPropagateG::write(const Packet& pkt) {
  swap_w_buffer_->push(pkt);
  return true;
}
/**
 * @brief This method must be implemented by subclass, and convert the message
 *        which received from the specific communication type to the form of Packet,
 * @param pkt[out] The Packet object converted from the message
 * @return Return true if read successful, or return false
 */
bool GzPropagateG::read(Packet& pkt) {
  if (swap_r_buffer_->empty()) return false;

  swap_r_buffer_->pop(pkt);
  return true;
}

void GzPropagateG::msg_w_update() {
  MsgqPacket1 _pkg;
  _pkg.msg_id = 0x02; // means from gazebo plugin

  TIMER_INIT
  while (rw_thread_alive_) {
    if (!swap_w_buffer_->empty()) {
      swap_w_buffer_->pop(_pkg.pkg);
      msgq_->write_to_msgq(GZ_W_THREAD, &_pkg, sizeof(_pkg));
    }

    TIMER_CONTROL(10)
  }
}

void GzPropagateG::msg_r_update() {
  MsgqPacket1 _pkg;

  TIMER_INIT
  while (rw_thread_alive_) {
    if (msgq_->read_from_msgq(GZ_R_THREAD, &_pkg, sizeof(_pkg))) {
      swap_r_buffer_->push(_pkg.pkg);
    }

    TIMER_CONTROL(10)
  }
}

void GzPropagateG::event_update() {
  std::cout << "event_update..." << std::endl;
  parent_->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
  return;

  Packet pkt;
  if (read(pkt)) {
    __parse_command_pkg(pkt);
  }

  __update_robot_stats();
}

void GzPropagateG::__update_robot_stats() {
  Packet pkt;
  FOR_EACH_LEG(leg) {
    pkt = {INVALID_BYTE, leg_2_id_lut_[leg], MII_MSG_HEARTBEAT_MSG_1, 8, {0}};
    double pos  = 0;
    short count = 0;
    int offset  = 0;
    for (const auto& jnt : {JntType::KNEE, JntType::HIP, JntType::YAW}) {
      pos   = joints_[leg][jnt]->Position(0);
      count = (pos - linear_params_[leg]->offset) / linear_params_[leg]->scale;
      memcpy(pkt.data + offset, &count, sizeof(short));

      offset += sizeof(short);
    }
  }
}

void GzPropagateG::__parse_command_pkg(const Packet& pkt) {
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

void GzPropagateG::__write_command_to_sim(const Packet& pkt) {
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
    cmd = linear_params_[leg]->scale * count + linear_params_[leg]->offset;

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

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GzPropagateG)

} /* namespace agile_gazebo */

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
