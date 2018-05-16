/*
 * test_node.cpp
 *
 *  Created on: Apr 24, 2018
 *      Author: bibei
 */

#include <platform/sw_node/test_node.h>
#include <platform/proto/agile_proto.h>

#include <foundation/cfg_reader.h>

#include <stdio.h>

namespace agile_robot {

TestNode::TestNode()
  : SWNode("TestNode"),
    debug_status_(DebugStatus::DS_INVALID),
    /*n_total_msgs_(0),*/ N_total_msgs_(0), loop_dt(0),
    is_the_one_(false), which_one_(INVALID_BYTE) {
  ;
}

bool TestNode::auto_init() {
  if (!SWNode::auto_init()) return false;

  auto cfg = MiiCfgReader::instance();
  cfg->get_value(getLabel(), "size",      N_total_msgs_);
  cfg->get_value(getLabel(), "path",      out_path_);
  cfg->get_value(getLabel(), "filename",  out_filename_);
  cfg->get_value(getLabel(), "the_one",   is_the_one_);
  cfg->get_value(getLabel(), "which_one", which_one_);

  pkt_list_.reserve(N_total_msgs_);
  debug_status_ = DebugStatus::DS_INIT;
  return true;
}

TestNode::~TestNode() {
  LOG_WARNING << "~TestNode";
  if (pkt_list_.empty()) return;

  LOG_WARNING << "Write the msg to file...";
  writeMsgToFile();
}

void TestNode::handleMsg(const Packet& pkt) {
  switch (debug_status_) {
  case DebugStatus::DS_WAITING:
    if (MII_MSG_DEBUG_1 == pkt.msg_id) {
      debug_status_ = DebugStatus::DS_RECV_READY;
      t0_     = std::chrono::high_resolution_clock::now();
      LOG_WARNING << "I'm ready to recv";
    }
    break;
  case DebugStatus::DS_RECV_READY:
    if (MII_MSG_DEBUG_2 == pkt.msg_id) {
      debug_status_ = DebugStatus::DS_DATA_SAVE_READY;

      node_pkt_list_.reserve(pkt_list_.size());

      LOG_WARNING << "I'm ready to recv feedback";
    } else if (MII_MSG_DEBUG_1 == pkt.msg_id) {
      debug_status_ = DebugStatus::DS_READY_END;
    } else {
      curr_t_ = std::chrono::high_resolution_clock::now();
      loop_dt = std::chrono::duration_cast<std::chrono::milliseconds>
          (curr_t_ - t0_).count();

      LOG_EVERY_N(WARNING, 100) << "recv...";

      pkt_list_.push_back({loop_dt, pkt});
    }
    break;
  case DebugStatus::DS_DATA_SAVE_READY:
    if (MII_MSG_DEBUG_2 == pkt.msg_id) {
      debug_status_ = DebugStatus::DS_READY_END;
    } else {
      LOG_EVERY_N(WARNING, 100) << "feedback...";

      node_pkt_list_.push_back(pkt);
    }
    break;
  case DebugStatus::DS_READY_END:
  {
    t1s_.push_back(std::chrono::high_resolution_clock::now());
    std::string ending_info = "Passed the Test. total duration = ";
    for (const auto& t1 : t1s_) {
      loop_dt = std::chrono::duration_cast<std::chrono::milliseconds>
                 (t1 - t0_).count();
      ending_info += (std::to_string(loop_dt) + " ");
    }
    LOG_WARNING << ending_info;

    writeMsgToFile();
    pkt_list_.clear();
    debug_status_ = DebugStatus::DS_INIT;
    break;
  }
//  case DebugStatus::DS_INIT:
//  case DebugStatus::N_DS:
  default:
    break;
  } // end switch (debug_status_)
//  if (DebugStatus::DS_RECV_READY != debug_status_) {
//    if ((MII_MSG_DEBUG_1 == pkt.msg_id) && (DebugStatus::DS_WAITING == debug_status_) ) {
//      debug_status_ = DebugStatus::DS_RECV_READY;
//      t0_     = std::chrono::high_resolution_clock::now();
//    }
//    return;
//  }

//  curr_t_ = std::chrono::high_resolution_clock::now();
//  loop_dt = std::chrono::duration_cast<std::chrono::milliseconds>
//      (curr_t_ - t0_).count();
//
//  pkt_list_.push_back({loop_dt, pkt});
//  if (pkt_list_.size() == N_total_msgs_) {
//    t1_ = std::chrono::high_resolution_clock::now();
//    loop_dt = std::chrono::duration_cast<std::chrono::milliseconds>
//               (t1_ - t0_).count();
//    LOG_WARNING << "Passed the Test. total duration = " << loop_dt;
//
//    writeMsgToFile();
//    pkt_list_.clear();
//    debug_status_ = DebugStatus::DS_INIT;
//  }
}

void TestNode::checkDebugStatus(const Packet& pkt) {
//  switch (debug_status_) {
//  case DebugStatus::DS_WAITING:
//    if (MII_MSG_DEBUG_1 == pkt.msg_id) {
//      debug_status_ = DebugStatus::DS_RECV_READY;
//      t0_     = std::chrono::high_resolution_clock::now();
//    }
//    break;
//  case DebugStatus::DS_RECV_READY:
//    curr_t_ = std::chrono::high_resolution_clock::now();
//    loop_dt = std::chrono::duration_cast<std::chrono::milliseconds>
//        (curr_t_ - t0_).count();
//
//    pkt_list_.push_back({loop_dt, pkt});
//    break;
//  case DebugStatus::DS_DATA_SAVE_READY:
//  case DebugStatus::DS_INIT:
//  case DebugStatus::DS_READY_END:
//    t1_ = std::chrono::high_resolution_clock::now();
//    loop_dt = std::chrono::duration_cast<std::chrono::milliseconds>
//               (t1_ - t0_).count();
//    LOG_WARNING << "Passed the Test. total duration = " << loop_dt;
//
//    writeMsgToFile();
//    pkt_list_.clear();
//    debug_status_ = DebugStatus::DS_INIT;
//    break;
//  default:
//    break;
//  } // end switch (debug_status_)
}

bool TestNode::requireCmdDeliver() {
  return is_the_one_;
}

bool TestNode::generateCmd(std::vector<Packet>& pkts) {
  if (DebugStatus::DS_INIT != debug_status_) return false;

  LOG_WARNING << "Enter";
  Packet pkt = {ARM_BUS, which_one_, MII_MSG_DEBUG_1, 2, {0}};
  memcpy(pkt.data, &N_total_msgs_, sizeof(N_total_msgs_));
  // pkt.data[sizeof(N_total_msgs_)] = which_one_;

  pkts.push_back(pkt);
  debug_status_ = DebugStatus::DS_WAITING;
  return true;
}

bool TestNode::writeMsgToFile() {
  FILE* ofd = fopen((out_path_ + "/" + out_filename_).c_str(), "w");
  if (NULL == ofd) {
    LOG_ERROR << "Write these message into file fail!";
    return false;
  }

  fprintf(ofd, "Time BusID NodeID MsgID LEN D[0] D[1] D[2] D[3] D[4] D[5] D[6] D[7]\n");
//fprintf(ofd, "%04ld  0x%02X  0x%02X   0x%02X  %1x  0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n");
//fprintf(ofd, "0000  0x01  0x01   0x01  8  0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x07\n");
  for (const auto& p : pkt_list_) {
    fprintf(ofd, "%04ld  0x%02X  0x%02X   0x%02X  %1x  0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
        p.dt, p.pkt.bus_id, p.pkt.node_id, p.pkt.msg_id, p.pkt.size,
        (int)p.pkt.data[0], (int)p.pkt.data[1], (int)p.pkt.data[2], (int)p.pkt.data[3],
        (int)p.pkt.data[4], (int)p.pkt.data[5], (int)p.pkt.data[6], (int)p.pkt.data[7]);
  }

  fclose(ofd);
  return true;
}

} /* namespace agile_robot */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::TestNode, Label)
