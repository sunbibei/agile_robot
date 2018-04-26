/*
 * test_node.h
 *
 *  Created on: Apr 24, 2018
 *      Author: bibei
 */

#ifndef INCLUDE_PLATFORM_SW_NODE_TEST_NODE_H_
#define INCLUDE_PLATFORM_SW_NODE_TEST_NODE_H_

#include "sw_node.h"
#include <chrono>

namespace agile_robot {

class TestNode: public SWNode {
public:
  TestNode();
  virtual bool auto_init() override;

  virtual ~TestNode();

  /*!
   * @brief The sub-class must be complete this method, parse the Packet.
   */
  virtual void handleMsg(const Packet&) override;

  virtual bool requireCmdDeliver() override;
  /*!
   * @brief The new command packet will be pushed back into queue;
   *        If generate the Command packet, return true. Or return false.
   */
  virtual bool generateCmd(std::vector<Packet>&) override;

private:
  bool writeMsgToFile();
  void checkDebugStatus(const Packet&);

protected:
  ///! flag for leg node whether is ready or not.
  enum DebugStatus {
    DS_INVALID = -1,
    DS_INIT,
    DS_WAITING,
    DS_RECV_READY,
    DS_DATA_SAVE_READY,
    DS_READY_END,
    N_DS,
  } debug_status_;
  ///! the counter for receive message.
  // unsigned int n_total_msgs_;
  ///! the total number of receive message.
  unsigned int N_total_msgs_;
  ///! the starting time.
  std::chrono::high_resolution_clock::time_point t0_;
  ///! the ending   time.
  std::vector<std::chrono::high_resolution_clock::time_point> t1s_;
  ///! the each received Packet
  struct _EachPkt {
    int64_t dt;
    Packet  pkt;
  };
  std::vector<_EachPkt> pkt_list_;
  std::vector<Packet>   node_pkt_list_;

  std::chrono::high_resolution_clock::time_point curr_t_;
  int64_t loop_dt;

  ///! About the output file.
  std::string out_path_;
  std::string out_filename_;

  ///! Whether is the one node that send the start message to Leg Node.
  bool is_the_one_;
  ///! Test which Leg Node.
  unsigned char which_one_;
};

} /* namespace agile_robot */

#endif /* INCLUDE_PLATFORM_SW_NODE_TEST_NODE_H_ */
