/*
 * hw_manager.h
 *
 *  Created on: Aug 26, 2017
 *      Author: silence
 */

#ifndef INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_



#include "platform/sw_node/sw_node.h"

#include <chrono>
#include <foundation/internal/resource_manager.h>

namespace agile_robot {

class Master {
  SINGLETON_DECLARE(Master)
public:
  /*!
   * @brief Initialize the master.
   * @param mr     The read  frequency of master
   * @param mw     The write frequency of master
   * @param pr     The read  frequency of PropagateManager
   * @param pw     The write frequency of PropagateManager
   */
  bool init(int mr = 10000, int mw = 10000, int pr = 10000, int pw = 10000);
  /**
   * @brief This method will starts a tick thread for deliver Packets
   * @return Return true if everything is right, or return false.
   */
  bool run();

private:
  // read operator of every tick
  void tick_r();
  // write operator of every tick
  void tick_w();
  // The interval time between twice RW.(in ms)
  // std::chrono::milliseconds  tick_interval_;
  bool                       thread_alive_;

  class PropagateManager*    propagate_manager_;
  class SWNodeManager*       sw_node_manager_;

private:
  std::vector<Packet> queue_4_w_;
  std::vector<Packet> queue_4_r_;

  std::chrono::microseconds      w_interval_;
  std::chrono::microseconds      r_interval_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_HW_MANAGER_H_ */
