/*
 * propagate_manager.h
 *
 *  Created on: Aug 29, 2017
 *      Author: bibei
 */

#ifndef INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_
#define INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_

#include "platform/proto/agile_proto.h"
#include "foundation/internal/resource_manager.h"
#include "platform/propagate/propagate.h"

#include <chrono>
// #include <boost/lockfree/queue.hpp>
#include <mutex>

namespace agile_robot {
class PropagateManager: public internal::ResourceManager<Propagate> {
  SINGLETON_DECLARE(PropagateManager)
public:

  /*!
   * @brief This method will initialize something.
   * @param r_freq   The read  frequency.
   * @param w_freq   The write frequency.
   */
  bool init(int r_freq = 10000, int w_freq = 10000);
  /**
   * @brief This method will starts a propagate thread for reading and writing Packets
   * @return Return true if everything is right, or return false.
   */
  bool run();

  /**
   * @brief These methods offer the public function for user, User could obtains
   *        the incoming data through the follow @readPackets methods and sends
   *        the outgoing data through the @writePackets methods.
   */
  bool readPackets (std::vector<Packet>&);
  bool writePackets(const std::vector<Packet>&);

  // FOR DEBUG
  void print();

private:
  /**
   * @brief This method is the read function for propagate thread.
   */
  void updateRead();
  /**
   * @brief This method is the write function for propagate thread.
   */
  void updateWrite();

private:
  std::vector<MiiPtr<Propagate>> propa_list_by_bus_;
  bool                           thread_alive_;
  // size_t                      pkts_queue_size;

  std::chrono::microseconds      w_interval_;
  std::chrono::microseconds      r_interval_;

  ///! Maybe we should use the lock-free queue instead of std::vector.
//  boost::lockfree::queue<Packet>* pkts_queue_4_send_;
//  boost::lockfree::queue<Packet>* pkts_queue_4_recv_;

  std::mutex            lock_4_send_;
  std::mutex            lock_4_recv_;
  std::vector<Packet>   pkts_queue_4_send_;
  std::vector<Packet>   pkts_queue_4_recv_;
};

} /* namespace middleware */

#endif /* INCLUDE_SYSTEM_ROBOT_PROPAGATE_MANAGER_H_ */
