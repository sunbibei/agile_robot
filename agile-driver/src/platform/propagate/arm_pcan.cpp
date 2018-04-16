/*
 * arm_pcan.cpp
 *
 *  Created on: Nov 1, 2017
 *      Author: bibei
 */

#include "platform/propagate/arm_pcan.h"
#include "foundation/cfg_reader.h"

namespace agile_robot {

const int MAX_COUNT = 10;

ArmPcan::ArmPcan(const std::string& l)
  : PCanPropa(l) {
  send_msg_.MSGTYPE = PCAN_MESSAGE_STANDARD;
}

ArmPcan::~ArmPcan() {
  ;
}

bool ArmPcan::write(const Packet& pkt) {
  if (!connected_) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }
  send_msg_.MSGTYPE = PCAN_MESSAGE_STANDARD;
  send_msg_.ID      = MII_MSG_FILL_2NODE_MSG(pkt.node_id, pkt.msg_id);
  send_msg_.LEN     = pkt.size;
  memset(send_msg_.DATA, '\0', 8 * sizeof(BYTE));
  memcpy(send_msg_.DATA, pkt.data, send_msg_.LEN * sizeof(BYTE));

  if (false)
    printf("  - ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)send_msg_.ID, (int)send_msg_.LEN,
      (int)send_msg_.DATA[0], (int)send_msg_.DATA[1],
      (int)send_msg_.DATA[2], (int)send_msg_.DATA[3],
      (int)send_msg_.DATA[4], (int)send_msg_.DATA[5],
      (int)send_msg_.DATA[6], (int)send_msg_.DATA[7]);

  // try to 10 times
  for (int counter = 0; counter < MAX_COUNT; ++counter) {
    tmp_pcan_status_ = CAN_Write(pcan_config_.channel, &send_msg_);
    if (PCAN_ERROR_OK != tmp_pcan_status_) {
      LOG_DEBUG << "(" << counter + 1 << "/" << MAX_COUNT
          << ") Write CAN message FAIL, status code: " << tmp_pcan_status_
          << ", Waiting 50ms... ...";
      // Waiting 50ms
      usleep(50000);
    } else
      return true;
  }

  LOG_ERROR << "Write CAN FAIL!!!";
  return false;
}

bool ArmPcan::read(Packet& pkt) {
  if (!connected_) {
    // LOG_FIRST_N(WARNING, 10000) << "The pcan has not been launched, or initialized fail.";
    return false;
  }

  int counter = 0;
  memset(&recv_msg_, '\0', sizeof(TPCANMsg));

  while (PCAN_ERROR_OK != (tmp_pcan_status_ = CAN_Read(pcan_config_.channel, &recv_msg_, NULL))) {
    if (++counter <= MAX_COUNT) {
      //LOG_DEBUG << "read again!(" << counter << "/"
      //    << MAX_COUNT << "), error code: " << tmp_pcan_status_;
      usleep(5000);
    } else {
      LOG_ERROR << "The pcan channel has read fail!";
      return false;
    }
  }
  if (PCAN_ERROR_OK != tmp_pcan_status_) {
    // fail to read, we are trying to reset the pcan system.
    CAN_Reset(pcan_config_.channel);
    return false;
  }
  // LOG_ERROR << "test1";
  // I don't known what happen to here, It always receives a odd message with id
  // is 0x06. Everything is ok, and the result is wrong!
  // I have no idea, so here is compromise way, we will reset the pcan channel
  // when the odd message is coming.
  counter = 0;
  int reset_count = 0;
  while (!MII_MSG_IS_2HOST(recv_msg_.ID)) {
    // LOG_ERROR << "test2";
    // This is a compromise way that to compatible with the old protocol.
    if (recv_msg_.ID == 0x8) {
      if ((0x5 != recv_msg_.LEN) || (0x80 != (recv_msg_.DATA[0] & 0xf0))) return false;

      pkt.bus_id  = bus_id_;
      pkt.node_id = 0x08;
      pkt.msg_id  = ((recv_msg_.DATA[0] & 0x0f) - 0x01);
      pkt.size    = 2;
      memset(pkt.data, '\0', 8 * sizeof(char));
      pkt.data[0] = recv_msg_.DATA[3];
      pkt.data[1] = recv_msg_.DATA[4];
      if (false && 0x02 == pkt.msg_id)
        printf("  - ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
          (int)recv_msg_.ID, (int)recv_msg_.LEN,
          (int)recv_msg_.DATA[0], (int)recv_msg_.DATA[1],
          (int)recv_msg_.DATA[2], (int)recv_msg_.DATA[3],
          (int)recv_msg_.DATA[4]);
      return true;
    }
    if (++reset_count <= MAX_COUNT) {
      ;// LOG_EVERY_N(WARNING, 10) << "It read odd message"
      //     << ", and the host could not parse, read again... ...";
    } else {
      // LOG_EVERY_N(ERROR, 10) << "The pcan channel always read odd messages, and we give up read!"
      //     << "Now we are trying to reset the pcan system.";
      CAN_Reset(pcan_config_.channel);
      reset_count = 0;
    }

    if (++counter >= 2*MAX_COUNT) return false;
    tmp_pcan_status_ = CAN_Read(pcan_config_.channel, &recv_msg_, NULL);
  }

  if (false)
    printf("  - ID:0x%03X LEN:%1x DATA:0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
      (int)recv_msg_.ID, (int)recv_msg_.LEN,
      (int)recv_msg_.DATA[0], (int)recv_msg_.DATA[1],
      (int)recv_msg_.DATA[2], (int)recv_msg_.DATA[3],
      (int)recv_msg_.DATA[4], (int)recv_msg_.DATA[5],
      (int)recv_msg_.DATA[6], (int)recv_msg_.DATA[7]);

  pkt.bus_id  = bus_id_;
  pkt.node_id = MII_MSG_SPLIT_NODEID(recv_msg_.ID);
  pkt.msg_id  = MII_MSG_SPLIT_MSGID(recv_msg_.ID);
  pkt.size    = recv_msg_.LEN;
  memset(pkt.data, '\0', 8 * sizeof(char));
  memcpy(pkt.data, recv_msg_.DATA, pkt.size * sizeof(char));
  return true;
}

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(agile_robot::ArmPcan, Label)
