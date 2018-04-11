/*
 * agile_proto.h
 *
 *  Created on: Jul 18, 2017
 *      Author: Bibei
 */

#ifndef INCLUDE_PROTOCOL_AGILE_PROTOCOL_H_
#define INCLUDE_PROTOCOL_AGILE_PROTOCOL_H_

///! cancel the namespace
// namespace middleware {

// if the type of communication is can
// uncomment the follow line
#define PACKET_CAN
#ifndef PACKET_CAN
#define DATA_SIZE (0)
#else
#define DATA_SIZE (8)
#endif

/*#ifndef BYTE
#define BYTE unsigned char
#endif*/

const unsigned char MAX_NODE_NUM = 0x10u;
#define INVALID_BYTE (0x88)

struct Packet {
  unsigned char bus_id;
  unsigned char node_id;
  unsigned char msg_id;
  unsigned char size;
  unsigned char data[DATA_SIZE];
};

/*////////////////////////////////////////////////////////
The CAN message id define:
        10 09 08 07 06 05 04 03 02 01 00
to Node  0| 0|    NODE ID|        MSG ID|
to Group 0| 1|   GROUP ID|        MSG ID|
to Host  1| X|    NODE ID|        MSG ID|
////////////////////////////////////////////////////////*/
#define MII_MSGID_N_BITS      (11)
#define MII_MSGID_N_BYTE      (2)
///! The template of to node , to group and to host message
///! (No setting node_id and msg_id)
#define MII_MSGTPL_2NODE      (0x0000u)
#define MII_MSGTPL_2GRUP      (0x0200u)
#define MII_MSGTPL_2HOST      (0x0400u)

////////////////////////////////////////////////////////
///! The TOOLS for operating message
////////////////////////////////////////////////////////
// Judge the type of message
#define MII_MSG_IS_2HOST(msg)       (((msg) & (0x0400u)) == (0x0400u))
#define MII_MSG_IS_2NODE(msg)       (((msg) & (0x0600u)) == (0x0000u))
#define MII_MSG_IS_2GRUP(msg)       (((msg) & (0x0600u)) == (0x0200u))
// Extracting the node_id, group_id and msg_id from the message
#define MII_MSG_SPLIT_NODEID(msg)   (((msg) & (0x01E0u)) >> 5)
#define MII_MSG_SPLIT_GRUPID(msg)   MII_MSG_SPLIT_NODEID(msg)

#define MII_MSG_SPLIT_MSGID(msg)    ((msg) & (0x001Fu))

// Filling the node_id and msg_id into the given msg which contains the
// message head such as where will to go.
#define __MII_MSG_FILL(msg, node_id, msg_id) \
    ((((msg) & 0x0600u) | (((node_id) & 0x0Fu) << 5)) | ((msg_id) & (0x1Fu)))

// Filling the node id and msg_id into the to host message.
#define MII_MSG_FILL_2HOST_MSG(node_id, msg_id) \
    __MII_MSG_FILL(0x0400u, node_id, msg_id)
// Filling the node_id and msg_id into the to node message.
#define MII_MSG_FILL_2NODE_MSG(node_id, msg_id) \
    __MII_MSG_FILL(0x0000u, node_id, msg_id)
// Filling the node_id and msg_id into the to group message.
#define MII_MSG_FILL_2GRUP_MSG(node_id, msg_id) \
    __MII_MSG_FILL(0x0200u, node_id, msg_id)

// Filling the node_id into the given message
#define MII_MSG_FILL_NODEID(msg, node_id) \
    __MII_MSG_FILL(msg, node_id, MII_MSG_EXTRACT_MSG_ID(msg))
// Filling the msg_id into the given message
#define MII_MSG_FILL_MSGID(msg, msg_id) \
    __MII_MSG_FILL(msg, MII_MSG_EXTRACT_NODE_ID(msg), msg_id)

// #undef __MII_MSG_FILL
////////////////////////////////////////////////////////
///! DEFINE the message id, define by octal
////////////////////////////////////////////////////////

/*////////////////////////////////////////////////////////
THE HEARTBEAT MESSAGE USING BY SENSOR FEEDBACK
////////////////////////////////////////////////////////*/
#define MII_MSG_HEARTBEAT_RESERVE (000u)
/*!
 * In the Leg Node, the count of joint position and TD
 *      00 01| 02 03| 04 05| 06 07
 *       KFE |  HFE |  HAA |  TD
 */
#define MII_MSG_HEARTBEAT_1   (001u)
///! null
#define MII_MSG_HEARTBEAT_2   (002u)
///! null
#define MII_MSG_HEARTBEAT_3   (003u)
///! null
#define MII_MSG_HEARTBEAT_4   (004u)
///! null
#define MII_MSG_HEARTBEAT_5   (005u)
///! null
#define MII_MSG_HEARTBEAT_6   (006u)
///! null
#define MII_MSG_HEARTBEAT_7   (007u)

/*////////////////////////////////////////////////////////
THE COMMON MESSAGE USING BY COMMAND IN JOINT SPACE
////////////////////////////////////////////////////////*/
#define MII_MSG_COMMON_RESERVE    (010u)
///! The joint position command
#define MII_MSG_COMMON_1          (011u)
///! The joint velocity command
#define MII_MSG_COMMON_2          (012u)
///! The joint torque   command
#define MII_MSG_COMMON_3          (013u)
///! The knee and hip joint position and velocity command
#define MII_MSG_COMMON_4          (014u)
///! The yaw joint position and velocity command
#define MII_MSG_COMMON_5          (015u)

/*////////////////////////////////////////////////////////
THE MOTOR MESSAGE USING BY COMMAND IN MOTOR SPACE
The motor command/msg id, offer the interface to control/monitor motor directly.
////////////////////////////////////////////////////////*/
#define MII_MSG_MOTOR_RESERVE     (020u)
///! The motor position command, or motor position feedback
#define MII_MSG_MOTOR_1           (021u)
///! The motor velocity command, or motor velocity feedback
#define MII_MSG_MOTOR_2           (022u)
///! The motor torque command,   or motor torque   feedback
#define MII_MSG_MOTOR_3           (023u)

/*////////////////////////////////////////////////////////
THE DEBUG MESSAGE [RESERVED]
The debug command id, offer the interface to control arm through the user PC.
////////////////////////////////////////////////////////*/
#define MII_MSG_DEBUG_RESERVE     (030u)
///!
#define MII_MSG_DEBUG_1           (031u)
///!
#define MII_MSG_DEBUG_2           (032u)
///!
#define MII_MSG_DEBUG_3           (033u)

/**
 * TEST PROGRAM and RESULTS
 */

// #define TEST_PROTOCOL
#ifdef TEST_PROTOCOL

#include <stdio.h>
int main(int argc, char* argv[]) {
  int count = 0;
  unsigned int id = MII_MSG_FILL_2HOST_MSG(0x02, 0x01);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 0: id: 0x441, toHost: 0x01, node id: 0x02, msg id: 0x01

  id = MII_MSG_FILL_NODEID(id, 0x0A);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 1: id: 0x541, toHost: 0x01, node id: 0x0A, msg id: 0x01

  id = MII_MSG_FILL_MSGID(id, 0x1F);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 2: id: 0x55F, toHost: 0x01, node id: 0x0A, msg id: 0x1F

  id = MII_MSG_FILL_2NODE_MSG(0x0F, 0x11);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 3: id: 0x1F1, toHost: 0x00, node id: 0x0F, msg id: 0x11

  id = MII_MSG_FILL_2GRUP_MSG(0x05, 0xAF);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 4: id: 0x2AF, toHost: 0x00, node id: 0x05, msg id: 0x0F

  id = MII_MSG_FILL_NODEID(id, 0x0A);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 5: id: 0x34F, toHost: 0x00, node id: 0x0A, msg id: 0x0F

  id = MII_MSG_FILL_MSGID(id, 0xFF);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 6: id: 0x35F, toHost: 0x00, node id: 0x0A, msg id: 0x1F

  id = MII_MSG_FILL_MSGID(id, MII_MSG_MOTOR_1);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 7: id: 0x351, toHost: 0x00, node id: 0x0A, msg id: 0x11

  id = MII_MSG_FILL_MSGID(id, MII_MSG_MOTOR_3);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 8: id: 0x353, toHost: 0x00, node id: 0x0A, msg id: 0x13

  id = MII_MSG_FILL_MSGID(id, MII_MSG_DEBUG_RESERVE);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 9: id: 0x358, toHost: 0x00, node id: 0x0A, msg id: 0x18

  id = MII_MSG_FILL_MSGID(id, MII_MSG_DEBUG_2);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 10: id: 0x35A, toHost: 0x00, node id: 0x0A, msg id: 0x1A

  id = MII_MSG_FILL_MSGID(id, MII_MSG_HEARTBEAT_RESERVE);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 11: id: 0x340, toHost: 0x00, node id: 0x0A, msg id: 0x00

  id = MII_MSG_FILL_MSGID(id, MII_MSG_HEARTBEAT_1);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 12: id: 0x341, toHost: 0x00, node id: 0x0A, msg id: 0x01

  id = MII_MSG_FILL_MSGID(id, MII_MSG_COMMON_RESERVE);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 13: id: 0x348, toHost: 0x00, node id: 0x0A, msg id: 0x08

  id = MII_MSG_FILL_MSGID(id, MII_MSG_COMMON_1);
  printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
      count++, id, MII_MSG_IS_2HOST(id), MII_MSG_SPLIT_NODEID(id),
      MII_MSG_SPLIT_MSGID(id));
  // 14: id: 0x349, toHost: 0x00, node id: 0x0A, msg id: 0x09
  return 0;
}
#endif


#endif /* INCLUDE_PROTOCOL_QR_PROTOCOL_H_ */
