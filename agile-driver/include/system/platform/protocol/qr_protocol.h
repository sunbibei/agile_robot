/*
 * qr_protocol.h
 *
 *  Created on: Jul 18, 2017
 *      Author: silence
 */

#ifndef INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_
#define INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_

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

struct Packet {
  unsigned char bus_id;
  unsigned char node_id;
  unsigned char msg_id;
  unsigned char size;
  unsigned char data[DATA_SIZE];
};

#define INVALID_BYTE (0x88)

/*////////////////////////////////////////////////////////
The CAN message id define:
        10 09 08 07 06 05 04 03 02 01 00
to Node  0| 0|    NODE ID|        MSG ID|
to Group 0| 1|   GROUP ID|        MSG ID|
to Host  1| X|    NODE ID|        MSG ID|
////////////////////////////////////////////////////////*/
#define MII_MSG_ID_BITS_SIZE      (11)
#define MII_MSG_ID_BYTE_SIZE      (2)
///! The template of to node , to group and to host message(No setting node_id and msg_id)
#define MII_MSG_TEMPLATE_TO_NODE      (0x0000u)
#define MII_MSG_TEMPLATE_TO_GROUP     (0x0200u)
#define MII_MSG_TEMPLATE_TO_HOST      (0x0400u)

////////////////////////////////////////////////////////
///! The UTILS for operating message
////////////////////////////////////////////////////////
// Judge the type of message
#define MII_MSG_IS_TO_HOST(msg)       (((msg) & (0x0400u)) == (0x0400u))
#define MII_MSG_IS_TO_NODE(msg)       (((msg) & (0x0600u)) == (0x0000u))
#define MII_MSG_IS_TO_GROUP(msg)      (((msg) & (0x0600u)) == (0x0200u))
// Extracting the node_id, group_id and msg_id from the message
#define MII_MSG_EXTRACT_NODE_ID(msg)  (((msg) & (0x01E0u)) >> 5)
#define MII_MSG_EXTRACT_GROUP_ID(msg) MII_MSG_EXTRACT_NODE_ID(msg)
#define MII_MSG_EXTRACT_MSG_ID(msg)    ((msg) & (0x001Fu))
// Filling the node_id and msg_id into the given msg which contains the
// message head such as where will to go.
#define __MII_MSG_FILL_TO_MSG(msg, node_id, msg_id) \
    ((((msg) & 0x0600u) | (((node_id) & 0x0Fu) << 5)) | ((msg_id) & (0x1Fu)))
// Filling the node id and msg_id into the to host message.
#define MII_MSG_FILL_TO_HOST_MSG(node_id, msg_id) \
    __MII_MSG_FILL_TO_MSG(0x0400u, node_id, msg_id)
// Filling the node_id and msg_id into the to node message.
#define MII_MSG_FILL_TO_NODE_MSG(node_id, msg_id) \
    __MII_MSG_FILL_TO_MSG(0x0000u, node_id, msg_id)
// Filling the node_id and msg_id into the to group message.
#define MII_MSG_FILL_TO_GROUP_MSG(node_id, msg_id) \
    __MII_MSG_FILL_TO_MSG(0x0200u, node_id, msg_id)
// Filling the node_id into the given message
#define MII_MSG_FILL_NODE_ID(msg, node_id) \
    __MII_MSG_FILL_TO_MSG(msg, node_id, MII_MSG_EXTRACT_MSG_ID(msg))
// Filling the msg_id into the given message
#define MII_MSG_FILL_MSG_ID(msg, msg_id) \
    __MII_MSG_FILL_TO_MSG(msg, MII_MSG_EXTRACT_NODE_ID(msg), msg_id)


////////////////////////////////////////////////////////
///! define the message id, define by octal
////////////////////////////////////////////////////////
#define MII_MSG_HEARTBEAT_RESERVE (000u)
///! null
#define MII_MSG_HEARTBEAT_MSG     (001u)
/*!
 * In the Leg Node, joint position(in radian), td(in count)
 *      00 01 02 03 04 05 06 07
 *      knee | hip | yaw | td
 * In the Power Node: (in A)
 *      00 01 02 03 04 05 06 07
 *       fl  |  fr |  hl |  hr
 */
#define MII_MSG_HEARTBEAT_MSG_1   (002u)
///! Return the current of power node 1, or motor position 
#define MII_MSG_HEARTBEAT_MSG_2   (003u)
///! Return the current of power node 2
#define MII_MSG_HEARTBEAT_MSG_3   (004u)
///! Return the current of power node 3
#define MII_MSG_HEARTBEAT_MSG_4   (005u)
///! Return the overall current of power node,
#define MII_MSG_HEARTBEAT_MSG_5   (006u)
///! Return the error information of power node
#define MII_MSG_HEARTBEAT_MSG_6   (007u)

///! null
#define MII_MSG_COMMON_RESERVE    (010u)
///! The joint position command
#define MII_MSG_COMMON_DATA_1     (011u)
///! The joint velocity command
#define MII_MSG_COMMON_DATA_2     (012u)
///! The joint torque   command
#define MII_MSG_COMMON_DATA_3     (013u)
///! The knee and hip joint position and velocity command
#define MII_MSG_COMMON_DATA_4     (014u)
///! The yaw joint position and velocity command
#define MII_MSG_COMMON_DATA_5     (015u)

///! The motor command/msg id, offer the interface to control/monitor motor directly.
#define MII_MSG_MOTOR_RESERVE     (020u)
///! The motor position command, or motor position feedback
#define MII_MSG_MOTOR_CMD_1       (021u)
///! The motor velocity command, or motor velocity feedback
#define MII_MSG_MOTOR_CMD_2       (022u)
///! The motor torque command,   or motor torque   feedback
#define MII_MSG_MOTOR_CMD_3       (023u)

///! The debug command id, offer the interface to control arm through the user PC.
#define MII_MSG_DEBUG_RESERVE     (030u)
///!
#define MII_MSG_DEBUG_SIGNAL_1    (031u)
///!
#define MII_MSG_DEBUG_SIGNAL_2    (032u)
///!
#define MII_MSG_DEBUG_SIGNAL_3    (033u)


////////////////////////////////////////////////////////////////////////
///////////////// The IMU communication protocol by the USB
////////////////////////////////////////////////////////////////////////

/*////////////////////////////////////////////////////////
The USB message format define:
           0     1      2     3       4       5   6   7   8
to IMU   0xFF | 0xAA | ID | DATA_0 | DATA_1 |
to Host  0x55 | UP_ID|                 DATA             | SUM
////////////////////////////////////////////////////////*/
#define USB_UP_MESSAGE_SIZE        (11)
#define USB_UP_MESSAGE_DATA_SIZE   (8)

#define MII_USB_UP_HEADER          (0x55)

#define MII_USB_UP_ID_TIME         (0x50)
#define MII_USB_UP_ID_ACC          (0x51)
#define MII_USB_UP_ID_ANG_VEL      (0x52)
#define MII_USB_UP_ID_ANG          (0x53)
#define MII_USB_UP_ID_MAGNIC       (0x54)
#define MII_USB_UP_ID_STATUS       (0x55)
#define MII_USB_UP_ID_PA_CM        (0x56)
#define MII_USB_UP_ID_LON_LAT      (0x57)
#define MII_USB_UP_ID_YAW          (0x58)
#define MII_USB_UP_ID_QUATERNION   (0x59)
#define MII_USB_UP_ID_ACCURACY     (0x5A)

#define MII_USB_DOWN_HEADER        (0xFFAA)

#define MII_USB_DOWN_ID_SAVE       (0x00)
#define MII_USB_DOWN_ID_POSTBACK   (0x02)
#define MII_USB_DOWN_ID_RATE       (0x03)
#define MII_USB_DOWN_ID_BAUD       (0x04)

///! This macro method passes a MII_USB_UP_ID_XXX as id parameter, and passes
///! a short type as the out RSWL and RSWH data.
#define MII_USB_SET_POSTBACK_ID(rsw, id) ((rsw) |= (0x01 << ((id) & (0x0F))))

#define MII_USB_DOWN_RATE_0_1HZ    (0x01)
#define MII_USB_DOWN_RATE_0_5HZ    (0x02)
#define MII_USB_DOWN_RATE_1HZ      (0x03)
#define MII_USB_DOWN_RATE_2HZ      (0x04)
#define MII_USB_DOWN_RATE_5HZ      (0x05)
#define MII_USB_DOWN_RATE_10HZ     (0x06)
#define MII_USB_DOWN_RATE_20HZ     (0x07)
#define MII_USB_DOWN_RATE_50HZ     (0x08)
#define MII_USB_DOWN_RATE_100HZ    (0x09)
#define MII_USB_DOWN_RATE_200HZ    (0x0A)
#define MII_USB_DOWN_RATE_ONCE     (0x0B)
#define MII_USB_DOWN_RATE_0HZ      (0x0C)

#define MII_USB_DOWN_BAUD_2400     (0x00)
#define MII_USB_DOWN_BAUD_4800     (0x01)
#define MII_USB_DOWN_BAUD_9600     (0x02)
#define MII_USB_DOWN_BAUD_19200    (0x03)
#define MII_USB_DOWN_BAUD_38400    (0x04)
#define MII_USB_DOWN_BAUD_57600    (0x05)
#define MII_USB_DOWN_BAUD_115200   (0x06)
#define MII_USB_DOWN_BAUD_230400   (0x07)
#define MII_USB_DOWN_BAUD_460800   (0x08)
#define MII_USB_DOWN_BAUD_921600   (0x09)

#define MII_USB_DOWN_BAUD(b) MII_USB_DOWN_BAUD_##b

//} /* end namespace middleware */

/**
 * TEST PROGRAM and RESULTS
 */

// #define TEST_PROTOCOL
#ifdef TEST_PROTOCOL

 #include <stdio.h>
 using namespace middleware;

 int main(int argc, char* argv[]) {
   int count = 0;
   unsigned int id = MII_MSG_FILL_TO_HOST_MSG(0x02, 0x01);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 0: id: 0x441, toHost: 0x01, node id: 0x02, msg id: 0x01

   id = MII_MSG_FILL_NODE_ID(id, 0x0A);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 1: id: 0x541, toHost: 0x01, node id: 0x0A, msg id: 0x01

   id = MII_MSG_FILL_MSG_ID(id, 0x1F);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 2: id: 0x55F, toHost: 0x01, node id: 0x0A, msg id: 0x1F

   id = MII_MSG_FILL_TO_NODE_MSG(0x0F, 0x11);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 3: id: 0x1F1, toHost: 0x00, node id: 0x0F, msg id: 0x11

   id = MII_MSG_FILL_TO_GROUP_MSG(0x05, 0xAF);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 4: id: 0x2AF, toHost: 0x00, node id: 0x05, msg id: 0x0F

   id = MII_MSG_FILL_NODE_ID(id, 0x0A);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 5: id: 0x34F, toHost: 0x00, node id: 0x0A, msg id: 0x0F

   id = MII_MSG_FILL_MSG_ID(id, 0xFF);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 6: id: 0x35F, toHost: 0x00, node id: 0x0A, msg id: 0x1F

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_MOTOR_CMD_1);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 7: id: 0x351, toHost: 0x00, node id: 0x0A, msg id: 0x11

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_MOTOR_CMD_3);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 8: id: 0x353, toHost: 0x00, node id: 0x0A, msg id: 0x13

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_DEBUG_RESERVE);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 9: id: 0x358, toHost: 0x00, node id: 0x0A, msg id: 0x18

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_DEBUG_SIGNAL_2);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 10: id: 0x35A, toHost: 0x00, node id: 0x0A, msg id: 0x1A

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_HEARTBEAT_RESERVE);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 11: id: 0x340, toHost: 0x00, node id: 0x0A, msg id: 0x00

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_HEARTBEAT_MSG);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 12: id: 0x341, toHost: 0x00, node id: 0x0A, msg id: 0x01

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_COMMON_RESERVE);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 13: id: 0x348, toHost: 0x00, node id: 0x0A, msg id: 0x08

   id = MII_MSG_FILL_MSG_ID(id, MII_MSG_COMMON_DATA_1);
   printf("%d: id: 0x%02X, toHost: 0x%02X, node id: 0x%02X, msg id: 0x%02X\n",
       count++, id, MII_MSG_IS_TO_HOST(id), MII_MSG_EXTRACT_NODE_ID(id),
       MII_MSG_EXTRACT_MSG_ID(id));
   // 14: id: 0x349, toHost: 0x00, node id: 0x0A, msg id: 0x09
   return 0;
 }
#endif


#endif /* INCLUDE_MIDDLEWARE_PROTOCOL_QR_PROTOCOL_H_ */
