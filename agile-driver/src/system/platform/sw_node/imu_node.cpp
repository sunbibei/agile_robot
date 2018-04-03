/*
 * imu_node.cpp
 *
 *  Created on: Oct 11, 2017
 *      Author: bibei
 */

#include "system/platform/sw_node/imu_node.h"

#include "repository/resource/imu_sensor.h"
#include "foundation/cfg_reader.h"

namespace middleware {

ImuNode::ImuNode(const std::string& __l)
: SWNode(__l), imu_sensor_(nullptr) {

}

bool ImuNode::auto_init() {
  if (!SWNode::auto_init())     return false;
  auto cfg = MiiCfgReader::instance();

  std::string tmp_str;
  cfg->get_value_fatal(getLabel(), "label", tmp_str);
  imu_sensor_ = Label::getHardwareByName<ImuSensor>(tmp_str);

  return (nullptr != imu_sensor_);
}

ImuNode::~ImuNode() {
  // Nothing to do here.
}

void ImuNode::handleMsg(const Packet& pkt) {
  /*for (auto v : pkt.data) {
    printf("0x%02X ", v);
  }
  printf("\n");
  return;*/
  static short tmp[4]   = {0};
  static double vals[4] = {0};
  switch (pkt.msg_id) {
  case MII_USB_UP_ID_TIME:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_ACC:
    for (int i = 0; i < 3; ++i) {
      memcpy(tmp + i, pkt.data + 2*i, sizeof(short));
    }
    ///! 0.004785156 = 16 * g / 32768 = 16 * 9.8 / 32768
    imu_sensor_->updateLinearAcc(((double)tmp[0])*0.004785156,
        ((double)tmp[1])*0.004785156, ((double)tmp[2])*0.004785156);
    break;
  case MII_USB_UP_ID_ANG_VEL:
    for (int i = 0; i < 3; ++i) {
      memcpy(tmp + i, pkt.data + 2*i, sizeof(short));
    }
    ///! 0.001064724 = 2000 / 32768 * \pi / 180 = 2000/32768*3.14/180
    imu_sensor_->updateAngVel(((double)tmp[0])*0.001064724,
        ((double)tmp[1])*0.001064724, ((double)tmp[2])*0.001064724);
    break;
  case MII_USB_UP_ID_ANG:
    for (int i = 0; i < 3; ++i) {
      memcpy(tmp + i, pkt.data + 2*i, sizeof(short));
      vals[i] = tmp[i] / 32768.0 * 180.0;
    }
    ///！ 0.000095825 = 180 / 32768 * \pi / 180 = 3.14 / 32768
    imu_sensor_->updateOrientation(vals[0],
        vals[1], vals[2], 1);
    break;
  case MII_USB_UP_ID_MAGNIC:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_STATUS:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_PA_CM :
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_LON_LAT:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_YAW:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_QUATERNION:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  case MII_USB_UP_ID_ACCURACY:
    LOG_WARNING << "NO IMPLEMENT!";
    break;
  default:
    LOG_WARNING << "What fucking message!";
  }
}

bool ImuNode::requireCmdDeliver() { return false; }

} /* namespace middleware */

#include <class_loader/class_loader_register_macro.h>
CLASS_LOADER_REGISTER_CLASS(middleware::ImuNode, Label)
